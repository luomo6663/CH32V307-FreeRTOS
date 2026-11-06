/* TIM3 hardware PWM, with complete frame commits in the low interval. */
#include "servo.h"
#include "ch32v30x.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#define QUEUE_SIZE 16U
#define FRAME_US (1000000U / SERVO_PWM_FREQUENCY_HZ)
#define COMMIT_US 3000U
/* Leave a wrap guard even with code_2's 3333 us frame. */
#define WRAP_GUARD_US ((FRAME_US > COMMIT_US + 2000U) ? 1000U : 100U)
typedef struct {
    uint16_t pulse, target, start, low, high;
    uint32_t elapsed, duration;
    uint8_t motion, enabled, test_phase;
} channel_t;
enum { CMD_MOVE, CMD_CENTER, CMD_HOLD, CMD_OFF, CMD_SWEEP, CMD_SIDE };
typedef struct {
    uint8_t type, mask, side;
    uint16_t us[3], low, high;
    uint32_t ms;
} command_t;
static const uint16_t s_min[2][3] = ROBOT_SERVO_MIN_US_TABLE;
static const uint16_t s_center[2][3] = ROBOT_SERVO_CENTER_US_TABLE;
static const uint16_t s_max[2][3] = ROBOT_SERVO_MAX_US_TABLE;
static const uint8_t s_installed[2] = {
    ROBOT_SERVO_RIGHT_INSTALLED_MASK, ROBOT_SERVO_LEFT_INSTALLED_MASK
};
static const char *const s_headers[2][3] = {{"H21","H3","H4"},{"H5","H6","H7"}};
static channel_t s_bank[2][3];
static command_t s_queue[QUEUE_SIZE];
static uint8_t s_head, s_count, s_side, s_requested_side, s_valid;
static uint32_t s_timer_hz;
static TickType_t s_last_tick;
/* ISR mailbox, protected by task critical sections. */
static volatile uint16_t s_frame[3];
static volatile uint8_t s_frame_side, s_hardware_side;
static servo_status_t s_status;

static void Servo_WriteSide(uint8_t side)
{
    uint8_t high = side ^ (ROBOT_SERVO_SIDE_INVERT != 0U);
    if(high) GPIO_SetBits(GPIOD, GPIO_Pin_2);
    else GPIO_ResetBits(GPIOD, GPIO_Pin_2);
}
static void Servo_Publish(void)
{
    uint8_t b, c;
    taskENTER_CRITICAL();
    s_frame_side = s_side;
    s_status.sweeping = 0;
    for(b = 0; b < 2; b++) {
        s_status.installed_mask[b] = s_installed[b];
        for(c = 0; c < 3; c++) {
            channel_t *ch = &s_bank[b][c];
            s_status.saved_us[b][c] = ch->pulse;
            s_status.target_us[b][c] = ch->target;
            s_status.motion[b][c] = ch->motion;
            s_status.boot_test_phase[b][c] = ch->test_phase;
            s_status.enabled[b][c] = ch->enabled;
            if(b == s_side) {
                s_status.pulse_us[c] = ch->pulse;
                s_frame[c] = (s_valid && ch->enabled) ? ch->pulse : 0;
                if(ch->motion == SERVO_SWEEPING) s_status.sweeping = 1;
            }
        }
    }
    s_status.side_right = (s_side == 0);
    s_status.config_valid = s_valid;
    taskEXIT_CRITICAL();
}
static servo_result_t Servo_Queue(command_t *cmd)
{
    uint8_t c, side, mask;
    servo_result_t result = SERVO_OK;
    taskENTER_CRITICAL();
    side = s_requested_side;
    mask = cmd->mask;
    if(!s_valid) result = SERVO_ERR_CONFIG;
    else if(s_count == QUEUE_SIZE) result = SERVO_ERR_QUEUE_FULL;
    else if(cmd->type != CMD_SIDE) {
        if(!mask || (mask & ~SERVO_MASK_ALL)) result = SERVO_ERR_RANGE;
        else if(mask != SERVO_MASK_ALL && (mask & ~s_installed[side])) result = SERVO_ERR_UNAVAILABLE;
        mask &= s_installed[side];
        if(!mask && result == SERVO_OK) result = SERVO_ERR_UNAVAILABLE;
        if(cmd->ms > ROBOT_SERVO_MAX_MOTION_MS) result = SERVO_ERR_RANGE;
        if(cmd->type == CMD_SWEEP && (cmd->ms < 20 || cmd->low >= cmd->high)) result = SERVO_ERR_RANGE;
        for(c = 0; c < 3 && result == SERVO_OK; c++) {
            if(!(mask & (1U << c))) continue;
            if(cmd->type == CMD_MOVE && (cmd->us[c] < s_min[side][c] || cmd->us[c] > s_max[side][c])) result = SERVO_ERR_RANGE;
            if(cmd->type == CMD_SWEEP && (cmd->low < s_min[side][c] || cmd->high > s_max[side][c])) result = SERVO_ERR_RANGE;
        }
    }
    if(result == SERVO_OK) {
        cmd->mask = mask;
        s_queue[(s_head + s_count) % QUEUE_SIZE] = *cmd;
        s_count++;
        if(cmd->type == CMD_SIDE) s_requested_side = cmd->side;
    }
    taskEXIT_CRITICAL();
    return result;
}
void SERVO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_TimeBaseInitTypeDef tb = {0};
    TIM_OCInitTypeDef oc = {0};
    RCC_ClocksTypeDef clocks = {0};
    NVIC_InitTypeDef nvic = {0};
    uint8_t b, c;
    uint32_t divider;
    s_head = s_count = 0;
    s_side = s_requested_side = ROBOT_SERVO_DEFAULT_SIDE_RIGHT ? 0 : 1;
    s_frame_side = s_hardware_side = s_side;
    s_valid = (FRAME_US > COMMIT_US + WRAP_GUARD_US && FRAME_US <= 65536U);
    memset(s_bank, 0, sizeof(s_bank));
    memset(&s_status, 0, sizeof(s_status));
    for(b = 0; b < 2; b++) {
        if(s_installed[b] & ~SERVO_MASK_ALL) s_valid = 0;
        for(c = 0; c < 3; c++) {
            if(s_min[b][c] < 500 || s_max[b][c] > 2500 ||
               s_min[b][c] > s_center[b][c] || s_center[b][c] > s_max[b][c]) s_valid = 0;
            s_bank[b][c].pulse = s_bank[b][c].target = s_center[b][c];
            s_bank[b][c].enabled = ((s_installed[b] >> c) & 1U);
        }
    }
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_Cmd(TIM3, DISABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC4, DISABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    Servo_WriteSide(s_side); /* Set latch before switching pin mode. */
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);
    RCC_GetClocksFreq(&clocks);
    s_timer_hz = clocks.PCLK1_Frequency;
    if(clocks.PCLK1_Frequency != clocks.HCLK_Frequency) s_timer_hz *= 2U;
    divider = s_timer_hz / 1000000U;
    if(!divider || divider > 65536U || s_timer_hz % 1000000U) { s_valid = 0; divider = 1; }
    tb.TIM_Prescaler = (uint16_t)(divider - 1);
    tb.TIM_Period = (uint16_t)(FRAME_US - 1);
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tb);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &oc); TIM_OC2Init(TIM3, &oc); TIM_OC3Init(TIM3, &oc);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    /* Internal CH4 deadline; PB1 is not driven. */
    oc.TIM_OCMode = TIM_OCMode_Timing;
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_Pulse = COMMIT_US;
    TIM_OC4Init(TIM3, &oc);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    Servo_Publish();
    /* Start with an empty frame so enabling the timer cannot stretch the first
     * high pulse. The first CC4 commit arms center for the next full frame. */
    TIM_SetCompare1(TIM3, 0); TIM_SetCompare2(TIM3, 0); TIM_SetCompare3(TIM3, 0);
    TIM_GenerateEvent(TIM3, TIM_EventSource_Update);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
    s_last_tick = xTaskGetTickCount();
    TIM_Cmd(TIM3, ENABLE);
    if(ROBOT_SERVO_BOOT_TEST && s_valid) {
        for(c = 0; c < 3; c++) {
            channel_t *ch = &s_bank[s_side][c];
            if(!ch->enabled) continue;
            ch->start = ch->pulse;
            ch->target = s_min[s_side][c];
            ch->duration = 1500U;
            ch->motion = SERVO_MOVING;
            ch->test_phase = 1U;
        }
        Servo_Publish();
    }
}
static void Servo_Apply(const command_t *cmd)
{
    uint8_t c;
    if(cmd->type == CMD_SIDE) {
        if(s_side != cmd->side) {
            for(c = 0; c < 3; c++) {
                s_bank[s_side][c].motion = SERVO_HOLD;
                s_bank[s_side][c].test_phase = 0;
                s_bank[s_side][c].target = s_bank[s_side][c].pulse;
            }
            s_side = cmd->side;
        }
        return;
    }
    for(c = 0; c < 3; c++) {
        channel_t *ch = &s_bank[s_side][c];
        if(!(cmd->mask & (1U << c))) continue;
        ch->test_phase = 0; /* Any explicit channel command cancels its boot test. */
        ch->motion = SERVO_HOLD; ch->target = ch->pulse; ch->elapsed = 0;
        if(cmd->type == CMD_OFF) ch->enabled = 0;
        else if(cmd->type == CMD_MOVE || cmd->type == CMD_CENTER) {
            ch->enabled = 1; ch->start = ch->pulse;
            ch->target = (cmd->type == CMD_CENTER) ? s_center[s_side][c] : cmd->us[c];
            ch->duration = cmd->ms;
            if(cmd->ms && ch->start != ch->target) ch->motion = SERVO_MOVING;
            else ch->pulse = ch->target;
        } else if(cmd->type == CMD_SWEEP) {
            ch->enabled = 1; ch->low = cmd->low; ch->high = cmd->high; ch->duration = cmd->ms;
            ch->pulse = ch->target = cmd->low; /* Explicit bench command starts at low. */
            ch->motion = SERVO_SWEEPING;
        }
    }
}
void SERVO_Update(void)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t dt = (uint32_t)(((uint64_t)(now - s_last_tick) * 1000U) / configTICK_RATE_HZ);
    uint8_t c, have_cmd = 0;
    command_t cmd;
    if(!dt) return;
    s_last_tick = now;
    /* Preserve FIFO ordering across the ISR bank switch. */
    if(s_hardware_side == s_side) {
        for(c = 0; c < 3; c++) {
            channel_t *ch = &s_bank[s_side][c];
            if(ch->motion == SERVO_MOVING) {
                uint32_t remaining = ch->duration - ch->elapsed;
                ch->elapsed += (dt < remaining) ? dt : remaining;
                ch->pulse = (uint16_t)((int32_t)ch->start +
                    ((int64_t)((int32_t)ch->target - ch->start) * ch->elapsed) / ch->duration);
                if(ch->elapsed == ch->duration) {
                    if(ch->test_phase == 1U || ch->test_phase == 2U) {
                        ch->start = ch->pulse;
                        ch->elapsed = 0;
                        ch->target = (ch->test_phase == 1U) ? s_max[s_side][c] : s_center[s_side][c];
                        ch->duration = (ch->test_phase == 1U) ? 3000U : 1500U;
                        ch->test_phase++;
                    } else {
                        ch->test_phase = 0;
                        ch->motion = SERVO_HOLD;
                    }
                }
            } else if(ch->motion == SERVO_SWEEPING) {
                uint32_t half = ch->duration / 2U, phase;
                ch->elapsed = (uint32_t)(((uint64_t)ch->elapsed + dt) % ch->duration);
                phase = ch->elapsed;
                if(phase < half) ch->pulse = ch->low + ((uint32_t)(ch->high-ch->low)*phase)/half;
                else ch->pulse = ch->high - ((uint32_t)(ch->high-ch->low)*(phase-half))/(ch->duration-half);
                ch->target = ch->pulse;
            }
        }
        taskENTER_CRITICAL();
        if(s_count) {
            cmd = s_queue[s_head]; s_head = (s_head + 1U) % QUEUE_SIZE; s_count--; have_cmd = 1;
        }
        taskEXIT_CRITICAL();
        if(have_cmd) Servo_Apply(&cmd);
    }
    Servo_Publish();
}
servo_result_t SERVO_Move(uint8_t mask, const uint16_t us[3], uint32_t ms)
{
    command_t cmd = {0};
    if(!us) return SERVO_ERR_RANGE;
    cmd.type = CMD_MOVE; cmd.mask = mask; cmd.ms = ms;
    memcpy(cmd.us, us, sizeof(cmd.us));
    return Servo_Queue(&cmd);
}
servo_result_t SERVO_SetChannelPulseUs(uint8_t c, uint16_t us)
{
    uint16_t pulses[3] = {us,us,us};
    if(c >= 3) return SERVO_ERR_RANGE;
    return SERVO_Move((uint8_t)(1U << c), pulses, 0);
}
servo_result_t SERVO_SetPulseUs(uint16_t us) { return SERVO_SetChannelPulseUs(1, us); }
servo_result_t SERVO_MoveAngle(uint8_t mask, int16_t degrees, uint32_t ms)
{
    uint16_t us[3];
    if(degrees < -135 || degrees > 135) return SERVO_ERR_RANGE;
    /* Same -135..135 degree / 500..2500 us mapping as code_2. */
    us[0] = us[1] = us[2] = (uint16_t)(500U +
        ((uint32_t)(degrees + 135) * 2000U + 135U) / 270U);
    return SERVO_Move(mask, us, ms);
}
servo_result_t SERVO_SetAllPulseUs(uint16_t us)
{
    uint16_t pulses[3] = {us,us,us}; return SERVO_Move(SERVO_MASK_ALL, pulses, 0);
}
static servo_result_t Servo_Simple(uint8_t type, uint8_t mask)
{
    command_t cmd = {0}; cmd.type = type; cmd.mask = mask; return Servo_Queue(&cmd);
}
servo_result_t SERVO_Center(uint8_t mask) { return Servo_Simple(CMD_CENTER, mask); }
servo_result_t SERVO_Off(uint8_t mask) { return Servo_Simple(CMD_OFF, mask); }
servo_result_t SERVO_Hold(uint8_t mask) { return Servo_Simple(CMD_HOLD, mask); }
servo_result_t SERVO_StopSweep(void) { return SERVO_Hold(SERVO_MASK_ALL); }
servo_result_t SERVO_Sweep(uint8_t mask, uint16_t low, uint16_t high, uint32_t ms)
{
    command_t cmd = {0}; cmd.type = CMD_SWEEP; cmd.mask = mask;
    cmd.low = low; cmd.high = high; cmd.ms = ms; return Servo_Queue(&cmd);
}
servo_result_t SERVO_StartSweep(uint16_t low, uint16_t high, uint32_t ms)
{ return SERVO_Sweep(SERVO_MASK_ALL, low, high, ms); }
servo_result_t SERVO_SetSideRight(uint8_t right)
{
    command_t cmd = {0}; cmd.type = CMD_SIDE; cmd.side = right ? 0 : 1; return Servo_Queue(&cmd);
}
const char *SERVO_ChannelName(uint8_t c)
{
    static const char *const names[3] = {"A","B","C"}; return c < 3 ? names[c] : "?";
}
const char *SERVO_HeaderName(uint8_t side, uint8_t c)
{ return side < 2 && c < 3 ? s_headers[side][c] : "?"; }
const char *SERVO_ResultName(servo_result_t r)
{
    static const char *const names[] = {"queued", "range/arguments", "interface unavailable", "queue full", "invalid config"};
    return (unsigned)r <= SERVO_ERR_CONFIG ? names[r] : "unknown";
}
void SERVO_GetLimits(uint8_t side, uint8_t c, uint16_t *low, uint16_t *center, uint16_t *high)
{
    if(side >= 2 || c >= 3 || !low || !center || !high) return;
    *low = s_min[side][c]; *center = s_center[side][c]; *high = s_max[side][c];
}
void SERVO_GetStatus(servo_status_t *out)
{
    if(!out) return;
    taskENTER_CRITICAL();
    *out = s_status;
    out->requested_side_right = (s_requested_side == 0);
    out->hardware_side_right = (s_hardware_side == 0);
    out->queued = s_count;
    out->compare[0] = TIM3->CH1CVR; out->compare[1] = TIM3->CH2CVR; out->compare[2] = TIM3->CH3CVR;
    out->prescaler = TIM3->PSC; out->period = TIM3->ATRLR;
    out->timer_control = TIM3->CTLR1; out->timer_hz = s_timer_hz;
    out->afio_pcfr1 = AFIO->PCFR1; out->gpio_cfglr = GPIOB->CFGLR; out->gpiod_odr = GPIOD->OUTDR;
    taskEXIT_CRITICAL();
}
uint8_t SERVO_GetSideRight(void) { return s_status.side_right; }
uint16_t SERVO_GetChannelPulseUs(uint8_t c) { return c < 3 ? s_status.pulse_us[c] : 0; }
uint16_t SERVO_GetPulseUs(void) { return SERVO_GetChannelPulseUs(1); }
uint8_t SERVO_IsSweeping(void) { return s_status.sweeping; }
#ifndef SERVO_HOST_TEST
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#endif
void TIM3_IRQHandler(void)
{
    uint16_t count;
    if(TIM_GetITStatus(TIM3, TIM_IT_CC4) == RESET) return;
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    count = TIM3->CNT;
    /* Late interrupts must not switch during pulses or close to counter wrap. */
    if(count < COMMIT_US || count > FRAME_US - WRAP_GUARD_US) return;
    TIM_SetCompare1(TIM3, s_frame[0]); TIM_SetCompare2(TIM3, s_frame[1]); TIM_SetCompare3(TIM3, s_frame[2]);
    if(s_hardware_side != s_frame_side) {
        Servo_WriteSide(s_frame_side); s_hardware_side = s_frame_side;
    }
}
