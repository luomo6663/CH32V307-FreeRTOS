/********************************** (C) COPYRIGHT *******************************
* File Name          : stepper.c
* Description        : STEP/DIR pulse generator for the turntable stepper.
*******************************************************************************/

#include "stepper.h"
#include "ch32v30x.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

#define STEPPER_TIMER                 TIM5
#define STEPPER_STEP_GPIO             GPIOA
#define STEPPER_STEP_PIN              GPIO_Pin_0
#define STEPPER_DIR_GPIO              GPIOA
#define STEPPER_DIR_PIN               GPIO_Pin_1
#define STEPPER_TIMER_BASE_TICK_HZ    1000000U
#define STEPPER_TIMER_MAX_PRESCALER   65536U

static volatile uint8_t  s_state = STEPPER_STATE_IDLE;
static volatile uint8_t  s_park_pending = 0U;
static volatile int8_t   s_direction = 0;
static volatile int32_t  s_position_steps = 0;
static volatile uint32_t s_pulses_remaining = 0U;

static int32_t  s_target_steps = 0;
static uint32_t s_steps_per_rev = ROBOT_STEPPER_STEPS_PER_REV;
static uint32_t s_timer_clock_hz = 0U;
static uint32_t s_prescaler_divider = 1U;
static uint32_t s_current_rate_hz = 0U;
static uint32_t s_target_rate_hz = 0U;
static volatile uint32_t s_rate_hz = 0U;
static uint16_t s_prescaler = 1U;
static uint16_t s_period = 0U;
static uint16_t s_compare = 0U;

static int32_t Stepper_ClampAdd(int32_t base, int32_t delta)
{
    int64_t value = (int64_t)base + (int64_t)delta;

    if(value > (int64_t)INT32_MAX)
    {
        return INT32_MAX;
    }

    if(value < (int64_t)INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t)value;
}

static void Stepper_SetPinMode(uint16_t pin, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.GPIO_Pin = pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = mode;
    GPIO_Init(STEPPER_STEP_GPIO, &gpio);
}

static void Stepper_WriteDirection(int8_t direction)
{
    uint8_t level = (direction > 0) ? 1U : 0U;

#if(ROBOT_STEPPER_DIR_INVERT)
    level ^= 1U;
#endif

    if(level != 0U)
    {
        GPIO_SetBits(STEPPER_DIR_GPIO, STEPPER_DIR_PIN);
    }
    else
    {
        GPIO_ResetBits(STEPPER_DIR_GPIO, STEPPER_DIR_PIN);
    }
}

/*
 * Do not use Delay_Us() here: it reprograms the same Cortex-style SysTick
 * that FreeRTOS uses for its tick. This short CPU loop is deliberately
 * conservative and only runs when a new move is requested.
 */
static void Stepper_DirectionSetupDelay(void)
{
    volatile uint32_t loops;

    loops = (SystemCoreClock / 1000000U) *
            ROBOT_STEPPER_DIR_SETUP_US * 2U;
    if(loops == 0U)
    {
        loops = 1U;
    }

    while(loops-- > 0U)
    {
        __asm volatile ("nop");
    }
}

static uint32_t Stepper_ApplyRate(uint32_t requested_hz)
{
    uint32_t period_ticks;
    uint32_t high_ticks;
    uint64_t divisor;

    if(requested_hz == 0U)
    {
        requested_hz = 1U;
    }
    if(requested_hz > ROBOT_STEPPER_MAX_RATE_HZ)
    {
        requested_hz = ROBOT_STEPPER_MAX_RATE_HZ;
    }

    divisor = (uint64_t)s_prescaler_divider * (uint64_t)requested_hz;
    period_ticks = (uint32_t)(((uint64_t)s_timer_clock_hz +
                               (divisor / 2ULL)) / divisor);
    if(period_ticks < 2U)
    {
        period_ticks = 2U;
    }
    else if(period_ticks > 65536U)
    {
        period_ticks = 65536U;
    }

    high_ticks =
        (uint32_t)(((uint64_t)ROBOT_STEPPER_PULSE_WIDTH_US *
                    (uint64_t)s_timer_clock_hz) /
                   ((uint64_t)s_prescaler_divider * 1000000ULL));
    if(high_ticks == 0U)
    {
        high_ticks = 1U;
    }
    if(high_ticks >= period_ticks)
    {
        high_ticks = period_ticks / 2U;
        if(high_ticks == 0U)
        {
            high_ticks = 1U;
        }
    }

    TIM_SetAutoreload(STEPPER_TIMER, (uint16_t)(period_ticks - 1U));
    TIM_SetCompare1(STEPPER_TIMER, (uint16_t)high_ticks);

    s_prescaler = (uint16_t)(s_prescaler_divider - 1U);
    s_period = (uint16_t)(period_ticks - 1U);
    s_compare = (uint16_t)high_ticks;
    s_rate_hz = (uint32_t)((uint64_t)s_timer_clock_hz /
                           ((uint64_t)s_prescaler_divider * period_ticks));

    return s_rate_hz;
}

static uint32_t Stepper_MinimumRateHz(void)
{
    uint64_t divisor = (uint64_t)s_prescaler_divider * 65536ULL;
    uint32_t minimum;

    minimum = (uint32_t)(((uint64_t)s_timer_clock_hz + divisor - 1ULL) /
                         divisor);
    return (minimum == 0U) ? 1U : minimum;
}

/*
 * Selects a timer prescaler that keeps the target rate inside the 16-bit TIM5
 * range, then starts the ramp at a lower rate. STEPPER_Update() moves the
 * active rate toward the target in small steps.
 */
static uint8_t Stepper_ConfigureRate(uint32_t requested_hz)
{
    uint32_t prescaler;
    uint32_t base_prescaler;
    uint32_t period_ticks;
    uint32_t start_rate;
    uint64_t divisor;
    uint64_t needed;

    if(requested_hz == 0U)
    {
        return 0U;
    }

    if(requested_hz > ROBOT_STEPPER_MAX_RATE_HZ)
    {
        requested_hz = ROBOT_STEPPER_MAX_RATE_HZ;
    }

    base_prescaler =
        (s_timer_clock_hz + (STEPPER_TIMER_BASE_TICK_HZ - 1U)) /
        STEPPER_TIMER_BASE_TICK_HZ;
    if(base_prescaler == 0U)
    {
        base_prescaler = 1U;
    }

    needed = ((uint64_t)s_timer_clock_hz +
              ((uint64_t)requested_hz * 65536ULL) - 1ULL) /
             ((uint64_t)requested_hz * 65536ULL);
    if(needed > (uint64_t)base_prescaler)
    {
        base_prescaler = (uint32_t)needed;
    }

    if(base_prescaler > STEPPER_TIMER_MAX_PRESCALER)
    {
        return 0U;
    }

    prescaler = base_prescaler;
    for(;;)
    {
        divisor = (uint64_t)prescaler * (uint64_t)requested_hz;
        period_ticks = (uint32_t)(((uint64_t)s_timer_clock_hz +
                                   (divisor / 2ULL)) / divisor);

        if(period_ticks <= 65536U)
        {
            break;
        }

        prescaler++;
        if(prescaler > STEPPER_TIMER_MAX_PRESCALER)
        {
            return 0U;
        }
    }

    if(period_ticks < 2U)
    {
        return 0U;
    }

    s_prescaler_divider = prescaler;

    TIM_Cmd(STEPPER_TIMER, DISABLE);
    TIM_ITConfig(STEPPER_TIMER, TIM_IT_Update, DISABLE);

    STEPPER_TIMER->PSC = (uint16_t)(prescaler - 1U);
    TIM_SetCounter(STEPPER_TIMER, 0U);

    start_rate = ROBOT_STEPPER_START_RATE_HZ;
    if((start_rate == 0U) || (start_rate > requested_hz))
    {
        start_rate = requested_hz;
    }
    if(start_rate < Stepper_MinimumRateHz())
    {
        start_rate = Stepper_MinimumRateHz();
    }

    s_target_rate_hz = requested_hz;
    s_current_rate_hz = Stepper_ApplyRate(start_rate);

    TIM_GenerateEvent(STEPPER_TIMER, TIM_EventSource_Update);
    TIM_ClearITPendingBit(STEPPER_TIMER, TIM_IT_Update);

    return (s_current_rate_hz != 0U) ? 1U : 0U;
}

static void Stepper_ParkStepOutput(void)
{
    TIM_CCxCmd(STEPPER_TIMER, TIM_Channel_1, TIM_CCx_Disable);
    Stepper_SetPinMode(STEPPER_STEP_PIN, GPIO_Mode_Out_PP);
    GPIO_ResetBits(STEPPER_STEP_GPIO, STEPPER_STEP_PIN);
}

static float Stepper_ClampRpm(float rpm)
{
    if(!(rpm > 0.0f))
    {
        rpm = ROBOT_STEPPER_DEFAULT_RPM;
    }
    if(rpm < ROBOT_STEPPER_MIN_RPM)
    {
        rpm = ROBOT_STEPPER_MIN_RPM;
    }
    else if(rpm > ROBOT_STEPPER_MAX_RPM)
    {
        rpm = ROBOT_STEPPER_MAX_RPM;
    }

    return rpm;
}

static void Stepper_StopOutput(void)
{
    TIM_ITConfig(STEPPER_TIMER, TIM_IT_Update, DISABLE);
    TIM_Cmd(STEPPER_TIMER, DISABLE);
    TIM_ClearITPendingBit(STEPPER_TIMER, TIM_IT_Update);
    TIM_SetCompare1(STEPPER_TIMER, 0U);
    Stepper_ParkStepOutput();

    s_rate_hz = 0U;
    s_current_rate_hz = 0U;
    s_target_rate_hz = 0U;
    s_park_pending = 0U;
}

static void Stepper_StartConfiguredTimer(void)
{
    TIM_SetCounter(STEPPER_TIMER, 0U);
    TIM_ClearITPendingBit(STEPPER_TIMER, TIM_IT_Update);

    Stepper_SetPinMode(STEPPER_STEP_PIN, GPIO_Mode_AF_PP);
    TIM_CCxCmd(STEPPER_TIMER, TIM_Channel_1, TIM_CCx_Enable);

    s_park_pending = 0U;
    TIM_ITConfig(STEPPER_TIMER, TIM_IT_Update, ENABLE);
    TIM_Cmd(STEPPER_TIMER, ENABLE);
}

static uint8_t Stepper_PrepareMove(int8_t direction, uint32_t rate_hz)
{
    Stepper_WriteDirection(direction);

    if(ROBOT_STEPPER_DIR_SETUP_US != 0U)
    {
        Stepper_DirectionSetupDelay();
    }

    if(Stepper_ConfigureRate(rate_hz) == 0U)
    {
        Stepper_StopOutput();
        return 0U;
    }

    return 1U;
}

void STEPPER_Init(void)
{
    GPIO_InitTypeDef     gpio = {0};
    TIM_TimeBaseInitTypeDef time_base = {0};
    TIM_OCInitTypeDef    oc = {0};
    NVIC_InitTypeDef     nvic = {0};
    RCC_ClocksTypeDef    clocks = {0};
    uint32_t             timer_prescaler;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    RCC_GetClocksFreq(&clocks);
    s_timer_clock_hz = clocks.PCLK1_Frequency;
    if(clocks.PCLK1_Frequency != clocks.HCLK_Frequency)
    {
        s_timer_clock_hz *= 2U;
    }

    gpio.GPIO_Pin = STEPPER_STEP_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(STEPPER_STEP_GPIO, &gpio);

    gpio.GPIO_Pin = STEPPER_DIR_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(STEPPER_DIR_GPIO, &gpio);

    Stepper_WriteDirection(1);

    timer_prescaler = s_timer_clock_hz / STEPPER_TIMER_BASE_TICK_HZ;
    if(timer_prescaler == 0U)
    {
        timer_prescaler = 1U;
    }
    else if(timer_prescaler > STEPPER_TIMER_MAX_PRESCALER)
    {
        timer_prescaler = STEPPER_TIMER_MAX_PRESCALER;
    }

    time_base.TIM_Prescaler = (uint16_t)(timer_prescaler - 1U);
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Period = 999U;
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(STEPPER_TIMER, &time_base);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = 0U;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(STEPPER_TIMER, &oc);
    TIM_OC1PreloadConfig(STEPPER_TIMER, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(STEPPER_TIMER, ENABLE);

    TIM_ITConfig(STEPPER_TIMER, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(STEPPER_TIMER, TIM_IT_Update);

    nvic.NVIC_IRQChannel = TIM5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    Stepper_ParkStepOutput();
    TIM_Cmd(STEPPER_TIMER, DISABLE);

    s_state = STEPPER_STATE_IDLE;
    s_direction = 0;
    s_position_steps = 0;
    s_target_steps = 0;
    s_pulses_remaining = 0U;
    s_park_pending = 0U;
    s_rate_hz = 0U;
    s_current_rate_hz = 0U;
    s_target_rate_hz = 0U;
    s_steps_per_rev = ROBOT_STEPPER_STEPS_PER_REV;
}

uint8_t STEPPER_MovePulses(int32_t pulses, uint32_t rate_hz)
{
    int8_t   direction;
    uint32_t magnitude;
    int32_t  target;

    if(pulses == 0)
    {
        return 1U;
    }

    direction = (pulses > 0) ? 1 : -1;
    magnitude = (pulses > 0)
                    ? (uint32_t)pulses
                    : (uint32_t)(-(int64_t)pulses);

    STEPPER_Stop();

    if(Stepper_PrepareMove(direction, rate_hz) == 0U)
    {
        return 0U;
    }

    taskENTER_CRITICAL();

    target = Stepper_ClampAdd(s_position_steps, pulses);

    s_direction = direction;
    s_pulses_remaining = magnitude;
    s_target_steps = target;
    s_state = STEPPER_STATE_MOVING;

    Stepper_StartConfiguredTimer();

    taskEXIT_CRITICAL();

    return 1U;
}

uint8_t STEPPER_MoveDegrees(float degrees, float rpm)
{
    float    pulse_count;
    int32_t  pulses;
    uint32_t rate_hz;
    uint32_t steps_per_rev;

    if(degrees == 0.0f)
    {
        return 1U;
    }

    steps_per_rev = STEPPER_GetStepsPerRev();
    if(steps_per_rev == 0U)
    {
        return 0U;
    }

    pulse_count = (degrees * (float)steps_per_rev) / 360.0f;
    if(pulse_count >= 2147483647.0f)
    {
        pulses = INT32_MAX;
    }
    else if(pulse_count <= -2147483648.0f)
    {
        pulses = INT32_MIN;
    }
    else
    {
        pulses = (pulse_count >= 0.0f)
                     ? (int32_t)(pulse_count + 0.5f)
                     : (int32_t)(pulse_count - 0.5f);
    }
    if(pulses == 0)
    {
        pulses = (degrees > 0.0f) ? 1 : -1;
    }

    rpm = Stepper_ClampRpm(rpm);
    rate_hz = STEPPER_RpmToRate(rpm);

    return STEPPER_MovePulses(pulses, rate_hz);
}

uint8_t STEPPER_Run(int8_t direction, float rpm)
{
    uint32_t rate_hz;
    uint32_t steps_per_rev;

    if(direction == 0)
    {
        STEPPER_Stop();
        return 1U;
    }

    steps_per_rev = STEPPER_GetStepsPerRev();
    if(steps_per_rev == 0U)
    {
        return 0U;
    }

    rpm = Stepper_ClampRpm(rpm);
    rate_hz = STEPPER_RpmToRate(rpm);

    STEPPER_Stop();

    direction = (direction > 0) ? 1 : -1;
    if(Stepper_PrepareMove(direction, rate_hz) == 0U)
    {
        return 0U;
    }

    taskENTER_CRITICAL();

    s_direction = direction;
    s_pulses_remaining = 0U;
    s_target_steps = s_position_steps;
    s_state = STEPPER_STATE_RUNNING;

    Stepper_StartConfiguredTimer();

    taskEXIT_CRITICAL();

    return 1U;
}

void STEPPER_Stop(void)
{
    taskENTER_CRITICAL();

    Stepper_StopOutput();
    s_state = STEPPER_STATE_IDLE;
    s_direction = 0;
    s_pulses_remaining = 0U;
    s_target_steps = s_position_steps;

    taskEXIT_CRITICAL();
}

void STEPPER_SetStepsPerRev(uint32_t steps_per_rev)
{
    if(steps_per_rev == 0U)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_steps_per_rev = steps_per_rev;
    taskEXIT_CRITICAL();
}

uint32_t STEPPER_GetStepsPerRev(void)
{
    uint32_t steps_per_rev;

    taskENTER_CRITICAL();
    steps_per_rev = s_steps_per_rev;
    taskEXIT_CRITICAL();

    return steps_per_rev;
}

uint32_t STEPPER_RpmToRate(float rpm)
{
    float    rate;
    uint32_t steps_per_rev;

    steps_per_rev = STEPPER_GetStepsPerRev();
    rpm = Stepper_ClampRpm(rpm);

    rate = (float)steps_per_rev * rpm / 60.0f;
    if(!(rate >= 1.0f))
    {
        return 1U;
    }
    if(rate > (float)ROBOT_STEPPER_MAX_RATE_HZ)
    {
        return ROBOT_STEPPER_MAX_RATE_HZ;
    }

    return (uint32_t)(rate + 0.5f);
}

void STEPPER_SetPositionPulses(int32_t position_steps)
{
    STEPPER_Stop();

    taskENTER_CRITICAL();
    s_position_steps = position_steps;
    s_target_steps = position_steps;
    taskEXIT_CRITICAL();
}

void STEPPER_Zero(void)
{
    STEPPER_SetPositionPulses(0);
}

static uint32_t Stepper_StepRate(uint32_t current, uint32_t target,
                                 uint32_t max_delta)
{
    if(current < target)
    {
        uint32_t next = current + max_delta;

        return (next > target) ? target : next;
    }

    if(current > target)
    {
        return ((current - target) > max_delta) ? (current - max_delta)
                                                : target;
    }

    return current;
}

void STEPPER_Update(void)
{
    uint32_t accel_step;
    uint32_t decel_step;
    uint32_t target_rate;
    uint32_t current_rate;
    uint32_t remaining;
    float    stop_rate;

    if(s_park_pending != 0U)
    {
        taskENTER_CRITICAL();
        if((s_park_pending != 0U) && (s_state == STEPPER_STATE_IDLE))
        {
            Stepper_ParkStepOutput();
            s_park_pending = 0U;
        }
        taskEXIT_CRITICAL();
    }

    if(s_state == STEPPER_STATE_IDLE)
    {
        return;
    }

    accel_step = (ROBOT_STEPPER_ACCEL_HZ_PER_S *
                  ROBOT_CONTROL_PERIOD_MS) / 1000U;
    decel_step = (ROBOT_STEPPER_DECEL_HZ_PER_S *
                  ROBOT_CONTROL_PERIOD_MS) / 1000U;
    if(accel_step == 0U)
    {
        accel_step = 1U;
    }
    if(decel_step == 0U)
    {
        decel_step = 1U;
    }

    target_rate = s_target_rate_hz;
    current_rate = s_current_rate_hz;
    remaining = s_pulses_remaining;

    if((s_state == STEPPER_STATE_MOVING) && (remaining > 0U))
    {
        /*
         * v^2 = 2 * a * s. Limit the target rate near the end of a move so
         * the pulse train can decelerate instead of stopping at full speed.
         */
        stop_rate = sqrtf(2.0f * (float)ROBOT_STEPPER_DECEL_HZ_PER_S *
                          (float)remaining);
        if(!(stop_rate >= 1.0f))
        {
            stop_rate = 1.0f;
        }
        if(stop_rate < (float)Stepper_MinimumRateHz())
        {
            stop_rate = (float)Stepper_MinimumRateHz();
        }
        if(stop_rate < (float)target_rate)
        {
            target_rate = (uint32_t)stop_rate;
        }
    }

    if(current_rate < target_rate)
    {
        current_rate = Stepper_StepRate(current_rate, target_rate, accel_step);
    }
    else if(current_rate > target_rate)
    {
        current_rate = Stepper_StepRate(current_rate, target_rate, decel_step);
    }

    if(current_rate != s_current_rate_hz)
    {
        taskENTER_CRITICAL();
        if(s_state != STEPPER_STATE_IDLE)
        {
            s_current_rate_hz = current_rate;
            (void)Stepper_ApplyRate(current_rate);
        }
        taskEXIT_CRITICAL();
    }
}

void STEPPER_GetStatus(stepper_status_t *out)
{
    if(out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();

    out->position_steps = s_position_steps;
    out->target_steps = s_target_steps;
    out->steps_per_rev = s_steps_per_rev;
    out->rate_hz = s_rate_hz;
    out->target_rate_hz = s_target_rate_hz;
    out->prescaler = s_prescaler;
    out->period = s_period;
    out->compare = s_compare;
    out->direction = s_direction;
    out->state = s_state;

    taskEXIT_CRITICAL();
}

const char *STEPPER_StateName(stepper_state_t state)
{
    switch(state)
    {
        case STEPPER_STATE_IDLE:
            return "idle";
        case STEPPER_STATE_MOVING:
            return "move";
        case STEPPER_STATE_RUNNING:
            return "run";
        default:
            return "?";
    }
}

void TIM5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM5_IRQHandler(void)
{
    if(TIM_GetITStatus(STEPPER_TIMER, TIM_IT_Update) == RESET)
    {
        return;
    }

    TIM_ClearITPendingBit(STEPPER_TIMER, TIM_IT_Update);

    if((s_state == STEPPER_STATE_IDLE) || (s_direction == 0))
    {
        return;
    }

    s_position_steps += s_direction;

    if(s_state == STEPPER_STATE_RUNNING)
    {
        return;
    }

    if(s_pulses_remaining > 0U)
    {
        s_pulses_remaining--;
    }

    if(s_pulses_remaining == 0U)
    {
        TIM_ITConfig(STEPPER_TIMER, TIM_IT_Update, DISABLE);
        TIM_Cmd(STEPPER_TIMER, DISABLE);
        TIM_SetCompare1(STEPPER_TIMER, 0U);

        s_state = STEPPER_STATE_IDLE;
        s_direction = 0;
        s_rate_hz = 0U;
        s_current_rate_hz = 0U;
        s_target_rate_hz = 0U;
        s_park_pending = 1U;
    }
}
