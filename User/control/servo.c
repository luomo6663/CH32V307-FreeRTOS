/********************************** (C) COPYRIGHT *******************************
* File Name          : servo.c
* Description        : PB5 / TIM3_CH2 50 Hz hobby servo output.
*******************************************************************************/

#include "servo.h"
#include "ch32v30x.h"
#include "FreeRTOS.h"
#include "task.h"

#define SERVO_TIMER                 TIM3
#define SERVO_TIMER_TICK_HZ         1000000U
#define SERVO_PULSE_COMMAND_MIN_US  500U
#define SERVO_PULSE_COMMAND_MAX_US  2500U

static uint16_t   s_pulse_us = 1500U;
static uint16_t   s_sweep_min_us = SERVO_SWEEP_MIN_US;
static uint16_t   s_sweep_max_us = SERVO_SWEEP_MAX_US;
static uint32_t   s_sweep_period_ms = SERVO_SWEEP_PERIOD_MS;
static uint32_t   s_sweep_elapsed_ms = 0U;
static TickType_t s_last_update_tick = 0U;
static uint8_t    s_sweep_active = 0U;

static uint16_t Servo_ClampPulse(uint16_t pulse_us)
{
    if(pulse_us < SERVO_PULSE_COMMAND_MIN_US)
    {
        return SERVO_PULSE_COMMAND_MIN_US;
    }

    if(pulse_us > SERVO_PULSE_COMMAND_MAX_US)
    {
        return SERVO_PULSE_COMMAND_MAX_US;
    }

    return pulse_us;
}

static void Servo_WritePulse(uint16_t pulse_us)
{
    s_pulse_us = pulse_us;
    TIM_SetCompare2(SERVO_TIMER, pulse_us);
}

void SERVO_Init(void)
{
    GPIO_InitTypeDef     gpio = {0};
    TIM_TimeBaseInitTypeDef time_base = {0};
    TIM_OCInitTypeDef    oc = {0};
    RCC_ClocksTypeDef    clocks = {0};
    uint32_t             timer_clock;
    uint32_t             prescaler;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB,
                           ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* TIM3 partial remap: CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1. */
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if(clocks.PCLK1_Frequency != clocks.HCLK_Frequency)
    {
        timer_clock *= 2U;
    }

    prescaler = timer_clock / SERVO_TIMER_TICK_HZ;
    if(prescaler == 0U)
    {
        prescaler = 1U;
    }

    time_base.TIM_Prescaler = (uint16_t)(prescaler - 1U);
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Period =
        (uint16_t)((SERVO_TIMER_TICK_HZ / SERVO_PWM_FREQUENCY_HZ) - 1U);
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(SERVO_TIMER, &time_base);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = s_pulse_us;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC2Init(SERVO_TIMER, &oc);
    TIM_OC2PreloadConfig(SERVO_TIMER, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(SERVO_TIMER, ENABLE);
    TIM_SetCompare2(SERVO_TIMER, s_pulse_us);
    TIM_Cmd(SERVO_TIMER, ENABLE);

    s_last_update_tick = xTaskGetTickCount();
}

void SERVO_SetPulseUs(uint16_t pulse_us)
{
    s_sweep_active = 0U;
    Servo_WritePulse(Servo_ClampPulse(pulse_us));
}

void SERVO_StartSweep(uint16_t min_pulse_us, uint16_t max_pulse_us,
                      uint32_t period_ms)
{
    if(min_pulse_us > max_pulse_us)
    {
        uint16_t swap = min_pulse_us;
        min_pulse_us = max_pulse_us;
        max_pulse_us = swap;
    }

    s_sweep_min_us = Servo_ClampPulse(min_pulse_us);
    s_sweep_max_us = Servo_ClampPulse(max_pulse_us);

    if(s_sweep_min_us == s_sweep_max_us)
    {
        s_sweep_active = 0U;
        Servo_WritePulse(s_sweep_min_us);
        return;
    }

    if(period_ms < 20U)
    {
        period_ms = 20U;
    }

    s_sweep_period_ms = period_ms;
    s_sweep_elapsed_ms = 0U;
    s_last_update_tick = xTaskGetTickCount();
    s_sweep_active = 1U;
}

void SERVO_StopSweep(void)
{
    s_sweep_active = 0U;
}

void SERVO_Update(void)
{
    TickType_t now;
    uint32_t   elapsed_ms;
    uint32_t   half_period_ms;
    uint32_t   phase_ms;
    uint32_t   range_us;
    uint16_t   pulse_us;

    if(s_sweep_active == 0U)
    {
        return;
    }

    now = xTaskGetTickCount();
    elapsed_ms = (uint32_t)(now - s_last_update_tick) * 1000U;
    elapsed_ms /= configTICK_RATE_HZ;
    if(elapsed_ms == 0U)
    {
        return;
    }

    s_last_update_tick = now;
    s_sweep_elapsed_ms = (s_sweep_elapsed_ms + elapsed_ms) %
                         s_sweep_period_ms;

    half_period_ms = s_sweep_period_ms / 2U;
    phase_ms = s_sweep_elapsed_ms;
    range_us = (uint32_t)(s_sweep_max_us - s_sweep_min_us);

    if(phase_ms < half_period_ms)
    {
        pulse_us = (uint16_t)(s_sweep_min_us +
                   ((range_us * phase_ms) / half_period_ms));
    }
    else
    {
        pulse_us = (uint16_t)(s_sweep_max_us -
                   ((range_us * (phase_ms - half_period_ms)) /
                    (s_sweep_period_ms - half_period_ms)));
    }

    Servo_WritePulse(pulse_us);
}

uint16_t SERVO_GetPulseUs(void)
{
    return s_pulse_us;
}

uint8_t SERVO_IsSweeping(void)
{
    return s_sweep_active;
}

void SERVO_GetStatus(servo_status_t *out)
{
    if(out == NULL)
    {
        return;
    }

    out->pulse_us = s_pulse_us;
    out->prescaler = SERVO_TIMER->PSC;
    out->period = SERVO_TIMER->ATRLR;
    out->compare = SERVO_TIMER->CH2CVR;
    out->timer_control = SERVO_TIMER->CTLR1;
    out->sweeping = s_sweep_active;
    out->afio_pcfr1 = AFIO->PCFR1;
    out->gpio_cfglr = GPIOB->CFGLR;
}
