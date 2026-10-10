/********************************** (C) COPYRIGHT *******************************
* File Name          : encoder.c
* Description        : Quadrature encoder input, speed conversion and bring-up
*                      diagnostics.
*******************************************************************************/

#include "encoder.h"
#include "ch32v30x.h"
#include <stddef.h>

typedef struct
{
    TIM_TypeDef  *timer;        /* quadrature timer that counts this wheel     */
    GPIO_TypeDef *port_a;
    uint16_t      pin_a;
    GPIO_TypeDef *port_b;
    uint16_t      pin_b;
    uint8_t       quadrature;
    uint8_t       edge_multiplier;
} encoder_hw_t;

static const encoder_hw_t s_hw[ENCODER_COUNT] =
{
    {TIM8,  GPIOC, GPIO_Pin_6,  GPIOC, GPIO_Pin_7,  1U, 4U},  /* W1 PC6/PC7   */
    {TIM10, GPIOD, GPIO_Pin_1,  GPIOD, GPIO_Pin_3,  1U, 4U},  /* W2 PD1/PD3   */
    {TIM1,  GPIOA, GPIO_Pin_8,  GPIOA, GPIO_Pin_9,  1U, 4U},  /* W3 PA8/PA9   */
    {TIM9,  GPIOD, GPIO_Pin_9,  GPIOD, GPIO_Pin_11, 1U, 4U}   /* W4 PD9/PD11  */
};

static int32_t  s_total[ENCODER_COUNT] = {0, 0, 0, 0};
static int32_t  s_delta[ENCODER_COUNT] = {0, 0, 0, 0};
static int16_t  s_rpm[ENCODER_COUNT] = {0, 0, 0, 0};
static uint16_t s_last_timer[ENCODER_COUNT] = {0, 0, 0, 0};
static int8_t   s_sign[ENCODER_COUNT] = {1, 1, 1, 1};

static uint8_t  s_last_level[ENCODER_COUNT][2];
static uint8_t  s_toggled[ENCODER_COUNT][2];
static uint32_t s_transitions[ENCODER_COUNT][2];

static const int8_t s_sign_default[ENCODER_COUNT] =
{
    (int8_t)ROBOT_ENCODER0_SIGN,
    (int8_t)ROBOT_ENCODER1_SIGN,
    (int8_t)ROBOT_ENCODER2_SIGN,
    (int8_t)ROBOT_ENCODER3_SIGN
};

static uint8_t Encoder_IsQuadrature(uint8_t wheel)
{
    return ((s_hw[wheel].quadrature != 0U) && (s_hw[wheel].timer != NULL)) ? 1U : 0U;
}

static void Encoder_InitTimer(TIM_TypeDef *timer)
{
    TIM_TimeBaseInitTypeDef time_base = {0};

    time_base.TIM_Prescaler = 0U;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Period = 0xFFFFU;
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(timer, &time_base);

    TIM_EncoderInterfaceConfig(timer,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    timer->CHCTLR1 &= (uint16_t)~(TIM_IC1F | TIM_IC2F);
    timer->CHCTLR1 |= (uint16_t)(((uint16_t)ROBOT_ENCODER_INPUT_FILTER << 4) |
                                 ((uint16_t)ROBOT_ENCODER_INPUT_FILTER << 12));

    TIM_SetCounter(timer, 0U);
    TIM_Cmd(timer, ENABLE);
}

static void Encoder_SampleLevels(uint8_t wheel)
{
#if(ROBOT_ENCODER_LEVEL_WATCH)
    const encoder_hw_t *hw = &s_hw[wheel];
    uint8_t level;

    if(hw->pin_a != 0U)
    {
        level = (uint8_t)GPIO_ReadInputDataBit(hw->port_a, hw->pin_a);
        if(level != s_last_level[wheel][0])
        {
            s_last_level[wheel][0] = level;
            s_toggled[wheel][0] = 1U;
            s_transitions[wheel][0]++;
        }
    }

    if(hw->pin_b != 0U)
    {
        level = (uint8_t)GPIO_ReadInputDataBit(hw->port_b, hw->pin_b);
        if(level != s_last_level[wheel][1])
        {
            s_last_level[wheel][1] = level;
            s_toggled[wheel][1] = 1U;
            s_transitions[wheel][1]++;
        }
    }
#else
    (void)wheel;
#endif
}

void ENCODER_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t wheel;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_TIM8 |
                           RCC_APB2Periph_TIM9 |
                           RCC_APB2Periph_TIM10, ENABLE);

    /* W2 CH1/CH2 and W4 CH1/CH2 only appear on PD1/PD3 and PD9/PD11 remapped. */
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM9, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM10, ENABLE);

    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_11;
    GPIO_Init(GPIOD, &gpio);

    for(wheel = 0U; wheel < ENCODER_COUNT; wheel++)
    {
        s_sign[wheel] = s_sign_default[wheel];

        if(Encoder_IsQuadrature(wheel) != 0U)
        {
            Encoder_InitTimer(s_hw[wheel].timer);
        }
    }

    ENCODER_ResetCounters();
    ENCODER_ResetDiagnostics();
}

void ENCODER_SetSign(uint8_t wheel, int8_t sign)
{
    if(wheel >= ENCODER_COUNT)
    {
        return;
    }

    s_sign[wheel] = (sign < 0) ? -1 : 1;
}

int8_t ENCODER_GetSign(uint8_t wheel)
{
    return (wheel < ENCODER_COUNT) ? s_sign[wheel] : 1;
}

uint8_t ENCODER_HasQuadrature(uint8_t wheel)
{
    if(wheel >= ENCODER_COUNT)
    {
        return 0U;
    }

    return Encoder_IsQuadrature(wheel);
}

uint32_t ENCODER_GetCountsPerRev(uint8_t wheel)
{
    uint32_t multiplier;

    if(wheel >= ENCODER_COUNT)
    {
        return 0U;
    }

    multiplier = s_hw[wheel].edge_multiplier;
    return (uint32_t)ROBOT_ENCODER_PPR * (uint32_t)ROBOT_ENCODER_GEAR_RATIO * multiplier;
}

void ENCODER_Update(void)
{
    uint8_t wheel;

    for(wheel = 0U; wheel < ENCODER_COUNT; wheel++)
    {
        uint16_t now = TIM_GetCounter(s_hw[wheel].timer);
        int32_t  delta = (int32_t)(int16_t)(now - s_last_timer[wheel]);

        s_last_timer[wheel] = now;

        delta *= (int32_t)s_sign[wheel];

        s_delta[wheel] = delta;
        s_total[wheel] += delta;
        s_rpm[wheel] = ENCODER_CountsToRpm(wheel, delta);

        Encoder_SampleLevels(wheel);
    }
}

int16_t ENCODER_CountsToRpm(uint8_t wheel, int32_t delta_count)
{
    float counts_per_rev;
    float rpm;

    if(wheel >= ENCODER_COUNT)
    {
        return 0;
    }

    counts_per_rev = (float)ENCODER_GetCountsPerRev(wheel);
    if(counts_per_rev <= 0.0f)
    {
        return 0;
    }

    rpm = ((float)delta_count * 60000.0f) /
          (counts_per_rev * (float)ENCODER_SAMPLE_PERIOD_MS);

    if(rpm > 32767.0f)
    {
        rpm = 32767.0f;
    }
    else if(rpm < -32768.0f)
    {
        rpm = -32768.0f;
    }

    return (int16_t)rpm;
}

int32_t ENCODER_GetCount(uint8_t wheel)
{
    return (wheel < ENCODER_COUNT) ? s_total[wheel] : 0;
}

int32_t ENCODER_GetDelta(uint8_t wheel)
{
    return (wheel < ENCODER_COUNT) ? s_delta[wheel] : 0;
}

int16_t ENCODER_GetRpm(uint8_t wheel)
{
    return (wheel < ENCODER_COUNT) ? s_rpm[wheel] : 0;
}

uint16_t ENCODER_GetRawCount(uint8_t wheel)
{
    if((wheel >= ENCODER_COUNT) || (s_hw[wheel].timer == NULL))
    {
        return 0U;
    }

    return TIM_GetCounter(s_hw[wheel].timer);
}

uint8_t ENCODER_GetInputLevel(uint8_t wheel, uint8_t channel)
{
    if(wheel >= ENCODER_COUNT)
    {
        return 0U;
    }

    if(channel == 0U)
    {
        return (s_hw[wheel].pin_a == 0U)
                   ? 0U
                   : (uint8_t)GPIO_ReadInputDataBit(s_hw[wheel].port_a, s_hw[wheel].pin_a);
    }

    return (s_hw[wheel].pin_b == 0U)
               ? 0U
               : (uint8_t)GPIO_ReadInputDataBit(s_hw[wheel].port_b, s_hw[wheel].pin_b);
}

uint8_t ENCODER_GetToggleFlag(uint8_t wheel, uint8_t channel)
{
    if((wheel >= ENCODER_COUNT) || (channel > 1U))
    {
        return 0U;
    }

    return s_toggled[wheel][channel];
}

uint32_t ENCODER_GetTransitionCount(uint8_t wheel, uint8_t channel)
{
    if((wheel >= ENCODER_COUNT) || (channel > 1U))
    {
        return 0U;
    }

    return s_transitions[wheel][channel];
}

void ENCODER_GetStatus(uint8_t wheel, encoder_status_t *out)
{
    if((wheel >= ENCODER_COUNT) || (out == NULL))
    {
        return;
    }

    out->count = s_total[wheel];
    out->delta = s_delta[wheel];
    out->rpm = s_rpm[wheel];
    out->raw = ENCODER_GetRawCount(wheel);
    out->level_a = ENCODER_GetInputLevel(wheel, 0U);
    out->level_b = ENCODER_GetInputLevel(wheel, 1U);
    out->toggled_a = s_toggled[wheel][0];
    out->toggled_b = s_toggled[wheel][1];
    out->transitions_a = s_transitions[wheel][0];
    out->transitions_b = s_transitions[wheel][1];
}

void ENCODER_Reset(uint8_t wheel)
{
    if(wheel >= ENCODER_COUNT)
    {
        return;
    }

    s_total[wheel] = 0;
    s_delta[wheel] = 0;
    s_rpm[wheel] = 0;

    /*
     * Re-seed the delta reference from the live counter instead of clearing
     * the timer. The 16 bit counter keeps running, so a reset issued from the
     * console cannot create a false speed spike in the control task.
     */
    if(s_hw[wheel].timer != NULL)
    {
        s_last_timer[wheel] = TIM_GetCounter(s_hw[wheel].timer);
    }
}

void ENCODER_ResetCounters(void)
{
    uint8_t wheel;

    for(wheel = 0U; wheel < ENCODER_COUNT; wheel++)
    {
        ENCODER_Reset(wheel);
    }
}

void ENCODER_ResetDiagnostics(void)
{
    uint8_t wheel;
    uint8_t channel;

    for(wheel = 0U; wheel < ENCODER_COUNT; wheel++)
    {
        for(channel = 0U; channel < 2U; channel++)
        {
            s_toggled[wheel][channel] = 0U;
            s_transitions[wheel][channel] = 0U;
            s_last_level[wheel][channel] = ENCODER_GetInputLevel(wheel, channel);
        }
    }
}
