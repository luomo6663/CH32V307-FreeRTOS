/********************************** (C) COPYRIGHT *******************************
* File Name          : motor.c
* Description        : Four channel H-bridge PWM layer with runtime calibration.
*******************************************************************************/

#include "motor.h"

typedef struct
{
    GPIO_TypeDef *port_in1;
    uint16_t      pin_in1;
    GPIO_TypeDef *port_in2;
    uint16_t      pin_in2;
    TIM_TypeDef  *tim;
    uint8_t       ch_in1;
    uint8_t       ch_in2;
} motor_hw_t;

/*
 * TIM2 partial remap 1 moves CH1/CH2 to PA15/PB3 while CH3/CH4 stay on PA2/PA3.
 * TIM4 is fully remapped to PD12-PD15.
 */
static const motor_hw_t s_hw[MOTOR_COUNT] =
{
    {GPIOA, GPIO_Pin_2,  GPIOA, GPIO_Pin_3,  TIM2, 3U, 4U}, /* W1 right rear  */
    {GPIOA, GPIO_Pin_15, GPIOB, GPIO_Pin_3,  TIM2, 1U, 2U}, /* W2 right front */
    {GPIOD, GPIO_Pin_12, GPIOD, GPIO_Pin_13, TIM4, 1U, 2U}, /* W3 left front  */
    {GPIOD, GPIO_Pin_14, GPIOD, GPIO_Pin_15, TIM4, 3U, 4U}  /* W4 left rear   */
};

static const uint8_t s_invert_default[MOTOR_COUNT] =
{
    (uint8_t)ROBOT_MOTOR0_INVERT,
    (uint8_t)ROBOT_MOTOR1_INVERT,
    (uint8_t)ROBOT_MOTOR2_INVERT,
    (uint8_t)ROBOT_MOTOR3_INVERT
};

static int16_t      s_target[MOTOR_COUNT] = {0, 0, 0, 0};
static int16_t      s_applied[MOTOR_COUNT] = {0, 0, 0, 0};
static uint8_t      s_invert[MOTOR_COUNT] = {0, 0, 0, 0};
static int16_t      s_slew = (int16_t)MOTOR_DEFAULT_SLEW;
static uint16_t     s_pwm_period = 1U;
static uint32_t     s_timer_clock = 0U;
static motor_mode_t s_mode = MOTOR_MODE_DUAL_PWM;

static void Motor_SetCompare(TIM_TypeDef *tim, uint8_t channel, uint16_t compare)
{
    switch(channel)
    {
        case 1U:
            TIM_SetCompare1(tim, compare);
            break;
        case 2U:
            TIM_SetCompare2(tim, compare);
            break;
        case 3U:
            TIM_SetCompare3(tim, compare);
            break;
        case 4U:
            TIM_SetCompare4(tim, compare);
            break;
        default:
            break;
    }
}

static void Motor_SetPinMode(GPIO_TypeDef *port, uint16_t pin, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.GPIO_Pin = pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = mode;
    GPIO_Init(port, &gpio);
}

/*
 * Pushes the applied logical duty of one wheel to its two bridge inputs.
 * A compare value above ARR keeps a channel steadily high.
 */
static void Motor_WriteBridge(uint8_t motor, int16_t duty)
{
    const motor_hw_t *hw = &s_hw[motor];
    int32_t magnitude;
    uint16_t pulse;
    int16_t d = duty;

    if(s_invert[motor] != 0U)
    {
        d = (int16_t)(-d);
    }

    magnitude = (d < 0) ? -(int32_t)d : (int32_t)d;
    pulse = (uint16_t)((magnitude * (int32_t)s_pwm_period) / MOTOR_DUTY_MAX);

    if(s_mode == MOTOR_MODE_PWM_DIR)
    {
        /* IN1 carries the PWM, IN2 is the static direction level. */
        Motor_SetCompare(hw->tim, hw->ch_in1, (d > 0) ? pulse : 0U);

        if(d < 0)
        {
            GPIO_SetBits(hw->port_in2, hw->pin_in2);
        }
        else
        {
            GPIO_ResetBits(hw->port_in2, hw->pin_in2);
        }
    }
    else
    {
        Motor_SetCompare(hw->tim, hw->ch_in1, (d > 0) ? pulse : 0U);
        Motor_SetCompare(hw->tim, hw->ch_in2, (d < 0) ? pulse : 0U);
    }

    s_applied[motor] = duty;
}

static void Motor_UpdateClock(void)
{
    RCC_ClocksTypeDef clocks = {0};

    /*
     * APB1 timers are clocked at PCLK1, doubled whenever the APB1 prescaler
     * divides the AHB clock. At 96 MHz core that is 96 MHz for TIM2/TIM4.
     */
    RCC_GetClocksFreq(&clocks);
    if(clocks.PCLK1_Frequency == clocks.HCLK_Frequency)
    {
        s_timer_clock = clocks.PCLK1_Frequency;
    }
    else
    {
        s_timer_clock = clocks.PCLK1_Frequency * 2U;
    }
}

static void Motor_InitTimer(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef time_base = {0};
    TIM_OCInitTypeDef oc = {0};
    uint32_t ticks;
    uint32_t prescaler;

    /* Pick the smallest prescaler that fits the period into 16 bits. */
    ticks = s_timer_clock / MOTOR_PWM_FREQUENCY_HZ;
    if(ticks == 0U)
    {
        ticks = 1U;
    }

    prescaler = (ticks + 65535U) / 65536U;
    if(prescaler == 0U)
    {
        prescaler = 1U;
    }

    ticks = s_timer_clock / (prescaler * MOTOR_PWM_FREQUENCY_HZ);
    if(ticks == 0U)
    {
        ticks = 1U;
    }
    else if(ticks > 65536U)
    {
        ticks = 65536U;
    }

    s_pwm_period = (uint16_t)(ticks - 1U);

    time_base.TIM_Prescaler = (uint16_t)(prescaler - 1U);
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Period = s_pwm_period;
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(tim, &time_base);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = 0U;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(tim, &oc);
    TIM_OC2Init(tim, &oc);
    TIM_OC3Init(tim, &oc);
    TIM_OC4Init(tim, &oc);

    TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(tim, ENABLE);

    TIM_SetCompare1(tim, 0U);
    TIM_SetCompare2(tim, 0U);
    TIM_SetCompare3(tim, 0U);
    TIM_SetCompare4(tim, 0U);

    TIM_Cmd(tim, ENABLE);
}

void MOTOR_Init(void)
{
    uint8_t motor;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);

    /*
     * TIM2 partial remap 1 moves CH1/CH2 to PA15/PB3, TIM4 is fully remapped
     * to PD12-PD15.
     *
     * PA15/PB3 need no debug-port work on CH32V307: the chip only implements
     * the 2-wire SDI (SWDIO = PA13, SWCLK = PA14) and its pin table lists
     * PA15 = TIM2_CH1_1 and PB3 = TIM2_CH2_1 as plain alternate functions.
     */
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

    for(motor = 0U; motor < MOTOR_COUNT; motor++)
    {
        Motor_SetPinMode(s_hw[motor].port_in1, s_hw[motor].pin_in1, GPIO_Mode_AF_PP);
        Motor_SetPinMode(s_hw[motor].port_in2, s_hw[motor].pin_in2, GPIO_Mode_AF_PP);
        s_invert[motor] = s_invert_default[motor];
        s_target[motor] = 0;
        s_applied[motor] = 0;
    }

    Motor_UpdateClock();
    Motor_InitTimer(TIM2);
    Motor_InitTimer(TIM4);
    MOTOR_CoastAll();
}

void MOTOR_SetDuty(uint8_t motor, int16_t duty)
{
    if(motor >= MOTOR_COUNT)
    {
        return;
    }

    if(duty > (int16_t)MOTOR_DUTY_MAX)
    {
        duty = (int16_t)MOTOR_DUTY_MAX;
    }
    else if(duty < -(int16_t)MOTOR_DUTY_MAX)
    {
        duty = -(int16_t)MOTOR_DUTY_MAX;
    }

    s_target[motor] = duty;

    if(s_slew <= 0)
    {
        s_applied[motor] = duty;
        Motor_WriteBridge(motor, duty);
    }
}

void MOTOR_Update(void)
{
    uint8_t motor;

    for(motor = 0U; motor < MOTOR_COUNT; motor++)
    {
        int32_t applied = s_applied[motor];

        if(s_slew > 0)
        {
            int32_t delta = (int32_t)s_target[motor] - applied;

            if(delta > s_slew)
            {
                delta = s_slew;
            }
            else if(delta < -s_slew)
            {
                delta = -s_slew;
            }

            applied += delta;
            s_applied[motor] = (int16_t)applied;
        }
        else
        {
            s_applied[motor] = s_target[motor];
        }

        Motor_WriteBridge(motor, s_applied[motor]);
    }
}

void MOTOR_Brake(uint8_t motor)
{
    if(motor >= MOTOR_COUNT)
    {
        return;
    }

    /* A compare above ARR keeps a PWM channel steadily high. */
    Motor_SetCompare(s_hw[motor].tim, s_hw[motor].ch_in1, (uint16_t)(s_pwm_period + 1U));

    if(s_mode == MOTOR_MODE_PWM_DIR)
    {
        GPIO_SetBits(s_hw[motor].port_in2, s_hw[motor].pin_in2);
    }
    else
    {
        Motor_SetCompare(s_hw[motor].tim, s_hw[motor].ch_in2, (uint16_t)(s_pwm_period + 1U));
    }

    s_target[motor] = 0;
    s_applied[motor] = 0;
}

void MOTOR_Coast(uint8_t motor)
{
    if(motor >= MOTOR_COUNT)
    {
        return;
    }

    Motor_SetCompare(s_hw[motor].tim, s_hw[motor].ch_in1, 0U);

    if(s_mode == MOTOR_MODE_PWM_DIR)
    {
        GPIO_ResetBits(s_hw[motor].port_in2, s_hw[motor].pin_in2);
    }
    else
    {
        Motor_SetCompare(s_hw[motor].tim, s_hw[motor].ch_in2, 0U);
    }

    s_target[motor] = 0;
    s_applied[motor] = 0;
}

void MOTOR_BrakeAll(void)
{
    uint8_t motor;

    for(motor = 0U; motor < MOTOR_COUNT; motor++)
    {
        MOTOR_Brake(motor);
    }
}

void MOTOR_CoastAll(void)
{
    uint8_t motor;

    for(motor = 0U; motor < MOTOR_COUNT; motor++)
    {
        MOTOR_Coast(motor);
    }
}

void MOTOR_SetDriveMode(motor_mode_t mode)
{
    uint8_t motor;

    if(mode == s_mode)
    {
        return;
    }

    s_mode = mode;

    for(motor = 0U; motor < MOTOR_COUNT; motor++)
    {
        if(mode == MOTOR_MODE_DUAL_PWM)
        {
            Motor_SetPinMode(s_hw[motor].port_in2, s_hw[motor].pin_in2, GPIO_Mode_AF_PP);
            TIM_CCxCmd(s_hw[motor].tim, s_hw[motor].ch_in2, TIM_CCx_Enable);
        }
        else
        {
            TIM_CCxCmd(s_hw[motor].tim, s_hw[motor].ch_in2, TIM_CCx_Disable);
            Motor_SetPinMode(s_hw[motor].port_in2, s_hw[motor].pin_in2, GPIO_Mode_Out_PP);
            GPIO_ResetBits(s_hw[motor].port_in2, s_hw[motor].pin_in2);
        }

        Motor_WriteBridge(motor, s_applied[motor]);
    }
}

motor_mode_t MOTOR_GetDriveMode(void)
{
    return s_mode;
}

void MOTOR_SetInvert(uint8_t motor, uint8_t invert)
{
    if(motor >= MOTOR_COUNT)
    {
        return;
    }

    s_invert[motor] = (invert != 0U) ? 1U : 0U;
    Motor_WriteBridge(motor, s_applied[motor]);
}

uint8_t MOTOR_GetInvert(uint8_t motor)
{
    return (motor < MOTOR_COUNT) ? s_invert[motor] : 0U;
}

void MOTOR_SetSlewLimit(int16_t counts_per_period)
{
    if(counts_per_period < 0)
    {
        counts_per_period = 0;
    }
    else if(counts_per_period > (int16_t)MOTOR_DUTY_MAX)
    {
        counts_per_period = (int16_t)MOTOR_DUTY_MAX;
    }

    s_slew = counts_per_period;
}

int16_t MOTOR_GetSlewLimit(void)
{
    return s_slew;
}

int16_t MOTOR_GetDuty(uint8_t motor)
{
    return (motor < MOTOR_COUNT) ? s_applied[motor] : 0;
}

uint32_t MOTOR_GetTimerClockHz(void)
{
    return s_timer_clock;
}

uint16_t MOTOR_GetPwmPeriod(void)
{
    return s_pwm_period;
}
