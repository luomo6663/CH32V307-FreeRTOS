#ifndef __MOTOR_H
#define __MOTOR_H

#include "ch32v30x.h"
#include "wheel_id.h"
#include "config/robot_config.h"

/*
 * Four TMI8140 H-bridge channels driven with two PWM inputs each.
 *
 * Pin map (wheel order matches wheel_id.h):
 *   W1 right rear  IN1 PA2  (TIM2_CH3)  IN2 PA3  (TIM2_CH4)
 *   W2 right front IN1 PA15 (TIM2_CH1)  IN2 PB3  (TIM2_CH2)
 *   W3 left front  IN1 PD12 (TIM4_CH1)  IN2 PD13 (TIM4_CH2)
 *   W4 left rear   IN1 PD14 (TIM4_CH3)  IN2 PD15 (TIM4_CH4)
 *
 * Truth table in dual-PWM mode:
 *   duty > 0 : IN1 = PWM, IN2 = 0
 *   duty < 0 : IN1 = 0,   IN2 = PWM
 *   brake    : IN1 = IN2 = high
 *   coast    : IN1 = IN2 = low
 */

#define MOTOR_COUNT                 WHEEL_COUNT
#define MOTOR_DUTY_MAX              ROBOT_MOTOR_DUTY_MAX
#define MOTOR_PWM_FREQUENCY_HZ      ROBOT_MOTOR_PWM_FREQUENCY_HZ
#define MOTOR_DEFAULT_SLEW          ROBOT_MOTOR_DEFAULT_SLEW

typedef enum
{
    MOTOR_MODE_DUAL_PWM = 0,    /* IN1/IN2 both PWM (default wiring)   */
    MOTOR_MODE_PWM_DIR  = 1     /* IN1 = PWM, IN2 = static direction   */
} motor_mode_t;

void         MOTOR_Init(void);

/* Requested duty; with slew limiting enabled MOTOR_Update() walks to it. */
void         MOTOR_SetDuty(uint8_t motor, int16_t duty);

/* Applies the slew-limited duty of every wheel. Call once per control period. */
void         MOTOR_Update(void);

void         MOTOR_Brake(uint8_t motor);
void         MOTOR_Coast(uint8_t motor);
void         MOTOR_BrakeAll(void);
void         MOTOR_CoastAll(void);

void         MOTOR_SetDriveMode(motor_mode_t mode);
motor_mode_t MOTOR_GetDriveMode(void);

void         MOTOR_SetInvert(uint8_t motor, uint8_t invert);
uint8_t      MOTOR_GetInvert(uint8_t motor);

void         MOTOR_SetSlewLimit(int16_t counts_per_period);
int16_t      MOTOR_GetSlewLimit(void);

int16_t      MOTOR_GetDuty(uint8_t motor);
uint32_t     MOTOR_GetTimerClockHz(void);
uint16_t     MOTOR_GetPwmPeriod(void);

#endif /* __MOTOR_H */
