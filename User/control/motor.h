#ifndef __MOTOR_H
#define __MOTOR_H

#include "ch32v30x.h"
#include "wheel_id.h"
#include "config/robot_config.h"

/*
 * Four TMI8140 H-bridge channels driven with two PWM inputs each.
 *
 * Pin map (wheel order matches wheel_id.h, the board column is the power board
 * silkscreen MOTOR1..MOTOR4 / ENC1..ENC4):
 *   W1 right rear  IN1 PA2  (TIM2_CH3)  IN2 PA3  (TIM2_CH4), MOTOR3/ENC3
 *   W2 right front IN1 PA15 (TIM2_CH1)  IN2 PB3  (TIM2_CH2), MOTOR4/ENC4
 *   W3 left front  IN1 PD12 (TIM4_CH1)  IN2 PD13 (TIM4_CH2), MOTOR1/ENC1
 *   W4 left rear   IN1 PD14 (TIM4_CH3)  IN2 PD15 (TIM4_CH4), MOTOR2/ENC2
 *
 * Truth table in dual-PWM mode:
 *   duty > 0 : IN1 = PWM, IN2 = 0
 *   duty < 0 : IN1 = 0,   IN2 = PWM
 *   brake    : IN1 = IN2 = high
 *   coast    : IN1 = IN2 = low
 *
 * Truth table in PWM + DIR mode:
 *   duty > 0 : IN1 = PWM,           IN2 = 0
 *   duty < 0 : IN1 = period - PWM,  IN2 = 1
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

/* Reported wiring of one wheel, for the PINS console command. */
typedef struct
{
    const char *timer;
    const char *in1_port;
    uint8_t     in1_pin;
    uint8_t     in1_channel;
    const char *in2_port;
    uint8_t     in2_pin;
    uint8_t     in2_channel;    /* 0 = IN2 is a plain GPIO pin */
    uint8_t     pwm_dir;        /* 1 = this wheel always runs PWM + DIR */
} motor_wiring_t;

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

/* Copies the wiring of one wheel; false when the index is out of range. */
uint8_t      MOTOR_GetWiring(uint8_t motor, motor_wiring_t *out);

void         MOTOR_SetInvert(uint8_t motor, uint8_t invert);
uint8_t      MOTOR_GetInvert(uint8_t motor);

void         MOTOR_SetSlewLimit(int16_t counts_per_period);
int16_t      MOTOR_GetSlewLimit(void);

int16_t      MOTOR_GetDuty(uint8_t motor);
uint32_t     MOTOR_GetTimerClockHz(void);
uint16_t     MOTOR_GetPwmPeriod(void);

#endif /* __MOTOR_H */
