#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>
#include "config/robot_config.h"

/*
 * Single hobby servo output.
 *
 *   PB5 -> TIM3_CH2 (TIM3 partial remap), 50 Hz
 *
 * The TBS-K20 accepts 50-330 Hz and 500-2500 us pulses; 1500 us is center.
 *
 * Power the servo from a separate 5-8.4 V supply and connect its ground to the
 * board ground. PB5 only carries the 3.3 V PWM signal.
 */

#define SERVO_PWM_FREQUENCY_HZ      ROBOT_SERVO_PWM_FREQUENCY_HZ
#define SERVO_SWEEP_MIN_US          ROBOT_SERVO_SWEEP_MIN_US
#define SERVO_SWEEP_MAX_US          ROBOT_SERVO_SWEEP_MAX_US
#define SERVO_SWEEP_PERIOD_MS       ROBOT_SERVO_SWEEP_PERIOD_MS

typedef struct
{
    uint16_t pulse_us;
    uint16_t prescaler;
    uint16_t period;
    uint16_t compare;
    uint16_t timer_control;
    uint8_t  sweeping;
    uint32_t afio_pcfr1;
    uint32_t gpio_cfglr;
} servo_status_t;

void     SERVO_Init(void);

/* Hold a fixed pulse width in microseconds and stop any active sweep. */
void     SERVO_SetPulseUs(uint16_t pulse_us);

/* Set the pulse range and period, then start the non-blocking sweep. */
void     SERVO_StartSweep(uint16_t min_pulse_us, uint16_t max_pulse_us,
                          uint32_t period_ms);
void     SERVO_StopSweep(void);

/* Call from the control task once per ROBOT_CONTROL_PERIOD_MS. */
void     SERVO_Update(void);

uint16_t SERVO_GetPulseUs(void);
uint8_t  SERVO_IsSweeping(void);
void     SERVO_GetStatus(servo_status_t *out);

#endif /* __SERVO_H */
