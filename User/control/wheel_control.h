#ifndef __WHEEL_CONTROL_H
#define __WHEEL_CONTROL_H

#include <stdint.h>
#include <stddef.h>
#include "wheel_id.h"
#include "config/robot_config.h"

/*
 * Per-wheel speed loop.
 *
 * Wheels with a quadrature input run a PI(D) loop on the measured rpm.
 * A wheel without a quadrature input can run open loop from a calibrated
 * duty-per-rpm gain; with the gain left at 0 it reports NO_FEEDBACK and the
 * caller is expected to reject the move instead of driving blind.
 */

typedef enum
{
    WHEEL_STATE_DISABLED = 0,   /* no target, bridge coasting                 */
    WHEEL_STATE_CLOSED_LOOP,    /* PI(D) on encoder feedback                  */
    WHEEL_STATE_OPEN_LOOP,      /* feed-forward duty, no usable feedback      */
    WHEEL_STATE_STALL,          /* latched: commanded, but the wheel is stuck */
    WHEEL_STATE_DIR_FAULT,      /* latched: feedback opposes the command      */
    WHEEL_STATE_NO_FEEDBACK     /* latched: no feedback and no fallback gain  */
} wheel_state_t;

void          WHEEL_CONTROL_Init(void);
void          WHEEL_CONTROL_Update(void);

/* Zero every target and coast. Clears the state but keeps latched faults. */
void          WHEEL_CONTROL_Stop(void);

void          WHEEL_CONTROL_SetTargets(const float target_rpm[WHEEL_COUNT]);
void          WHEEL_CONTROL_SetTargetRpm(uint8_t wheel, float target_rpm);
float         WHEEL_CONTROL_GetTargetRpm(uint8_t wheel);

void          WHEEL_CONTROL_SetGains(uint8_t wheel, float kp, float ki, float kd);
void          WHEEL_CONTROL_GetGains(uint8_t wheel, float *kp, float *ki, float *kd);
void          WHEEL_CONTROL_GetPidTerms(uint8_t wheel, float *p, float *i, float *d);
void          WHEEL_CONTROL_SetOpenLoopGain(uint8_t wheel, float duty_per_rpm);
float         WHEEL_CONTROL_GetOpenLoopGain(uint8_t wheel);

int16_t       WHEEL_CONTROL_GetOutput(uint8_t wheel);
int16_t       WHEEL_CONTROL_GetMeasuredRpm(uint8_t wheel);
wheel_state_t WHEEL_CONTROL_GetState(uint8_t wheel);
uint8_t       WHEEL_CONTROL_IsFaulted(uint8_t wheel);
uint8_t       WHEEL_CONTROL_AnyFault(void);

/* 1 when every wheel can be driven with usable feedback or a fallback gain. */
uint8_t       WHEEL_CONTROL_IsReady(void);

/* Clears latched faults so the next non-zero target is attempted again. */
void          WHEEL_CONTROL_ClearFaults(void);

#endif /* __WHEEL_CONTROL_H */
