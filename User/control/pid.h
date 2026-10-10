#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stddef.h>

/*
 * Generic PI(D) controller used by the per-wheel speed loops.
 *
 * - The derivative acts on the measurement, not the error, so a setpoint step
 *   does not produce a kick.
 * - The integrator is clamped and frozen while the output is already on a
 *   rail, which removes wind-up.
 * - The derivative term is low-pass filtered with a 0..1 weight.
 */

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float d_state;
    float prev_measure;

    float out_min;
    float out_max;
    float integral_min;
    float integral_max;

    float deriv_filter;

    float p_term;
    float i_term;
    float d_term;

    uint8_t first_run;
} pid_controller_t;

void  PID_Init(pid_controller_t *pid, float kp, float ki, float kd,
               float out_min, float out_max);
void  PID_SetGains(pid_controller_t *pid, float kp, float ki, float kd);
void  PID_GetGains(const pid_controller_t *pid, float *kp, float *ki, float *kd);
void  PID_SetOutputLimits(pid_controller_t *pid, float out_min, float out_max);
void  PID_SetIntegralLimits(pid_controller_t *pid, float i_min, float i_max);
void  PID_SetDerivativeFilter(pid_controller_t *pid, float alpha);
void  PID_Reset(pid_controller_t *pid);

/* Last computed terms, useful for telemetry and commissioning. */
void  PID_GetTerms(const pid_controller_t *pid, float *p, float *i, float *d);

/* dt_s is in seconds. Returns the clamped control output. */
float PID_Update(pid_controller_t *pid, float setpoint, float measure, float dt_s);

#endif /* __PID_H */
