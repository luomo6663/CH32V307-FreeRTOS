/********************************** (C) COPYRIGHT *******************************
* File Name          : pid.c
* Description        : Generic PI(D) controller for the wheel speed loops.
*******************************************************************************/

#include "pid.h"

static float PID_Clamp(float value, float min, float max)
{
    if(value < min)
    {
        return min;
    }

    if(value > max)
    {
        return max;
    }

    return value;
}

void PID_Init(pid_controller_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = out_min;
    pid->integral_max = out_max;

    pid->deriv_filter = 1.0f;

    PID_Reset(pid);
}

void PID_SetGains(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_GetGains(const pid_controller_t *pid, float *kp, float *ki, float *kd)
{
    if(kp != NULL)
    {
        *kp = pid->kp;
    }
    if(ki != NULL)
    {
        *ki = pid->ki;
    }
    if(kd != NULL)
    {
        *kd = pid->kd;
    }
}

void PID_SetOutputLimits(pid_controller_t *pid, float out_min, float out_max)
{
    if(out_min > out_max)
    {
        float tmp = out_min;

        out_min = out_max;
        out_max = tmp;
    }

    pid->out_min = out_min;
    pid->out_max = out_max;

    /* Keep the integrator inside the new window as well. */
    pid->integral_min = PID_Clamp(pid->integral_min, out_min, out_max);
    pid->integral_max = PID_Clamp(pid->integral_max, out_min, out_max);
    pid->integral = PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
}

void PID_SetIntegralLimits(pid_controller_t *pid, float i_min, float i_max)
{
    if(i_min > i_max)
    {
        float tmp = i_min;

        i_min = i_max;
        i_max = tmp;
    }

    pid->integral_min = PID_Clamp(i_min, pid->out_min, pid->out_max);
    pid->integral_max = PID_Clamp(i_max, pid->out_min, pid->out_max);
    pid->integral = PID_Clamp(pid->integral, pid->integral_min, pid->integral_max);
}

void PID_SetDerivativeFilter(pid_controller_t *pid, float alpha)
{
    if(alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if(alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    pid->deriv_filter = alpha;
}

void PID_Reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->d_state = 0.0f;
    pid->prev_measure = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->first_run = 1U;
}

void PID_GetTerms(const pid_controller_t *pid, float *p, float *i, float *d)
{
    if(p != NULL)
    {
        *p = pid->p_term;
    }
    if(i != NULL)
    {
        *i = pid->i_term;
    }
    if(d != NULL)
    {
        *d = pid->d_term;
    }
}

float PID_Update(pid_controller_t *pid, float setpoint, float measure, float dt_s)
{
    float error;
    float output;

    if(dt_s <= 0.0f)
    {
        return 0.0f;
    }

    error = setpoint - measure;

    /*
     * The first sample only seeds the derivative memory; otherwise the whole
     * measurement would look like a step and produce a huge spike.
     */
    if(pid->first_run != 0U)
    {
        pid->prev_measure = measure;
        pid->first_run = 0U;
    }

    pid->p_term = pid->kp * error;

    if(pid->kd != 0.0f)
    {
        float raw = (-pid->kd * (measure - pid->prev_measure)) / dt_s;

        pid->d_state += pid->deriv_filter * (raw - pid->d_state);
    }
    else
    {
        pid->d_state = 0.0f;
    }

    pid->d_term = pid->d_state;
    pid->i_term = pid->integral;

    output = pid->p_term + pid->i_term + pid->d_term;

    if(((output >= pid->out_max) && (error > 0.0f)) ||
       ((output <= pid->out_min) && (error < 0.0f)))
    {
        /* Hold the integrator while the output is already saturated. */
    }
    else
    {
        pid->integral = PID_Clamp(pid->integral + (pid->ki * error * dt_s),
                                  pid->integral_min,
                                  pid->integral_max);
        pid->i_term = pid->integral;
        output = pid->p_term + pid->i_term + pid->d_term;
    }

    output = PID_Clamp(output, pid->out_min, pid->out_max);

    pid->prev_measure = measure;

    return output;
}
