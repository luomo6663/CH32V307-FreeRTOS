/********************************** (C) COPYRIGHT *******************************
* File Name          : wheel_control.c
* Description        : Per-wheel speed loop with stall and direction supervision.
*******************************************************************************/

#include "wheel_control.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include <math.h>

typedef struct
{
    pid_controller_t pid;
    float            target_rpm;
    float            open_loop_gain;    /* duty per rpm, 0 = not available     */
    int16_t          measured_rpm;
    float            measured_filtered;
    int16_t          output_duty;
    uint16_t         stall_count;
    uint16_t         dir_fault_count;
    wheel_state_t    state;
} wheel_loop_t;

static wheel_loop_t s_loop[WHEEL_COUNT];

static const float s_open_loop_default[WHEEL_COUNT] =
{
    0.0f,                           /* W1 right rear : closed loop          */
    0.0f,                           /* W2 right front: closed loop          */
    ROBOT_WHEEL3_OPEN_LOOP_GAIN,    /* W3 left front : no speed feedback    */
    0.0f                            /* W4 left rear  : closed loop          */
};

static const float s_kp_default[WHEEL_COUNT] =
{
    ROBOT_PID_KP_W1,
    ROBOT_PID_KP_W2,
    ROBOT_PID_KP_W3,
    ROBOT_PID_KP_W4
};

static const float s_ki_default[WHEEL_COUNT] =
{
    ROBOT_PID_KI_W1,
    ROBOT_PID_KI_W2,
    ROBOT_PID_KI_W3,
    ROBOT_PID_KI_W4
};

static const float s_kd_default[WHEEL_COUNT] =
{
    ROBOT_PID_KD_W1,
    ROBOT_PID_KD_W2,
    ROBOT_PID_KD_W3,
    ROBOT_PID_KD_W4
};

static int8_t WheelControl_Sign(float value)
{
    if(value > 0.0f)
    {
        return 1;
    }

    if(value < 0.0f)
    {
        return -1;
    }

    return 0;
}

static uint8_t WheelControl_IsFaultState(wheel_state_t state)
{
    return ((state == WHEEL_STATE_STALL) ||
            (state == WHEEL_STATE_DIR_FAULT) ||
            (state == WHEEL_STATE_NO_FEEDBACK)) ? 1U : 0U;
}

void WHEEL_CONTROL_Init(void)
{
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        PID_Init(&s_loop[wheel].pid,
                 s_kp_default[wheel],
                 s_ki_default[wheel],
                 s_kd_default[wheel],
                 ROBOT_PID_OUTPUT_MIN,
                 ROBOT_PID_OUTPUT_MAX);
        PID_SetDerivativeFilter(&s_loop[wheel].pid, ROBOT_PID_DERIV_FILTER);

        s_loop[wheel].target_rpm = 0.0f;
        s_loop[wheel].open_loop_gain = s_open_loop_default[wheel];
        s_loop[wheel].measured_rpm = 0;
        s_loop[wheel].measured_filtered = 0.0f;
        s_loop[wheel].output_duty = 0;
        s_loop[wheel].stall_count = 0U;
        s_loop[wheel].dir_fault_count = 0U;
        s_loop[wheel].state = WHEEL_STATE_DISABLED;
    }
}

void WHEEL_CONTROL_Stop(void)
{
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        s_loop[wheel].target_rpm = 0.0f;
        s_loop[wheel].measured_rpm = 0;
        s_loop[wheel].measured_filtered = 0.0f;
        s_loop[wheel].output_duty = 0;
        s_loop[wheel].stall_count = 0U;
        s_loop[wheel].dir_fault_count = 0U;

        if(WheelControl_IsFaultState(s_loop[wheel].state) == 0U)
        {
            s_loop[wheel].state = WHEEL_STATE_DISABLED;
        }

        PID_Reset(&s_loop[wheel].pid);
        MOTOR_Coast(wheel);
    }
}

void WHEEL_CONTROL_SetTargetRpm(uint8_t wheel, float target_rpm)
{
    float previous;

    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    if(fabsf(target_rpm) < ROBOT_PID_MIN_TARGET_RPM)
    {
        target_rpm = 0.0f;
    }

    previous = s_loop[wheel].target_rpm;
    if(fabsf(previous - target_rpm) < 0.25f)
    {
        return;
    }

    s_loop[wheel].target_rpm = target_rpm;
    s_loop[wheel].stall_count = 0U;
    s_loop[wheel].dir_fault_count = 0U;

    /* A direction change invalidates the old integral state. */
    if(WheelControl_Sign(previous) != WheelControl_Sign(target_rpm))
    {
        PID_Reset(&s_loop[wheel].pid);
        s_loop[wheel].measured_filtered = (float)s_loop[wheel].measured_rpm;
    }
}

void WHEEL_CONTROL_SetTargets(const float target_rpm[WHEEL_COUNT])
{
    uint8_t wheel;

    if(target_rpm == NULL)
    {
        return;
    }

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        WHEEL_CONTROL_SetTargetRpm(wheel, target_rpm[wheel]);
    }
}

float WHEEL_CONTROL_GetTargetRpm(uint8_t wheel)
{
    return (wheel < WHEEL_COUNT) ? s_loop[wheel].target_rpm : 0.0f;
}

void WHEEL_CONTROL_SetGains(uint8_t wheel, float kp, float ki, float kd)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    PID_SetGains(&s_loop[wheel].pid, kp, ki, kd);
}

void WHEEL_CONTROL_GetGains(uint8_t wheel, float *kp, float *ki, float *kd)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    PID_GetGains(&s_loop[wheel].pid, kp, ki, kd);
}

void WHEEL_CONTROL_GetPidTerms(uint8_t wheel, float *p, float *i, float *d)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    PID_GetTerms(&s_loop[wheel].pid, p, i, d);
}

void WHEEL_CONTROL_SetOpenLoopGain(uint8_t wheel, float duty_per_rpm)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    if(duty_per_rpm < 0.0f)
    {
        duty_per_rpm = 0.0f;
    }
    else if(duty_per_rpm > 100.0f)
    {
        duty_per_rpm = 100.0f;
    }

    s_loop[wheel].open_loop_gain = duty_per_rpm;
}

float WHEEL_CONTROL_GetOpenLoopGain(uint8_t wheel)
{
    return (wheel < WHEEL_COUNT) ? s_loop[wheel].open_loop_gain : 0.0f;
}

static void WheelControl_ResetLoop(uint8_t wheel)
{
    s_loop[wheel].output_duty = 0;
    s_loop[wheel].stall_count = 0U;
    s_loop[wheel].dir_fault_count = 0U;
    s_loop[wheel].measured_filtered = 0.0f;
    PID_Reset(&s_loop[wheel].pid);
    MOTOR_Coast(wheel);
}

void WHEEL_CONTROL_Update(void)
{
    const float dt_s = (float)ENCODER_SAMPLE_PERIOD_MS / 1000.0f;
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        float target = s_loop[wheel].target_rpm;
        float measure;
        float output;

        {
            int16_t raw_rpm = ENCODER_GetRpm(wheel);

            if(fabsf((float)raw_rpm) <= ROBOT_PID_MEASURE_MAX_RPM)
            {
                s_loop[wheel].measured_filtered +=
                    ROBOT_PID_MEASURE_FILTER *
                    ((float)raw_rpm - s_loop[wheel].measured_filtered);
            }

            s_loop[wheel].measured_rpm =
                (int16_t)((s_loop[wheel].measured_filtered >= 0.0f)
                              ? (s_loop[wheel].measured_filtered + 0.5f)
                              : (s_loop[wheel].measured_filtered - 0.5f));
        }
        measure = s_loop[wheel].measured_filtered;

        if(WheelControl_IsFaultState(s_loop[wheel].state) != 0U)
        {
            s_loop[wheel].output_duty = 0;
            MOTOR_Coast(wheel);
            continue;
        }

        if(target == 0.0f)
        {
            s_loop[wheel].state = WHEEL_STATE_DISABLED;
            WheelControl_ResetLoop(wheel);
            continue;
        }

        if(ENCODER_HasQuadrature(wheel) != 0U)
        {
            output = PID_Update(&s_loop[wheel].pid, target, measure, dt_s);
            s_loop[wheel].state = WHEEL_STATE_CLOSED_LOOP;
        }
        else if(s_loop[wheel].open_loop_gain > 0.0f)
        {
            output = target * s_loop[wheel].open_loop_gain;
            s_loop[wheel].state = WHEEL_STATE_OPEN_LOOP;
        }
        else
        {
            s_loop[wheel].state = WHEEL_STATE_NO_FEEDBACK;
            WheelControl_ResetLoop(wheel);
            continue;
        }

        if(output > ROBOT_PID_OUTPUT_MAX)
        {
            output = ROBOT_PID_OUTPUT_MAX;
        }
        else if(output < ROBOT_PID_OUTPUT_MIN)
        {
            output = ROBOT_PID_OUTPUT_MIN;
        }

        s_loop[wheel].output_duty = (int16_t)output;
        MOTOR_SetDuty(wheel, s_loop[wheel].output_duty);

        if((fabsf(target) > ROBOT_PID_STALL_RPM) &&
           (fabsf(measure) < ROBOT_PID_STALL_RPM))
        {
            s_loop[wheel].stall_count++;
            if(s_loop[wheel].stall_count >= ROBOT_PID_STALL_LIMIT)
            {
                s_loop[wheel].state = WHEEL_STATE_STALL;
                WheelControl_ResetLoop(wheel);
            }
        }
        else
        {
            s_loop[wheel].stall_count = 0U;
        }

        if(((target > 0.0f) && (measure < -ROBOT_PID_STALL_RPM)) ||
           ((target < 0.0f) && (measure > ROBOT_PID_STALL_RPM)))
        {
            s_loop[wheel].dir_fault_count++;
            if(s_loop[wheel].dir_fault_count >= ROBOT_PID_DIR_FAULT_LIMIT)
            {
                s_loop[wheel].state = WHEEL_STATE_DIR_FAULT;
                WheelControl_ResetLoop(wheel);
            }
        }
        else
        {
            s_loop[wheel].dir_fault_count = 0U;
        }
    }
}

int16_t WHEEL_CONTROL_GetOutput(uint8_t wheel)
{
    return (wheel < WHEEL_COUNT) ? s_loop[wheel].output_duty : 0;
}

int16_t WHEEL_CONTROL_GetMeasuredRpm(uint8_t wheel)
{
    return (wheel < WHEEL_COUNT) ? s_loop[wheel].measured_rpm : 0;
}

wheel_state_t WHEEL_CONTROL_GetState(uint8_t wheel)
{
    return (wheel < WHEEL_COUNT) ? s_loop[wheel].state : WHEEL_STATE_DISABLED;
}

uint8_t WHEEL_CONTROL_IsFaulted(uint8_t wheel)
{
    if(wheel >= WHEEL_COUNT)
    {
        return 1U;
    }

    return WheelControl_IsFaultState(s_loop[wheel].state);
}

uint8_t WHEEL_CONTROL_AnyFault(void)
{
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        if(WheelControl_IsFaultState(s_loop[wheel].state) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

uint8_t WHEEL_CONTROL_IsReady(void)
{
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        if((ENCODER_HasQuadrature(wheel) == 0U) &&
           (s_loop[wheel].open_loop_gain <= 0.0f))
        {
            return 0U;
        }
    }

    return 1U;
}

void WHEEL_CONTROL_ClearFaults(void)
{
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        if(WheelControl_IsFaultState(s_loop[wheel].state) != 0U)
        {
            s_loop[wheel].state = WHEEL_STATE_DISABLED;
        }

        s_loop[wheel].stall_count = 0U;
        s_loop[wheel].dir_fault_count = 0U;
        PID_Reset(&s_loop[wheel].pid);
    }
}
