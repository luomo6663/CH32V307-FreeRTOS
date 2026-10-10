/********************************** (C) COPYRIGHT *******************************
* File Name          : chassis.c
* Description        : Mecanum chassis control, odometry and drive supervision.
*******************************************************************************/

#include "chassis.h"
#include "encoder.h"
#include "motor.h"
#include "wheel_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

typedef struct
{
    uint8_t        set_twist;
    uint8_t        set_open_loop;
    uint8_t        open_loop_mask;
    uint8_t        set_speed;
    uint8_t        speed_mask;
    uint8_t        stop;
    uint8_t        clear_faults;
    uint8_t        reset_pose;

    float          vx;
    float          vy;
    float          wz;
    int16_t        open_loop[WHEEL_COUNT];
    float          target_rpm[WHEEL_COUNT];
} chassis_cmd_t;

static chassis_cmd_t    s_cmd;
static chassis_status_t s_status;

static chassis_mode_t   s_mode = CHASSIS_MODE_IDLE;
static chassis_mode_t   s_mode_applied = (chassis_mode_t)0xFF;
static uint16_t         s_faults = 0U;
static TickType_t       s_last_cmd_tick = 0U;
static TickType_t       s_last_tick = 0U;
static int16_t          s_test_duty[WHEEL_COUNT] = {0, 0, 0, 0};
static float            s_test_rpm[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float            s_twist[3] = {0.0f, 0.0f, 0.0f};

static float Chassis_Clamp(float value, float min, float max)
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

static uint16_t Chassis_WheelFaultFlags(void)
{
    uint16_t flags = 0U;
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        if(WHEEL_CONTROL_IsFaulted(wheel) != 0U)
        {
            flags |= CHASSIS_FAULT_WHEEL(wheel);
        }
    }

    return flags;
}

void CHASSIS_Init(void)
{
    MOTOR_Init();
    ENCODER_Init();
    WHEEL_CONTROL_Init();
    Mecanum_Init(ROBOT_WHEEL_RADIUS_MM, ROBOT_HALF_LENGTH_MM, ROBOT_HALF_WIDTH_MM);

    MOTOR_CoastAll();

    s_mode = CHASSIS_MODE_IDLE;
    s_mode_applied = (chassis_mode_t)0xFF;
    s_faults = 0U;
    s_last_tick = xTaskGetTickCount();
    s_last_cmd_tick = s_last_tick;

    s_status.mode = (uint8_t)s_mode;
    s_status.faults = 0U;
    s_status.updates = 0U;
    s_status.omni_ready = WHEEL_CONTROL_IsReady() ? 1.0f : 0.0f;
    Mecanum_ResetPose(&s_status.pose);
}

void CHASSIS_SetMode(chassis_mode_t mode)
{
    if((mode != CHASSIS_MODE_IDLE) &&
       (mode != CHASSIS_MODE_TEST) &&
       (mode != CHASSIS_MODE_DRIVE) &&
       (mode != CHASSIS_MODE_SPEED))
    {
        return;
    }

    if(mode == CHASSIS_MODE_DRIVE)
    {
        /* Restart the command watchdog so the caller has a full window. */
        s_last_cmd_tick = xTaskGetTickCount();
    }

    s_mode = mode;
}

chassis_mode_t CHASSIS_GetMode(void)
{
    return s_mode;
}

void CHASSIS_CommandTwist(float vx, float vy, float wz)
{
    taskENTER_CRITICAL();

    s_cmd.vx = Chassis_Clamp(vx, -ROBOT_MAX_LINEAR_MM_S, ROBOT_MAX_LINEAR_MM_S);
    s_cmd.vy = Chassis_Clamp(vy, -ROBOT_MAX_LINEAR_MM_S, ROBOT_MAX_LINEAR_MM_S);
    s_cmd.wz = Chassis_Clamp(wz, -ROBOT_MAX_YAW_RATE_RAD_S, ROBOT_MAX_YAW_RATE_RAD_S);
    s_cmd.set_twist = 1U;

    taskEXIT_CRITICAL();
}

void CHASSIS_CommandOpenLoop(uint8_t wheel, int16_t duty)
{
    if(wheel >= WHEEL_COUNT)
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

    taskENTER_CRITICAL();

    s_cmd.open_loop[wheel] = duty;
    s_cmd.open_loop_mask |= (uint8_t)(1U << wheel);
    s_cmd.set_open_loop = 1U;

    taskEXIT_CRITICAL();
}

void CHASSIS_CommandWheelRpm(uint8_t wheel, float target_rpm)
{
    if(wheel >= WHEEL_COUNT)
    {
        return;
    }

    if(fabsf(target_rpm) < ROBOT_PID_MIN_TARGET_RPM)
    {
        target_rpm = 0.0f;
    }

    if(target_rpm > ROBOT_MAX_WHEEL_RPM)
    {
        target_rpm = ROBOT_MAX_WHEEL_RPM;
    }
    else if(target_rpm < -ROBOT_MAX_WHEEL_RPM)
    {
        target_rpm = -ROBOT_MAX_WHEEL_RPM;
    }

    taskENTER_CRITICAL();

    s_cmd.target_rpm[wheel] = target_rpm;
    s_cmd.speed_mask |= (uint8_t)(1U << wheel);
    s_cmd.set_speed = 1U;

    taskEXIT_CRITICAL();
}

void CHASSIS_CommandStop(void)
{
    taskENTER_CRITICAL();

    s_cmd.stop = 1U;

    taskEXIT_CRITICAL();
}

void CHASSIS_ClearFaults(void)
{
    taskENTER_CRITICAL();

    s_cmd.clear_faults = 1U;

    taskEXIT_CRITICAL();
}

void CHASSIS_ResetPose(void)
{
    taskENTER_CRITICAL();

    s_cmd.reset_pose = 1U;

    taskEXIT_CRITICAL();
}

static void Chassis_ApplyStop(void)
{
    uint8_t wheel;

    WHEEL_CONTROL_Stop();

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        s_test_duty[wheel] = 0;
        s_test_rpm[wheel] = 0.0f;
    }

    s_twist[0] = 0.0f;
    s_twist[1] = 0.0f;
    s_twist[2] = 0.0f;

    MOTOR_CoastAll();
    s_mode = CHASSIS_MODE_IDLE;
}

static void Chassis_TakeCommands(void)
{
    chassis_cmd_t pending;
    uint8_t       open_loop_mask;
    uint8_t       speed_mask;

    taskENTER_CRITICAL();
    pending = s_cmd;
    open_loop_mask = s_cmd.open_loop_mask;
    speed_mask = s_cmd.speed_mask;
    s_cmd.set_twist = 0U;
    s_cmd.set_open_loop = 0U;
    s_cmd.open_loop_mask = 0U;
    s_cmd.set_speed = 0U;
    s_cmd.speed_mask = 0U;
    s_cmd.stop = 0U;
    s_cmd.clear_faults = 0U;
    s_cmd.reset_pose = 0U;
    taskEXIT_CRITICAL();

    if(pending.reset_pose != 0U)
    {
        Mecanum_ResetPose(&s_status.pose);
    }

    if(pending.clear_faults != 0U)
    {
        s_faults = 0U;
        WHEEL_CONTROL_ClearFaults();
    }

    if(pending.stop != 0U)
    {
        s_faults &= (uint16_t)~CHASSIS_FAULT_COMM_TIMEOUT;
        Chassis_ApplyStop();
    }

    if(pending.set_twist != 0U)
    {
        s_twist[0] = pending.vx;
        s_twist[1] = pending.vy;
        s_twist[2] = pending.wz;
        s_last_cmd_tick = xTaskGetTickCount();
        s_mode = CHASSIS_MODE_DRIVE;
    }

    if(pending.set_open_loop != 0U)
    {
        uint8_t wheel;

        if(s_mode != CHASSIS_MODE_TEST)
        {
            WHEEL_CONTROL_Stop();
        }

        for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
        {
            if((open_loop_mask & (uint8_t)(1U << wheel)) != 0U)
            {
                s_test_duty[wheel] = pending.open_loop[wheel];
            }
        }

        s_mode = CHASSIS_MODE_TEST;
    }

    if(pending.set_speed != 0U)
    {
        uint8_t wheel;

        if(s_mode != CHASSIS_MODE_SPEED)
        {
            WHEEL_CONTROL_Stop();
            WHEEL_CONTROL_ClearFaults();
            s_faults = 0U;
            for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
            {
                s_test_rpm[wheel] = 0.0f;
            }
        }

        for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
        {
            if((speed_mask & (uint8_t)(1U << wheel)) != 0U)
            {
                s_test_rpm[wheel] = pending.target_rpm[wheel];
            }
        }

        s_last_cmd_tick = xTaskGetTickCount();
        s_mode = CHASSIS_MODE_SPEED;
    }
}

static void Chassis_UpdateDrive(void)
{
    float   wheel_mm_s[WHEEL_COUNT];
    float   target_rpm[WHEEL_COUNT];
    float   max_wheel_mm_s = Mecanum_WheelRpmToMmS(ROBOT_MAX_WHEEL_RPM);
    uint8_t wheel;

    Mecanum_TwistToMotorsLimited(s_twist[0], s_twist[1], s_twist[2],
                                 max_wheel_mm_s, wheel_mm_s);

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        target_rpm[wheel] = Mecanum_WheelMmSToRpm(wheel_mm_s[wheel]);
    }

    WHEEL_CONTROL_SetTargets(target_rpm);
    WHEEL_CONTROL_Update();
}

static void Chassis_UpdateTest(void)
{
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        MOTOR_SetDuty(wheel, s_test_duty[wheel]);
    }
}

static void Chassis_UpdateSpeed(void)
{
    WHEEL_CONTROL_SetTargets(s_test_rpm);
    WHEEL_CONTROL_Update();
}

static void Chassis_UpdateOdometry(float dt_s)
{
    float   motor_mm_s[WHEEL_COUNT];
    float   vx;
    float   vy;
    float   wz;
    uint8_t wheel;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        motor_mm_s[wheel] = Mecanum_WheelRpmToMmS((float)ENCODER_GetRpm(wheel));
    }

    Mecanum_MotorsToTwist(motor_mm_s, &vx, &vy, &wz);
    Mecanum_IntegratePose(&s_status.pose, vx, vy, wz, dt_s);

    s_status.vx_meas = vx;
    s_status.vy_meas = vy;
    s_status.wz_meas = wz;
}

void CHASSIS_Update(void)
{
    TickType_t now;
    float      dt_s;
    uint8_t    wheel;

    Chassis_TakeCommands();
    ENCODER_Update();

    now = xTaskGetTickCount();
    dt_s = (float)(now - s_last_tick) / (float)configTICK_RATE_HZ;
    s_last_tick = now;
    if((dt_s <= 0.0f) || (dt_s > 0.25f))
    {
        dt_s = (float)ROBOT_CONTROL_PERIOD_MS / 1000.0f;
    }

    if(s_mode != s_mode_applied)
    {
        if(s_mode != CHASSIS_MODE_DRIVE)
        {
            WHEEL_CONTROL_Stop();
        }

        s_mode_applied = s_mode;
    }

    switch(s_mode)
    {
        case CHASSIS_MODE_TEST:
            Chassis_UpdateTest();
            break;

        case CHASSIS_MODE_DRIVE:
            if(WHEEL_CONTROL_IsReady() == 0U)
            {
                s_faults |= CHASSIS_FAULT_NOT_READY;
                Chassis_ApplyStop();
                s_mode_applied = s_mode;
            }
            else if((now - s_last_cmd_tick) >
                    (TickType_t)pdMS_TO_TICKS(ROBOT_COMMAND_TIMEOUT_MS))
            {
                s_faults |= CHASSIS_FAULT_COMM_TIMEOUT;
                Chassis_ApplyStop();
                s_mode_applied = s_mode;
            }
            else
            {
                Chassis_UpdateDrive();
            }
            break;

        case CHASSIS_MODE_SPEED:
            if((now - s_last_cmd_tick) >
               (TickType_t)pdMS_TO_TICKS(ROBOT_SPEED_TEST_TIMEOUT_MS))
            {
                s_faults |= CHASSIS_FAULT_COMM_TIMEOUT;
                Chassis_ApplyStop();
                s_mode_applied = s_mode;
            }
            else
            {
                Chassis_UpdateSpeed();
            }
            break;

        case CHASSIS_MODE_IDLE:
        default:
            break;
    }

    MOTOR_Update();

    /*
     * DRIVE is a shared motion command, so one fault stops the whole chassis.
     * SPEED is a per-wheel bench mode: a bad wheel coasts by itself while the
     * good wheels keep running for PID tuning.
     */
    if((WHEEL_CONTROL_AnyFault() != 0U) && (s_mode == CHASSIS_MODE_DRIVE))
    {
        Chassis_ApplyStop();
        s_mode_applied = s_mode;
    }

    s_faults |= Chassis_WheelFaultFlags();

    Chassis_UpdateOdometry(dt_s);

    s_status.mode = (uint8_t)s_mode;
    s_status.faults = s_faults;
    s_status.vx_cmd = s_twist[0];
    s_status.vy_cmd = s_twist[1];
    s_status.wz_cmd = s_twist[2];
    s_status.omni_ready = WHEEL_CONTROL_IsReady() ? 1.0f : 0.0f;
    s_status.updates++;

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        s_status.rpm[wheel] = ENCODER_GetRpm(wheel);
        s_status.target_rpm_f[wheel] = WHEEL_CONTROL_GetTargetRpm(wheel);
        s_status.target_rpm[wheel] = (int16_t)s_status.target_rpm_f[wheel];
        s_status.duty[wheel] = MOTOR_GetDuty(wheel);
        s_status.wheel_state[wheel] = (uint8_t)WHEEL_CONTROL_GetState(wheel);
        WHEEL_CONTROL_GetPidTerms(wheel,
                                  &s_status.pid_p[wheel],
                                  &s_status.pid_i[wheel],
                                  &s_status.pid_d[wheel]);
    }
}

void CHASSIS_GetStatus(chassis_status_t *out)
{
    if(out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_status;
    taskEXIT_CRITICAL();
}

uint16_t CHASSIS_GetFaults(void)
{
    uint16_t faults;

    taskENTER_CRITICAL();
    faults = s_faults;
    taskEXIT_CRITICAL();

    return faults;
}

uint8_t CHASSIS_IsReady(void)
{
    return WHEEL_CONTROL_IsReady();
}

const char *CHASSIS_ModeName(chassis_mode_t mode)
{
    switch(mode)
    {
        case CHASSIS_MODE_IDLE:
            return "IDLE";
        case CHASSIS_MODE_TEST:
            return "TEST";
        case CHASSIS_MODE_DRIVE:
            return "DRIVE";
        case CHASSIS_MODE_SPEED:
            return "SPEED";
        default:
            return "?";
    }
}

const char *CHASSIS_WheelStateName(uint8_t state)
{
    switch((wheel_state_t)state)
    {
        case WHEEL_STATE_DISABLED:
            return "disabled";
        case WHEEL_STATE_CLOSED_LOOP:
            return "closed";
        case WHEEL_STATE_OPEN_LOOP:
            return "open";
        case WHEEL_STATE_STALL:
            return "stall";
        case WHEEL_STATE_DIR_FAULT:
            return "dir";
        case WHEEL_STATE_NO_FEEDBACK:
            return "no-fb";
        default:
            return "?";
    }
}
