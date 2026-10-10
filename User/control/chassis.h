#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdint.h>
#include "wheel_id.h"
#include "mecanum.h"
#include "config/robot_config.h"

/*
 * Chassis layer: mode handling, mecanum mixing, per-wheel speed targets,
 * odometry and drive supervision.
 *
 * Threading model
 *   - CHASSIS_Update() is called from the control task only, every
 *     ROBOT_CONTROL_PERIOD_MS.
 *   - CHASSIS_Command*() may be called from any task. Commands are latched
 *     behind a short critical section and applied by the control task.
 */

typedef enum
{
    CHASSIS_MODE_IDLE = 0,  /* motors coasting, safe default                  */
    CHASSIS_MODE_TEST,      /* raw open-loop duty, bench bring-up             */
    CHASSIS_MODE_DRIVE,     /* closed-loop mecanum drive with command timeout */
    CHASSIS_MODE_SPEED      /* closed-loop per-wheel rpm bench test           */
} chassis_mode_t;

/* Fault flags returned by CHASSIS_GetStatus(). */
#define CHASSIS_FAULT_COMM_TIMEOUT      0x0001U
#define CHASSIS_FAULT_NOT_READY         0x0002U
#define CHASSIS_FAULT_WHEEL(w)          ((uint16_t)(0x0010U << (w)))

typedef struct
{
    uint8_t        mode;
    uint16_t       faults;

    float          vx_cmd;
    float          vy_cmd;
    float          wz_cmd;

    float          vx_meas;
    float          vy_meas;
    float          wz_meas;

    int16_t        rpm[WHEEL_COUNT];
    int16_t        target_rpm[WHEEL_COUNT];
    int16_t        duty[WHEEL_COUNT];
    uint8_t        wheel_state[WHEEL_COUNT];

    float          target_rpm_f[WHEEL_COUNT];
    float          pid_p[WHEEL_COUNT];
    float          pid_i[WHEEL_COUNT];
    float          pid_d[WHEEL_COUNT];

    mecanum_pose_t pose;
    float          omni_ready;

    uint32_t       updates;
} chassis_status_t;

void           CHASSIS_Init(void);
void           CHASSIS_Update(void);

void           CHASSIS_SetMode(chassis_mode_t mode);
chassis_mode_t CHASSIS_GetMode(void);

/* Command interface, safe to call from the console task. */
void           CHASSIS_CommandTwist(float vx, float vy, float wz);
void           CHASSIS_CommandOpenLoop(uint8_t wheel, int16_t duty);
void           CHASSIS_CommandWheelRpm(uint8_t wheel, float target_rpm);
void           CHASSIS_CommandStop(void);
void           CHASSIS_ClearFaults(void);
void           CHASSIS_ResetPose(void);

/* Status snapshot, safe to call from any task. */
void           CHASSIS_GetStatus(chassis_status_t *out);
uint16_t       CHASSIS_GetFaults(void);
uint8_t        CHASSIS_IsReady(void);

const char    *CHASSIS_ModeName(chassis_mode_t mode);
const char    *CHASSIS_WheelStateName(uint8_t state);

#endif /* __CHASSIS_H */
