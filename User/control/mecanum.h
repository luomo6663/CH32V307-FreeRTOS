#ifndef __MECANUM_H
#define __MECANUM_H

#include <stddef.h>
#include <stdint.h>
#include "config/robot_config.h"

/*
 * Four wheel mecanum kinematics and odometry. Hardware independent: it never
 * touches timers or GPIO.
 *
 * Frame: x forward, y left, yaw counter-clockwise seen from above.
 * Units: mm, mm/s, rad, rad/s.
 *
 * Logical wheel order:
 *   0 = front left, 1 = front right, 2 = rear left, 3 = rear right
 *
 * The motor order is defined by motor.h. Use Mecanum_TwistToMotors() before
 * touching MOTOR_SetDuty(); the returned array is indexed like motor.h.
 */

#define MECANUM_WHEEL_COUNT     4U

typedef enum
{
    MECANUM_FL = 0,
    MECANUM_FR = 1,
    MECANUM_RL = 2,
    MECANUM_RR = 3
} mecanum_wheel_t;

typedef struct
{
    float x_mm;
    float y_mm;
    float yaw_rad;
} mecanum_pose_t;

/* Geometry defaults to robot_config.h; call with measured values to override. */
void  Mecanum_Init(float wheel_radius_mm, float half_length_mm,
                   float half_width_mm);

/* Field calibration: pass -1 to flip the rolling direction of one wheel. */
void  Mecanum_SetWheelSign(uint8_t logical_wheel, int8_t sign);
int8_t Mecanum_GetWheelSign(uint8_t logical_wheel);

/* Logical wheel index (MECANUM_FL..MECANUM_RR) to motor index (0..3). */
uint8_t Mecanum_WheelToMotor(uint8_t logical_wheel);

/* Inverse kinematics: chassis twist to wheel surface speed in mm/s. */
void  Mecanum_TwistToWheels(float vx, float vy, float wz,
                            float wheel_mm_s[MECANUM_WHEEL_COUNT]);

/* Same, reordered and signed for the motor indices used by motor.h. */
void  Mecanum_TwistToMotors(float vx, float vy, float wz,
                            float motor_mm_s[MECANUM_WHEEL_COUNT]);

/*
 * Inverse kinematics plus a proportional speed limit: when one wheel would
 * exceed max_wheel_mm_s every wheel is scaled by the same factor, so the
 * commanded direction and turn ratio survive the clamp.
 */
void  Mecanum_TwistToMotorsLimited(float vx, float vy, float wz,
                                   float max_wheel_mm_s,
                                   float motor_mm_s[MECANUM_WHEEL_COUNT]);

/* Forward kinematics: motor surface speeds back to a chassis twist. */
void  Mecanum_MotorsToTwist(const float motor_mm_s[MECANUM_WHEEL_COUNT],
                            float *vx, float *vy, float *wz);

/* Unit helpers for a wheel of the configured radius. */
float Mecanum_WheelMmSToRpm(float wheel_mm_s);
float Mecanum_WheelRpmToMmS(float rpm);

void  Mecanum_ResetPose(mecanum_pose_t *pose);

/* Integrate one control step with the twist measured by forward kinematics. */
void  Mecanum_IntegratePose(mecanum_pose_t *pose, float vx, float vy, float wz,
                            float dt_s);

#endif /* __MECANUM_H */
