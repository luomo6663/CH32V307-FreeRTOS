/********************************** (C) COPYRIGHT *******************************
* File Name          : mecanum.c
* Description        : Four wheel mecanum kinematics, speed limiting and odometry.
*******************************************************************************/

#include "mecanum.h"
#include <math.h>

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif

/*
 * Logical wheel order (FL, FR, RL, RR) to the motor order in motor.h
 * (M0 right rear, M1 right front, M2 left front, M3 left rear).
 */
static const uint8_t s_wheel_to_motor[MECANUM_WHEEL_COUNT] =
{
    2U,     /* front left  -> M2, wheel 3 */
    1U,     /* front right -> M1, wheel 2 */
    3U,     /* rear left   -> M3, wheel 4 */
    0U      /* rear right  -> M0, wheel 1 */
};

static float  s_radius_mm      = ROBOT_WHEEL_RADIUS_MM;
static float  s_half_length_mm = ROBOT_HALF_LENGTH_MM;
static float  s_half_width_mm  = ROBOT_HALF_WIDTH_MM;
static int8_t s_wheel_sign[MECANUM_WHEEL_COUNT] = {1, 1, 1, 1};

void Mecanum_Init(float wheel_radius_mm, float half_length_mm,
                  float half_width_mm)
{
    if(wheel_radius_mm > 0.0f)
    {
        s_radius_mm = wheel_radius_mm;
    }

    if(half_length_mm > 0.0f)
    {
        s_half_length_mm = half_length_mm;
    }

    if(half_width_mm > 0.0f)
    {
        s_half_width_mm = half_width_mm;
    }
}

void Mecanum_SetWheelSign(uint8_t logical_wheel, int8_t sign)
{
    if(logical_wheel >= MECANUM_WHEEL_COUNT)
    {
        return;
    }

    s_wheel_sign[logical_wheel] = (sign < 0) ? -1 : 1;
}

int8_t Mecanum_GetWheelSign(uint8_t logical_wheel)
{
    return (logical_wheel < MECANUM_WHEEL_COUNT) ? s_wheel_sign[logical_wheel] : 1;
}

uint8_t Mecanum_WheelToMotor(uint8_t logical_wheel)
{
    return (logical_wheel < MECANUM_WHEEL_COUNT)
               ? s_wheel_to_motor[logical_wheel]
               : 0U;
}

/*
 * Standard X-roller arrangement. The yaw term uses half length plus half
 * width, the usual simplification for a square-ish four wheel base.
 */
void Mecanum_TwistToWheels(float vx, float vy, float wz,
                           float wheel_mm_s[MECANUM_WHEEL_COUNT])
{
    float k = s_half_length_mm + s_half_width_mm;

    wheel_mm_s[MECANUM_FL] = (vx - vy - (k * wz)) * (float)s_wheel_sign[MECANUM_FL];
    wheel_mm_s[MECANUM_FR] = (vx + vy + (k * wz)) * (float)s_wheel_sign[MECANUM_FR];
    wheel_mm_s[MECANUM_RL] = (vx + vy - (k * wz)) * (float)s_wheel_sign[MECANUM_RL];
    wheel_mm_s[MECANUM_RR] = (vx - vy + (k * wz)) * (float)s_wheel_sign[MECANUM_RR];
}

void Mecanum_TwistToMotors(float vx, float vy, float wz,
                           float motor_mm_s[MECANUM_WHEEL_COUNT])
{
    uint8_t logical;
    float   wheel_mm_s[MECANUM_WHEEL_COUNT];

    Mecanum_TwistToWheels(vx, vy, wz, wheel_mm_s);

    for(logical = 0U; logical < MECANUM_WHEEL_COUNT; logical++)
    {
        motor_mm_s[s_wheel_to_motor[logical]] = wheel_mm_s[logical];
    }
}

void Mecanum_TwistToMotorsLimited(float vx, float vy, float wz,
                                  float max_wheel_mm_s,
                                  float motor_mm_s[MECANUM_WHEEL_COUNT])
{
    float peak = 0.0f;
    float scale = 1.0f;
    uint8_t index;

    Mecanum_TwistToMotors(vx, vy, wz, motor_mm_s);

    if(max_wheel_mm_s <= 0.0f)
    {
        return;
    }

    for(index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        float magnitude = fabsf(motor_mm_s[index]);

        if(magnitude > peak)
        {
            peak = magnitude;
        }
    }

    if(peak > max_wheel_mm_s)
    {
        scale = max_wheel_mm_s / peak;
    }

    for(index = 0U; index < MECANUM_WHEEL_COUNT; index++)
    {
        motor_mm_s[index] *= scale;
    }
}

void Mecanum_MotorsToTwist(const float motor_mm_s[MECANUM_WHEEL_COUNT],
                           float *vx, float *vy, float *wz)
{
    float   wheel_mm_s[MECANUM_WHEEL_COUNT];
    uint8_t logical;
    float   k = s_half_length_mm + s_half_width_mm;

    for(logical = 0U; logical < MECANUM_WHEEL_COUNT; logical++)
    {
        wheel_mm_s[logical] =
            motor_mm_s[s_wheel_to_motor[logical]] * (float)s_wheel_sign[logical];
    }

    if(vx != NULL)
    {
        *vx = (wheel_mm_s[MECANUM_FL] + wheel_mm_s[MECANUM_FR] +
               wheel_mm_s[MECANUM_RL] + wheel_mm_s[MECANUM_RR]) / 4.0f;
    }

    if(vy != NULL)
    {
        *vy = (-wheel_mm_s[MECANUM_FL] + wheel_mm_s[MECANUM_FR] +
               wheel_mm_s[MECANUM_RL] - wheel_mm_s[MECANUM_RR]) / 4.0f;
    }

    if(wz != NULL)
    {
        *wz = (-wheel_mm_s[MECANUM_FL] + wheel_mm_s[MECANUM_FR] -
               wheel_mm_s[MECANUM_RL] + wheel_mm_s[MECANUM_RR]) / (4.0f * k);
    }
}

float Mecanum_WheelMmSToRpm(float wheel_mm_s)
{
    return (wheel_mm_s / (2.0f * (float)M_PI * s_radius_mm)) * 60.0f;
}

float Mecanum_WheelRpmToMmS(float rpm)
{
    return (rpm / 60.0f) * 2.0f * (float)M_PI * s_radius_mm;
}

void Mecanum_ResetPose(mecanum_pose_t *pose)
{
    if(pose == NULL)
    {
        return;
    }

    pose->x_mm = 0.0f;
    pose->y_mm = 0.0f;
    pose->yaw_rad = 0.0f;
}

void Mecanum_IntegratePose(mecanum_pose_t *pose, float vx, float vy, float wz,
                           float dt_s)
{
    float yaw_mid;

    if((pose == NULL) || (dt_s <= 0.0f))
    {
        return;
    }

    /* Midpoint yaw keeps the path reasonable during a constant turn. */
    yaw_mid = pose->yaw_rad + (0.5f * wz * dt_s);

    pose->x_mm += ((vx * cosf(yaw_mid)) - (vy * sinf(yaw_mid))) * dt_s;
    pose->y_mm += ((vx * sinf(yaw_mid)) + (vy * cosf(yaw_mid))) * dt_s;
    pose->yaw_rad += wz * dt_s;

    while(pose->yaw_rad > (float)M_PI)
    {
        pose->yaw_rad -= (float)(2.0 * M_PI);
    }

    while(pose->yaw_rad < -(float)M_PI)
    {
        pose->yaw_rad += (float)(2.0 * M_PI);
    }
}
