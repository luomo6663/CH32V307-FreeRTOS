#ifndef __SERVO_H
#define __SERVO_H
#include <stdint.h>
#include "config/robot_config.h"
/* Logical A/B/C = PB4/PB5/PB0 = TIM3 CH1/2/3; PD2 selects bank.
 * Real R = hardware L (H5,H6,H7); real L = hardware R (H21,H3,H4).
 * Only the selected bank receives pulses. No position feedback is available. */
#define SERVO_CHANNEL_COUNT 3U
#define SERVO_MASK_ALL 7U
#define SERVO_PWM_FREQUENCY_HZ ROBOT_SERVO_PWM_FREQUENCY_HZ
#define SERVO_SWEEP_MIN_US ROBOT_SERVO_SWEEP_MIN_US
#define SERVO_SWEEP_MAX_US ROBOT_SERVO_SWEEP_MAX_US
#define SERVO_SWEEP_PERIOD_MS ROBOT_SERVO_SWEEP_PERIOD_MS
typedef enum { SERVO_OK, SERVO_ERR_RANGE, SERVO_ERR_UNAVAILABLE,
               SERVO_ERR_QUEUE_FULL, SERVO_ERR_CONFIG } servo_result_t;
typedef enum { SERVO_HOLD, SERVO_MOVING, SERVO_SWEEPING } servo_motion_t;
typedef enum { SERVO_ACTION_NONE, SERVO_ACTION_RESET, SERVO_ACTION_EXTEND,
    SERVO_ACTION_RETRACT, SERVO_ACTION_OPEN, SERVO_ACTION_GRIP,
    SERVO_ACTION_CLAMP, SERVO_ACTION_PICK } servo_action_t;
servo_result_t SERVO_Action(servo_action_t action, uint32_t ms);
const char *SERVO_ActionName(servo_action_t action);

typedef struct {
    uint16_t pulse_us[3]; /* Software output command, NOT measured position. */
    uint16_t compare[3]; /* Timer preload registers, NOT feedback. */
    uint16_t saved_us[2][3], target_us[2][3];
    uint8_t motion[2][3], enabled[2][3], installed_mask[2];
    uint8_t boot_test_phase[2][3]; /* 0=inactive, 1=min, 2=max, 3=center. */
    uint16_t prescaler, period, timer_control;
    uint8_t sweeping, side_right, requested_side_right, hardware_side_right;
    uint8_t queued, config_valid, action, pick_step;
    uint32_t timer_hz, afio_pcfr1, gpio_cfglr, gpiod_odr;
} servo_status_t;
void SERVO_Init(void);
void SERVO_Update(void); /* Sole task owner; call at 100 Hz. */
/* OK means queued, not physically done. ALL addresses installed channels only.
 * Validation uses the side selected by preceding queued SIDE commands. */
servo_result_t SERVO_SetPulseUs(uint16_t us); /* Legacy channel B. */
servo_result_t SERVO_SetChannelPulseUs(uint8_t channel, uint16_t us);
servo_result_t SERVO_SetAllPulseUs(uint16_t us);
servo_result_t SERVO_Jog(uint8_t channel, int16_t delta_us, uint32_t ms);
servo_result_t SERVO_Move(uint8_t mask, const uint16_t us[3], uint32_t ms);
/* code_2 convention: 0 degrees = 1500 us, range -135..135. */
servo_result_t SERVO_MoveAngle(uint8_t mask, int16_t degrees, uint32_t ms);
servo_result_t SERVO_Center(uint8_t mask);
servo_result_t SERVO_Off(uint8_t mask);
servo_result_t SERVO_Hold(uint8_t mask);
servo_result_t SERVO_Sweep(uint8_t mask, uint16_t min_us, uint16_t max_us, uint32_t ms);
servo_result_t SERVO_StartSweep(uint16_t min_us, uint16_t max_us, uint32_t ms);
servo_result_t SERVO_StopSweep(void);
servo_result_t SERVO_SetSideRight(uint8_t right);
uint8_t SERVO_GetSideRight(void);
uint16_t SERVO_GetPulseUs(void);
uint16_t SERVO_GetChannelPulseUs(uint8_t channel);
const char *SERVO_ChannelName(uint8_t channel);
const char *SERVO_HeaderName(uint8_t side, uint8_t channel);
const char *SERVO_ResultName(servo_result_t result);
void SERVO_GetLimits(uint8_t side, uint8_t channel, uint16_t *min_us,
                     uint16_t *center_us, uint16_t *max_us);
uint8_t SERVO_IsSweeping(void);
void SERVO_GetStatus(servo_status_t *out);
#endif
