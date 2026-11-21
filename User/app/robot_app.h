#ifndef __ROBOT_APP_H
#define __ROBOT_APP_H

#include <stdint.h>
#include "control/servo.h"

/*
 * Application bootstrap: hardware setup and RTOS task creation.
 *
 *   control task   100 Hz   chassis, servo command/motion and stepper updates
 *   TIM3 CC4 ISR           commits servo frames and bank routing in low interval
 *   console task   on RX    command line, latches chassis commands
 *   telemetry task idle     periodic CSV stream, off by default
 */

void ROBOT_APP_Init(void);
void ROBOT_APP_Start(void);
uint8_t ROBOT_LINK_Active(void);
uint8_t ROBOT_LINK_Zero(void);
uint8_t ROBOT_LINK_Start(int32_t pick_deg, int32_t place_deg, float rpm);
void ROBOT_LINK_Stop(void);
void ROBOT_LINK_InvalidateZero(void);
void ROBOT_LINK_PrintStatus(void);
uint8_t ROBOT_APP_Run(uint8_t home_only);
uint8_t ROBOT_APP_Arm(servo_action_t action);
uint8_t ROBOT_APP_ArmPose(uint16_t a_us, uint16_t b_us);
uint8_t ROBOT_APP_ArmLevel(uint16_t a_us);

#endif /* __ROBOT_APP_H */
