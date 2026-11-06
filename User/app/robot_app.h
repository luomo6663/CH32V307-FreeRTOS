#ifndef __ROBOT_APP_H
#define __ROBOT_APP_H

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

#endif /* __ROBOT_APP_H */
