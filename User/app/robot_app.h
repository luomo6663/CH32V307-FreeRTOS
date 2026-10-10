#ifndef __ROBOT_APP_H
#define __ROBOT_APP_H

/*
 * Application bootstrap: hardware setup and RTOS task creation.
 *
 *   control task   100 Hz   CHASSIS_Update(), owns motor and encoder hardware
 *   console task   on RX    command line, latches chassis commands
 *   telemetry task idle     periodic CSV stream, off by default
 */

void ROBOT_APP_Init(void);
void ROBOT_APP_Start(void);

#endif /* __ROBOT_APP_H */
