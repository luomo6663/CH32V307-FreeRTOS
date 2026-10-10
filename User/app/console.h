#ifndef __CONSOLE_H
#define __CONSOLE_H

/*
 * Interactive command console on USART3.
 *
 * CONSOLE_Task() drains complete lines from the COMM driver, decodes them and
 * talks to the chassis layer only. It never touches the motor or encoder
 * registers directly, so the 100 Hz control loop stays the single owner of the
 * hardware.
 */

void CONSOLE_Init(void);
void CONSOLE_Task(void *argument);

#endif /* __CONSOLE_H */
