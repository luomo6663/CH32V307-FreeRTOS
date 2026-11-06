#ifndef __CONSOLE_H
#define __CONSOLE_H

/*
 * Interactive command console on USART3.
 *
 * CONSOLE_Task() drains complete lines from the COMM driver, decodes them and
 * submits chassis and servo commands and calls stepper maintenance APIs.
 * Servo commands are queued; only the control task advances their motion and
 * the TIM3 ISR commits PWM frames. No console code writes servo registers.
 */

void CONSOLE_Init(void);
void CONSOLE_Task(void *argument);

#endif /* __CONSOLE_H */
