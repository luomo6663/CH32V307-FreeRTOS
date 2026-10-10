#ifndef __COMM_H
#define __COMM_H

#include "ch32v30x.h"

/*
 * Line based console on USART3.
 *   PB10 = TX (also used by printf)
 *   PB11 = RX
 *
 * RX runs from the USART3 interrupt into a lock free FIFO so no byte is lost
 * while a task is busy. No FreeRTOS API is called from the ISR.
 */

#define COMM_LINE_MAX   80U

void     COMM_Init(uint32_t baudrate);

/* Non-blocking. Copies one complete line into out and returns 1, else 0. */
uint8_t  COMM_PollLine(char *out, uint16_t max_len);

/* Bytes dropped because the software FIFO was full. */
uint32_t COMM_GetOverflowCount(void);

void     COMM_ResetOverflowCount(void);

#endif /* __COMM_H */
