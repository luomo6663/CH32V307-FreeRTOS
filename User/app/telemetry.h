#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include <stdint.h>

/*
 * Periodic telemetry on the console UART.
 *
 * CSV mode keeps the original tabular stream. VOFA mode emits FireWater
 * frames ("name:value,...") that VOFA+ can plot directly. Default is off so
 * the console stays readable.
 *
 * VOFA single-wheel channels: tgt, rpm, p, i, d, out.
 * VOFA all-wheel channels: t1,r1,o1 ... t4,r4,o4.
 */

#define TELEMETRY_VOFA_ALL          0xFFU

typedef enum
{
    TELEMETRY_FORMAT_CSV = 0,
    TELEMETRY_FORMAT_VOFA
} telemetry_format_t;

void     TELEMETRY_Init(void);
void     TELEMETRY_Task(void *argument);

void     TELEMETRY_SetEnabled(uint8_t enabled);
uint8_t  TELEMETRY_GetEnabled(void);

void     TELEMETRY_SetPeriodMs(uint32_t period_ms);
uint32_t TELEMETRY_GetPeriodMs(void);

void     TELEMETRY_SetFormat(telemetry_format_t format);
telemetry_format_t TELEMETRY_GetFormat(void);

/* Select 0..3 for one-wheel PID plots, or TELEMETRY_VOFA_ALL for four wheels. */
void     TELEMETRY_SetVofaWheel(uint8_t wheel);
uint8_t  TELEMETRY_GetVofaWheel(void);

#endif /* __TELEMETRY_H */
