#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "wheel_id.h"
#include "config/robot_config.h"

/*
 * Encoder input layer.
 *
 *   W1 right rear  PC6  / PC7  -> TIM8_CH1  / TIM8_CH2,  quadrature
 *   W2 right front PD1  / PD3  -> TIM10_CH1 / TIM10_CH2 (full remap), quadrature
 *   W3 left front  PA8  / PA9  -> TIM1_CH1  / TIM1_CH2,  quadrature
 *   W4 left rear   PD9  / PD11 -> TIM9_CH1  / TIM9_CH2  (full remap), quadrature
 *
 * PD1/PD3 reach TIM10, PD9/PD11 reach TIM9, and both need their full remap
 * enabled in ENCODER_Init(). Every wheel is a true quadrature input, so the
 * sign comes from the timer and no direction hint is needed.
 */

#define ENCODER_COUNT               WHEEL_COUNT
#define ENCODER_SAMPLE_PERIOD_MS    ROBOT_CONTROL_PERIOD_MS

typedef struct
{
    int32_t  count;         /* signed position count                       */
    int32_t  delta;         /* signed counts during the last update        */
    int16_t  rpm;           /* output shaft speed                          */
    uint16_t raw;           /* raw timer counter, quadrature wheels only   */
    uint8_t  level_a;       /* live pin level                              */
    uint8_t  level_b;
    uint8_t  toggled_a;     /* latched "pin moved" flag, bring-up only     */
    uint8_t  toggled_b;
    uint32_t transitions_a; /* sampled level changes, bring-up only        */
    uint32_t transitions_b;
} encoder_status_t;

void     ENCODER_Init(void);
void     ENCODER_Update(void);

void     ENCODER_SetSign(uint8_t wheel, int8_t sign);
int8_t   ENCODER_GetSign(uint8_t wheel);

uint8_t  ENCODER_HasQuadrature(uint8_t wheel);
uint32_t ENCODER_GetCountsPerRev(uint8_t wheel);

int32_t  ENCODER_GetCount(uint8_t wheel);
int32_t  ENCODER_GetDelta(uint8_t wheel);
int16_t  ENCODER_GetRpm(uint8_t wheel);
int16_t  ENCODER_CountsToRpm(uint8_t wheel, int32_t delta_count);

uint16_t ENCODER_GetRawCount(uint8_t wheel);
uint8_t  ENCODER_GetInputLevel(uint8_t wheel, uint8_t channel);
uint8_t  ENCODER_GetToggleFlag(uint8_t wheel, uint8_t channel);
uint32_t ENCODER_GetTransitionCount(uint8_t wheel, uint8_t channel);

void     ENCODER_GetStatus(uint8_t wheel, encoder_status_t *out);

void     ENCODER_Reset(uint8_t wheel);
void     ENCODER_ResetCounters(void);
void     ENCODER_ResetDiagnostics(void);

#endif /* __ENCODER_H */
