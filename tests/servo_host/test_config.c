#include <assert.h>
#include <stdio.h>
#include "ch32v30x.h"
#include "task.h"
#define SERVO_HOST_TEST
#include "../../User/control/servo.c"
TIM_TypeDef test_tim;
GPIO_TypeDef test_gpiob,test_gpiod;
AFIO_TypeDef test_afio;
uint32_t test_pclk=48000000,test_hclk=96000000;
static TickType_t ticks;
TickType_t xTaskGetTickCount(void) { return ticks; }
int main(void)
{
    servo_status_t s;
    SERVO_Init();SERVO_GetStatus(&s);
#ifdef TEST_SERVO_BAD_CONFIG
    assert(!s.config_valid && !test_tim.CH1CVR && !test_tim.CH2CVR);
    assert(SERVO_SetAllPulseUs(1500)==SERVO_ERR_CONFIG);
#else
    assert(s.config_valid);
#ifdef TEST_SERVO_MISSING
    assert(SERVO_SetSideRight(0)==SERVO_OK);
    /* Validation follows the queued SIDE before hardware has switched. */
    assert(SERVO_SetChannelPulseUs(2,1500)==SERVO_ERR_UNAVAILABLE);
    assert(SERVO_SetAllPulseUs(1600)==SERVO_OK);
    ticks+=5;SERVO_Update();test_tim.CNT=3000;TIM3_IRQHandler();
    ticks+=5;SERVO_Update();TIM3_IRQHandler();SERVO_GetStatus(&s);
    assert(s.saved_us[1][0]==1600 && s.saved_us[1][1]==1600 && !s.enabled[1][2]);
    assert(test_tim.CH3CVR==0);
#endif
#ifdef TEST_SERVO_CALIBRATED
    assert(s.pulse_us[0]==1450 && s.pulse_us[1]==1550);
    assert(SERVO_SetChannelPulseUs(0,1100)==SERVO_ERR_RANGE);
    assert(SERVO_SetAllPulseUs(1850)==SERVO_ERR_RANGE);
    assert(SERVO_Sweep(2,1200,1600,1000)==SERVO_ERR_RANGE);
    assert(SERVO_SetAllPulseUs(1600)==SERVO_OK);ticks+=5;SERVO_Update();
    assert(SERVO_Center(7)==SERVO_OK);ticks+=5;SERVO_Update();SERVO_GetStatus(&s);
    assert(s.pulse_us[0]==1450 && s.pulse_us[1]==1550);
#endif
#ifdef TEST_SERVO_INVERT
    assert(!(test_gpiod.OUTDR & 4));
    assert(SERVO_SetSideRight(0)==SERVO_OK);ticks+=5;SERVO_Update();
    test_tim.CNT=3000;TIM3_IRQHandler();assert(test_gpiod.OUTDR & 4);
#endif
#endif
    puts("servo configuration: all checks passed");return 0;
}
