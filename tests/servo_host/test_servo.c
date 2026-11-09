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
static void step(unsigned ms) { ticks+=ms/2;SERVO_Update(); }
static servo_status_t status(void) { servo_status_t s;SERVO_GetStatus(&s);return s; }
int main(void) {
    uint16_t pose[3]={1000,1100,1200};unsigned i;
    SERVO_Init();assert(status().config_valid && status().prescaler==95 && status().period==3332);
    assert(test_tim.CH1CVR==1500 && test_tim.CH2CVR==1500 && test_tim.CH3CVR==1500);
    assert(SERVO_Move(0,pose,100)==SERVO_ERR_RANGE);
    assert(SERVO_Move(7,pose,600001)==SERVO_ERR_RANGE);
    assert(SERVO_Action(SERVO_ACTION_NONE,1000)==SERVO_ERR_RANGE);
    assert(SERVO_Action(SERVO_ACTION_EXTEND,0)==SERVO_ERR_RANGE);
    for(i=0;i<16;i++) assert(SERVO_Center(7)==SERVO_OK);
    assert(SERVO_SetSideRight(0)==SERVO_ERR_QUEUE_FULL);
    assert(status().requested_side_right);
    for(i=0;i<16;i++) step(10);
    assert(!status().queued);
    ticks=0xfffffff0U;SERVO_Init();
    assert(SERVO_Move(7,pose,100)==SERVO_OK);step(10);step(100);
    assert(status().pulse_us[0]==1000 && status().pulse_us[1]==1100 && status().pulse_us[2]==1200);
    SERVO_Init();assert(SERVO_Move(7,pose,600000)==SERVO_OK);step(10);step(1000000);
    assert(status().pulse_us[0]==1000 && status().motion[0][0]==SERVO_HOLD);
    test_pclk=96000000;test_hclk=96000000;SERVO_Init();assert(status().timer_hz==96000000 && status().prescaler==95);
    test_pclk=60000000;test_hclk=120000000;SERVO_Init();assert(status().timer_hz==120000000 && status().prescaler==119);
    test_pclk=123;SERVO_Init();assert(!status().config_valid && !test_tim.CH1CVR);
    assert(SERVO_Center(7)==SERVO_ERR_CONFIG);
    puts("servo driver: timing, queue, tick wrap and invalid configuration passed");return 0;
}
