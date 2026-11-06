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
static void commit(unsigned count) { test_tim.CNT=count;TIM3_IRQHandler(); }
static servo_status_t status(void) { servo_status_t s;SERVO_GetStatus(&s);return s; }
static void init(void) { ticks=0;SERVO_Init(); }
int main(void)
{
    uint16_t pose[3]={1700,1300,1600}; unsigned i;
    servo_status_t s;
    init();s=status();assert(s.config_valid && s.side_right && s.hardware_side_right);
    assert(s.prescaler==95 && s.period==1000000U/ROBOT_SERVO_PWM_FREQUENCY_HZ-1U && (test_gpiod.OUTDR & 4));
    assert(s.pulse_us[0]==1500 && test_tim.CH1CVR==0);
    commit(3000);assert(test_tim.CH1CVR==1500);
    assert(SERVO_SetAllPulseUs(499)==SERVO_ERR_RANGE);
    assert(SERVO_SetAllPulseUs(2501)==SERVO_ERR_RANGE);
    assert(SERVO_Move(0,pose,100)==SERVO_ERR_RANGE);
    assert(SERVO_Move(7,pose,600001)==SERVO_ERR_RANGE);
    assert(SERVO_Move(7,pose,100)==SERVO_OK);step(10);
    step(50);s=status();assert(s.pulse_us[0]==1600 && s.pulse_us[1]==1400 && s.pulse_us[2]==1550);
    step(50);s=status();assert(s.pulse_us[0]==1700 && s.motion[0][0]==SERVO_HOLD);
    assert(SERVO_Sweep(1,1600,1400,100)==SERVO_ERR_RANGE);
    assert(SERVO_Sweep(1,1400,1600,19)==SERVO_ERR_RANGE);
    assert(SERVO_Sweep(1,1400,1600,101)==SERVO_OK);step(10);step(50);
    assert(status().pulse_us[0]==1600 && status().pulse_us[1]==1300);
    assert(SERVO_Hold(1)==SERVO_OK);step(10);s=status();step(100);assert(status().pulse_us[0]==s.pulse_us[0]);
    assert(SERVO_Off(1)==SERVO_OK);step(10);commit(3000);assert(test_tim.CH1CVR==0 && test_tim.CH2CVR==1300);
    assert(SERVO_Center(1)==SERVO_OK);step(10);commit(3000);assert(test_tim.CH1CVR==1500);
    /* FIFO applies a following move to the new bank only after low-window routing. */
    assert(SERVO_SetSideRight(0)==SERVO_OK);assert(SERVO_SetChannelPulseUs(1,1800)==SERVO_OK);
    step(10);assert(!status().side_right && status().hardware_side_right);
    commit(1500);assert(status().hardware_side_right);step(10);assert(status().queued==1);
    commit(FRAME_US-50);assert(status().hardware_side_right);
    commit(3000);assert(!status().hardware_side_right && !(test_gpiod.OUTDR & 4));
    step(10);assert(status().pulse_us[1]==1800);
    assert(SERVO_SetSideRight(1)==SERVO_OK);step(10);commit(3000);step(10);
    assert(status().pulse_us[1]==1300 && !status().sweeping);
    /* Leaving an actively scanning bank freezes it; OFF persists across banks. */
    init();assert(SERVO_Sweep(1,1400,1600,1000)==SERVO_OK);step(10);step(100);
    assert(SERVO_Off(2)==SERVO_OK);step(10);
    assert(SERVO_SetSideRight(0)==SERVO_OK);step(10);s=status();
    assert(s.motion[0][0]==SERVO_HOLD && !s.enabled[0][1]);
    commit(3000);step(100);
    assert(SERVO_SetSideRight(1)==SERVO_OK);step(10);commit(3000);step(10);
    assert(status().pulse_us[0]==s.saved_us[0][0] && !status().enabled[0][1]);
    init();for(i=0;i<16;i++) assert(SERVO_Center(7)==SERVO_OK);
    assert(SERVO_SetSideRight(0)==SERVO_ERR_QUEUE_FULL && status().requested_side_right);
    for(i=0;i<16;i++) step(10);
    assert(status().queued==0);
    /* Tick wrap and very long stalls remain bounded. */
    ticks=0xfffffff0U;SERVO_Init();assert(SERVO_Move(7,pose,100)==SERVO_OK);step(10);step(100);
    assert(status().pulse_us[0]==1700);
    init();assert(SERVO_Sweep(1,1400,1600,101)==SERVO_OK);step(10);step(1000000);
    assert(status().pulse_us[0]>=1400 && status().pulse_us[0]<=1600);
    init();assert(SERVO_MoveAngle(2,-135,0)==SERVO_OK);step(10);commit(3000);
    assert(test_tim.CH2CVR==500);
    assert(SERVO_MoveAngle(2,135,0)==SERVO_OK);step(10);commit(3000);
    assert(test_tim.CH2CVR==2500);
    assert(SERVO_MoveAngle(2,0,0)==SERVO_OK);step(10);commit(3000);
    assert(test_tim.CH2CVR==1500);
    assert(SERVO_MoveAngle(2,136,0)==SERVO_ERR_RANGE);
    test_pclk=123;SERVO_Init();assert(!status().config_valid && test_tim.CH1CVR==0);
    assert(SERVO_Center(7)==SERVO_ERR_CONFIG);
    puts("servo host: all checks passed");return 0;
}
