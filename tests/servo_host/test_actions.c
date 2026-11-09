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
static void run(unsigned ms) { unsigned i; for(i=0;i<ms;i+=10) { ticks+=5;SERVO_Update(); } }
static servo_status_t status(void) { servo_status_t s;SERVO_GetStatus(&s);return s; }
static void pose(unsigned a,unsigned b,unsigned c) {
    servo_status_t s=status();
    assert(s.pulse_us[0]==a && s.pulse_us[1]==b && s.pulse_us[2]==c);
    assert(test_tim.CH1CVR==a && test_tim.CH2CVR==b && test_tim.CH3CVR==c);
}
static void action(servo_action_t a) { assert(SERVO_Action(a,1000)==SERVO_OK);run(1010); }
int main(void) {
    unsigned i;uint16_t us[3]={750,850,850};servo_status_t s;
    SERVO_Init();pose(1500,1500,1500);
    assert(status().side_right && (test_gpiod.OUTDR&4));
    assert(SERVO_SetChannelPulseUs(0,749)==SERVO_ERR_RANGE);
    assert(SERVO_SetChannelPulseUs(1,1501)==SERVO_ERR_RANGE);
    assert(SERVO_SetChannelPulseUs(2,849)==SERVO_ERR_RANGE);
    assert(SERVO_Move(7,us,1000)==SERVO_OK);run(10);run(100);
    s=status();assert(s.pulse_us[0]>1400); /* Ease-in, not an abrupt linear start. */
    run(900);pose(750,850,850);
    action(SERVO_ACTION_OPEN);pose(750,850,1500);
    action(SERVO_ACTION_EXTEND);pose(875,1500,1500);
    action(SERVO_ACTION_CLAMP);pose(875,1500,1000);
    action(SERVO_ACTION_GRIP);pose(875,1500,850);
    action(SERVO_ACTION_RETRACT);pose(1500,1500,850);
    action(SERVO_ACTION_RESET);pose(1500,1500,1500);
    assert(SERVO_Sweep(7,500,2500,2000)==SERVO_ERR_UNAVAILABLE);
    assert(SERVO_Action(SERVO_ACTION_PICK,1000)==SERVO_OK);run(10);
    /* With the claw already open, sequence advances without unnecessary motion. */
    run(10);assert(status().pick_step==2);
    run(1000);pose(875,1500,1500);assert(status().pick_step==3);
    run(1000);pose(875,1500,850);assert(status().pick_step==3);
    run(200);assert(status().pick_step==4);
    run(1000);pose(1500,1500,850);assert(!status().action);
    assert(SERVO_Action(SERVO_ACTION_PICK,1000)==SERVO_OK);run(300);
    assert(SERVO_Hold(7)==SERVO_OK);run(10);s=status();run(5000);
    pose(s.pulse_us[0],s.pulse_us[1],s.pulse_us[2]);assert(!status().action);
    for(i=0;i<16;i++) assert(SERVO_Action(SERVO_ACTION_EXTEND,1000)==SERVO_OK);
    assert(SERVO_Off(7)==SERVO_OK);run(10);run(5000);
    assert(!test_tim.CH1CVR && !test_tim.CH2CVR && !test_tim.CH3CVR && !status().queued);
    SERVO_Init();assert(SERVO_Jog(0,-10,100)==SERVO_OK);run(110);pose(1490,1500,1500);
    assert(SERVO_Jog(0,11,100)==SERVO_ERR_RANGE);
    assert(SERVO_SetSideRight(0)==SERVO_OK);
    assert(SERVO_Action(SERVO_ACTION_EXTEND,1000)==SERVO_ERR_UNAVAILABLE);
    run(30);assert(!status().hardware_side_right && !(test_gpiod.OUTDR&4));
    assert(SERVO_SetChannelPulseUs(2,1500)==SERVO_OK);run(10);
    assert(SERVO_SetSideRight(1)==SERVO_OK);run(30);pose(1490,1500,1500);
    test_pclk=60000000;test_hclk=120000000;SERVO_Init();
    assert(status().prescaler==119);pose(1500,1500,1500);
    puts("servo actions: mapping, limits, smooth motion, pick, cancel and L/R passed");
    return 0;
}
