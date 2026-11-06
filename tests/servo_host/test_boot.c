#include <assert.h>
#include <stdio.h>
#include "ch32v30x.h"
#include "task.h"
#define SERVO_HOST_TEST
#include "../../User/control/servo.c"
TIM_TypeDef test_tim;
GPIO_TypeDef test_gpiob, test_gpiod;
AFIO_TypeDef test_afio;
uint32_t test_pclk=48000000, test_hclk=96000000;
static TickType_t ticks;
TickType_t xTaskGetTickCount(void) { return ticks; }
static void advance(unsigned ms) {
    unsigned i;
    for(i=0;i<ms;i+=10) {
        ticks+=5; SERVO_Update();
        test_tim.CNT=3000; TIM3_IRQHandler();
    }
}
static void expect(unsigned pulse) {
    servo_status_t s; unsigned c;
    SERVO_GetStatus(&s);
    assert(s.config_valid && s.hardware_side_right && (test_gpiod.OUTDR & 4));
    for(c=0;c<3;c++) {
        assert(s.pulse_us[c]==pulse && s.saved_us[1][c]==1500);
        assert(s.motion[1][c]==SERVO_HOLD);
    }
    assert(test_tim.CH1CVR==pulse && test_tim.CH2CVR==pulse && test_tim.CH3CVR==pulse);
}
int main(void) {
    servo_status_t s;
    SERVO_Init(); advance(1500); expect(500);
    advance(3000); expect(2500);
    advance(1500); expect(1500);
    advance(6000); expect(1500);
    SERVO_GetStatus(&s); assert(s.motion[0][0]==SERVO_HOLD);
    SERVO_Init(); advance(500);
    assert(SERVO_Hold(7)==SERVO_OK); advance(10);
    SERVO_GetStatus(&s);
    advance(6000); expect(s.pulse_us[0]);
    SERVO_Init(); advance(500);
    assert(SERVO_Off(7)==SERVO_OK); advance(10); advance(6000);
    assert(!test_tim.CH1CVR && !test_tim.CH2CVR && !test_tim.CH3CVR);
    puts("servo boot: right-only endpoints, one-shot and cancellation passed");
    return 0;
}
