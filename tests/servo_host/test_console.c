/* Compile the actual console with section GC; only the servo command path is retained. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static char output[8192];
static size_t used;
static int test_printf(const char *fmt, ...)
{
    int n; va_list args; va_start(args,fmt);
    n=vsnprintf(output+used,sizeof(output)-used,fmt,args);va_end(args);
    assert(n>=0 && (size_t)n<sizeof(output)-used);used+=(size_t)n;return n;
}
#define printf test_printf
/* MinGW retains externally visible functions even with --gc-sections. */
#define CONSOLE_Init static unused_console_init
#define CONSOLE_Task static unused_console_task
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-declaration"
#pragma GCC diagnostic ignored "-Wunused-function"
#include "../../User/app/console.c"
#pragma GCC diagnostic pop
#undef CONSOLE_Init
#undef CONSOLE_Task
#undef printf
#define SERVO_HOST_TEST
#include "../../User/control/servo.c"
TIM_TypeDef test_tim;
GPIO_TypeDef test_gpiob,test_gpiod;
AFIO_TypeDef test_afio;
uint32_t test_pclk=48000000,test_hclk=96000000,SystemCoreClock=96000000;
static TickType_t ticks;
TickType_t xTaskGetTickCount(void) { return ticks; }
void LOG_Lock(void) {}
void LOG_Unlock(void) {}
static void command(const char *args, const char *expected)
{
    char line[256];snprintf(line,sizeof(line),"SERVO %s",args);
    used=0;output[0]=0;strtok(line," ");Console_HandleServo();
    if(!strstr(output,expected)) { fprintf(stderr,"%s -> %s",args,output);assert(0); }
}
int main(void)
{
    unsigned i;uint32_t n;
    const char *bad[]={"A 1500abc","A -1500","A +1500","A 99999999999999999999999",
        "A 1500 extra","SIDE R extra","MOVE A 1500 -1","MOVE A 1500 600001",
        "POSE 1500 1500 1500","OFF Z","CENTER ALL extra","SWEEP A 1400 1600 100 x",
        "MAP extra","HOLD ALL x y z t u v", "ANGLE B -136 500",
        "ANGLE B 20x 500", "ANGLE B - 500", "ANGLE B 20 -1"};
    SERVO_Init();
    assert(Console_ServoNumber("600000",600000,&n) && n==600000);
    assert(!Console_ServoNumber("",2500,&n));
    for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) command(bad[i],"err:");
    assert(s_count==0);
    command("MAP","UNVERIFIED");command("HELP","bare us=channel B");
    command("SIDE R","ok servo queued");command("CENTER ALL","ok servo queued");
    command("A 1600","ok servo queued");command("1500","ok servo queued");
    command("MOVE B 1700 1000","ok servo queued");
    command("ANGLE B -20 500","ok servo queued");
    command("ANGLE B +20 500","ok servo queued");
    command("POSE 1500 1600 1400 1000","ok servo queued");
    command("SWEEP A 1400 1600 2000","ok servo queued");
    command("SWEEP 2000","ok servo queued");command("SWEEP","ok servo queued");
    command("SWEEP A 1600 1400 2000","err:");command("SWEEP 0","err:");
    command("HOLD","ok servo queued");command("OFF C","ok servo queued");
    while(s_count) { ticks+=5;SERVO_Update();test_tim.CNT=3000;TIM3_IRQHandler(); }
    puts("servo console: all checks passed");return 0;
}
