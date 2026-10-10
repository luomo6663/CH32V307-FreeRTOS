/********************************** (C) COPYRIGHT *******************************
* File Name          : robot_app.c
* Description        : Application bootstrap, RTOS tasks and runtime hooks.
*******************************************************************************/

#include "robot_app.h"
#include "console.h"
#include "telemetry.h"
#include "log.h"
#include "control/chassis.h"
#include "control/encoder.h"
#include "control/motor.h"
#include "control/servo.h"
#include "control/stepper.h"
#include "config/robot_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include <stdio.h>

#define CONTROL_TASK_PRIORITY       5U
#define CONSOLE_TASK_PRIORITY       3U
#define TELEMETRY_TASK_PRIORITY     2U

#define CONTROL_TASK_STACK_WORDS    448U
#define CONSOLE_TASK_STACK_WORDS    448U
#define TELEMETRY_TASK_STACK_WORDS  512U

static TaskHandle_t s_control_task;
static TaskHandle_t s_console_task;
static TaskHandle_t s_telemetry_task;

static void Robot_ControlTask(void *argument)
{
    TickType_t last_wake;

    (void)argument;

    last_wake = xTaskGetTickCount();

    for(;;)
    {
        CHASSIS_Update();
        SERVO_Update();
        STEPPER_Update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_CONTROL_PERIOD_MS));
    }
}

static void Robot_PrintBanner(void)
{
    LOGF("\r\n================================================\r\n");
    LOGF(" CH32V307 FreeRTOS mecanum robot + turntable v%s\r\n",
         ROBOT_FIRMWARE_VERSION);
    LOGF(" core %lu Hz | FreeRTOS %s\r\n",
         (unsigned long)SystemCoreClock, tskKERNEL_VERSION_NUMBER);
    LOGF(" chip %08lx | pwm %lu Hz | control %u Hz\r\n",
         (unsigned long)DBGMCU_GetCHIPID(),
         (unsigned long)MOTOR_PWM_FREQUENCY_HZ,
         (unsigned int)ROBOT_CONTROL_FREQUENCY_HZ);

    LOGF(" motors  W1 PA2/PA3  W2 PA15/PB3  W3 PD12/PD13  W4 PD14/PD15\r\n");
    LOGF(" encoder W1 PC6/PC7  W2 PD1/PD3  W3 PA8/PA9  W4 PD9/PD11\r\n");
    LOGF(" servo   PB5 TIM3_CH2, %lu Hz, %u-%u us sweep\r\n",
         (unsigned long)SERVO_PWM_FREQUENCY_HZ,
         (unsigned int)SERVO_SWEEP_MIN_US,
         (unsigned int)SERVO_SWEEP_MAX_US);
    LOGF(" turntable PA0=STEP TIM5_CH1, PA1=DIR, %lu pulses/rev\r\n",
         (unsigned long)ROBOT_STEPPER_STEPS_PER_REV);

    if(CHASSIS_IsReady() == 0U)
    {
        LOGF(" note: not every wheel has speed feedback yet, DRIVE is locked\r\n");
        LOGF("       use W <n> <duty> to move one wheel, or OL <n> <gain>\r\n");
    }

    LOGF(" console %lu baud, send H for help\r\n",
         (unsigned long)ROBOT_CONSOLE_BAUDRATE);
    LOGF(" motors are stopped until a command arrives\r\n");
    LOGF("================================================\r\n");
}

void ROBOT_APP_Init(void)
{
    TELEMETRY_Init();
    CHASSIS_Init();
    SERVO_Init();
    STEPPER_Init();
    Robot_PrintBanner();
}

void ROBOT_APP_Start(void)
{
    (void)xTaskCreate((TaskFunction_t)Robot_ControlTask,
                      "control",
                      CONTROL_TASK_STACK_WORDS,
                      NULL,
                      CONTROL_TASK_PRIORITY,
                      &s_control_task);

    (void)xTaskCreate((TaskFunction_t)CONSOLE_Task,
                      "console",
                      CONSOLE_TASK_STACK_WORDS,
                      NULL,
                      CONSOLE_TASK_PRIORITY,
                      &s_console_task);

    (void)xTaskCreate((TaskFunction_t)TELEMETRY_Task,
                      "telemetry",
                      TELEMETRY_TASK_STACK_WORDS,
                      NULL,
                      TELEMETRY_TASK_PRIORITY,
                      &s_telemetry_task);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;

    taskDISABLE_INTERRUPTS();
    printf("stack overflow in task %s\r\n", (task_name != NULL) ? task_name : "?");

    for(;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    printf("FreeRTOS heap exhausted\r\n");

    for(;;)
    {
    }
}
