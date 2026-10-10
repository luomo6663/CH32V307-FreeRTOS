/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Description        : Boot sequence: clocks, console, hardware, RTOS tasks.
*******************************************************************************/

#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app/log.h"
#include "app/console.h"
#include "app/robot_app.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();

    /* Console first so every later init step can be reported over USART3. */
    LOG_Init();
    CONSOLE_Init();

    ROBOT_APP_Init();
    ROBOT_APP_Start();

    vTaskStartScheduler();

    /* Only reached when the heap cannot hold the idle task. */
    for(;;)
    {
    }
}
