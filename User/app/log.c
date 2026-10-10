/********************************** (C) COPYRIGHT *******************************
* File Name          : log.c
* Description        : Mutex protected printf for multi task logging.
*******************************************************************************/

#include "log.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stddef.h>

static SemaphoreHandle_t s_log_mutex = NULL;

void LOG_Init(void)
{
    if(s_log_mutex == NULL)
    {
        s_log_mutex = xSemaphoreCreateMutex();
    }
}

void LOG_Lock(void)
{
    if((s_log_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        (void)xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    }
}

void LOG_Unlock(void)
{
    if((s_log_mutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        (void)xSemaphoreGive(s_log_mutex);
    }
}
