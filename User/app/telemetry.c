/********************************** (C) COPYRIGHT *******************************
* File Name          : telemetry.c
* Description        : Periodic CSV / VOFA+ FireWater telemetry.
*******************************************************************************/

#include "telemetry.h"
#include "log.h"
#include "control/chassis.h"
#include "config/robot_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

static uint8_t  s_enabled = 0U;
static uint32_t s_period_ms = ROBOT_TELEMETRY_PERIOD_MS;
static telemetry_format_t s_format = TELEMETRY_FORMAT_CSV;
static uint8_t  s_vofa_wheel = TELEMETRY_VOFA_ALL;

void TELEMETRY_Init(void)
{
    s_enabled = 0U;
    s_period_ms = ROBOT_TELEMETRY_PERIOD_MS;
    s_format = TELEMETRY_FORMAT_CSV;
    s_vofa_wheel = TELEMETRY_VOFA_ALL;
}

void TELEMETRY_SetEnabled(uint8_t enabled)
{
    s_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t TELEMETRY_GetEnabled(void)
{
    return s_enabled;
}

void TELEMETRY_SetPeriodMs(uint32_t period_ms)
{
    if(period_ms < 10U)
    {
        period_ms = 10U;
    }
    else if(period_ms > 5000U)
    {
        period_ms = 5000U;
    }

    s_period_ms = period_ms;
}

uint32_t TELEMETRY_GetPeriodMs(void)
{
    return s_period_ms;
}

void TELEMETRY_SetFormat(telemetry_format_t format)
{
    s_format = (format == TELEMETRY_FORMAT_VOFA)
                   ? TELEMETRY_FORMAT_VOFA
                   : TELEMETRY_FORMAT_CSV;
}

telemetry_format_t TELEMETRY_GetFormat(void)
{
    return s_format;
}

void TELEMETRY_SetVofaWheel(uint8_t wheel)
{
    if((wheel < WHEEL_COUNT) || (wheel == TELEMETRY_VOFA_ALL))
    {
        s_vofa_wheel = wheel;
    }
}

uint8_t TELEMETRY_GetVofaWheel(void)
{
    return s_vofa_wheel;
}

static const char *Telemetry_Fixed3(float value, char *buffer, size_t length)
{
    int32_t  scaled = (int32_t)((value * 1000.0f) +
                                ((value >= 0.0f) ? 0.5f : -0.5f));
    uint32_t magnitude = (uint32_t)((scaled < 0) ? -scaled : scaled);

    (void)snprintf(buffer, length, "%s%u.%03u",
                   (scaled < 0) ? "-" : "",
                   (unsigned int)(magnitude / 1000U),
                   (unsigned int)(magnitude % 1000U));

    return buffer;
}

static void Telemetry_EmitCsv(void)
{
    chassis_status_t status;

    CHASSIS_GetStatus(&status);

    LOGF("T,%lu,%u,"
         "%d,%d,%d,%d,"
         "%d,%d,%d,%d,"
         "%d,%d,%d,%d,"
         "%d,%d,%d,"
         "%u\r\n",
         (unsigned long)(xTaskGetTickCount() * (1000U / configTICK_RATE_HZ)),
         (unsigned int)status.mode,
         (int)status.rpm[0], (int)status.rpm[1],
         (int)status.rpm[2], (int)status.rpm[3],
         (int)status.target_rpm[0], (int)status.target_rpm[1],
         (int)status.target_rpm[2], (int)status.target_rpm[3],
         (int)status.duty[0], (int)status.duty[1],
         (int)status.duty[2], (int)status.duty[3],
         (int)status.pose.x_mm,
         (int)status.pose.y_mm,
         (int)(status.pose.yaw_rad * 1000.0f),
         (unsigned int)status.faults);
}

static void Telemetry_EmitVofa(void)
{
    chassis_status_t status;
    uint8_t          wheel = s_vofa_wheel;
    char             value[12][16];

    CHASSIS_GetStatus(&status);

    if(wheel < WHEEL_COUNT)
    {
        LOGF("tgt:%s,rpm:%s,p:%s,i:%s,d:%s,out:%s\r\n",
             Telemetry_Fixed3(status.target_rpm_f[wheel],
                              value[0], sizeof(value[0])),
             Telemetry_Fixed3((float)status.rpm[wheel],
                              value[1], sizeof(value[1])),
             Telemetry_Fixed3(status.pid_p[wheel],
                              value[2], sizeof(value[2])),
             Telemetry_Fixed3(status.pid_i[wheel],
                              value[3], sizeof(value[3])),
             Telemetry_Fixed3(status.pid_d[wheel],
                              value[4], sizeof(value[4])),
             Telemetry_Fixed3((float)status.duty[wheel],
                              value[5], sizeof(value[5])));
    }
    else
    {
        LOGF("t1:%s,r1:%s,o1:%s,t2:%s,r2:%s,o2:%s,"
             "t3:%s,r3:%s,o3:%s,t4:%s,r4:%s,o4:%s\r\n",
             Telemetry_Fixed3(status.target_rpm_f[0],
                              value[0], sizeof(value[0])),
             Telemetry_Fixed3((float)status.rpm[0],
                              value[1], sizeof(value[1])),
             Telemetry_Fixed3((float)status.duty[0],
                              value[2], sizeof(value[2])),
             Telemetry_Fixed3(status.target_rpm_f[1],
                              value[3], sizeof(value[3])),
             Telemetry_Fixed3((float)status.rpm[1],
                              value[4], sizeof(value[4])),
             Telemetry_Fixed3((float)status.duty[1],
                              value[5], sizeof(value[5])),
             Telemetry_Fixed3(status.target_rpm_f[2],
                              value[6], sizeof(value[6])),
             Telemetry_Fixed3((float)status.rpm[2],
                              value[7], sizeof(value[7])),
             Telemetry_Fixed3((float)status.duty[2],
                              value[8], sizeof(value[8])),
             Telemetry_Fixed3(status.target_rpm_f[3],
                              value[9], sizeof(value[9])),
             Telemetry_Fixed3((float)status.rpm[3],
                              value[10], sizeof(value[10])),
             Telemetry_Fixed3((float)status.duty[3],
                              value[11], sizeof(value[11])));
    }
}

static void Telemetry_Emit(void)
{
    if(s_format == TELEMETRY_FORMAT_VOFA)
    {
        Telemetry_EmitVofa();
    }
    else
    {
        Telemetry_EmitCsv();
    }
}

void TELEMETRY_Task(void *argument)
{
    TickType_t last_wake;
    uint32_t   last_period = 0U;

    (void)argument;

    last_wake = xTaskGetTickCount();

    for(;;)
    {
        uint32_t period = s_period_ms;

        if(period != last_period)
        {
            last_period = period;
            last_wake = xTaskGetTickCount();
        }

        if(s_enabled != 0U)
        {
            Telemetry_Emit();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period));
    }
}
