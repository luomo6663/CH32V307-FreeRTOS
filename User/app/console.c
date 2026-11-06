/********************************** (C) COPYRIGHT *******************************
* File Name          : console.c
* Description        : Command line interface over USART3.
*******************************************************************************/

#include "console.h"
#include "comm.h"
#include "log.h"
#include "telemetry.h"
#include "control/chassis.h"
#include "control/encoder.h"
#include "control/mecanum.h"
#include "control/motor.h"
#include "control/servo.h"
#include "control/stepper.h"
#include "control/wheel_control.h"
#include "config/robot_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *const s_wheel_name[WHEEL_COUNT] =
{
    "W1-RR", "W2-RF", "W3-LF", "W4-LR"
};

/* ------------------------------------------------------------------ */
/* token helpers                                                       */
/* ------------------------------------------------------------------ */

static char *Console_NextToken(void)
{
    return strtok(NULL, " \t");
}

static uint8_t Console_ParseInt(const char *text, int32_t *out)
{
    char *end;
    long  value;

    if((text == NULL) || (out == NULL))
    {
        return 0U;
    }

    value = strtol(text, &end, 10);
    if(end == text)
    {
        return 0U;
    }

    *out = (int32_t)value;
    return 1U;
}

static uint8_t Console_NextInt(int32_t *out)
{
    return Console_ParseInt(Console_NextToken(), out);
}

/*
 * Small float parser. newlib-nano cannot print %f without extra work and needs
 * an extra link symbol for %f scanf, so the console avoids both.
 */
static uint8_t Console_ParseFloat(const char *text, float *out)
{
    const char *p;
    float       value = 0.0f;
    float       scale = 0.1f;
    int32_t     exponent = 0;
    uint8_t     negative = 0U;
    uint8_t     digits = 0U;

    if((text == NULL) || (out == NULL))
    {
        return 0U;
    }

    p = text;

    if((*p == '+') || (*p == '-'))
    {
        negative = (*p == '-') ? 1U : 0U;
        p++;
    }

    while((*p >= '0') && (*p <= '9'))
    {
        value = (value * 10.0f) + (float)(*p - '0');
        p++;
        digits = 1U;
    }

    if(*p == '.')
    {
        p++;
        while((*p >= '0') && (*p <= '9'))
        {
            value += (float)(*p - '0') * scale;
            scale *= 0.1f;
            p++;
            digits = 1U;
        }
    }

    if(digits == 0U)
    {
        return 0U;
    }

    if((*p == 'e') || (*p == 'E'))
    {
        uint8_t exp_negative = 0U;

        p++;
        if((*p == '+') || (*p == '-'))
        {
            exp_negative = (*p == '-') ? 1U : 0U;
            p++;
        }

        if((*p < '0') || (*p > '9'))
        {
            return 0U;
        }

        while((*p >= '0') && (*p <= '9'))
        {
            exponent = (exponent * 10) + (int32_t)(*p - '0');
            if(exponent > 30)
            {
                exponent = 30;
            }
            p++;
        }

        while(exponent-- > 0)
        {
            value = (exp_negative != 0U) ? (value * 0.1f) : (value * 10.0f);
        }
    }

    if(*p != '\0')
    {
        return 0U;
    }

    *out = (negative != 0U) ? -value : value;
    return 1U;
}

static uint8_t Console_NextFloat(float *out)
{
    return Console_ParseFloat(Console_NextToken(), out);
}

static const char *Console_Fixed3(float value, char *buffer, size_t length)
{
    int32_t scaled = (int32_t)((value * 1000.0f) + ((value >= 0.0f) ? 0.5f : -0.5f));
    uint32_t magnitude = (uint32_t)((scaled < 0) ? -scaled : scaled);

    (void)snprintf(buffer, length, "%s%u.%03u",
                   (scaled < 0) ? "-" : "",
                   (unsigned int)(magnitude / 1000U),
                   (unsigned int)(magnitude % 1000U));

    return buffer;
}

/* ------------------------------------------------------------------ */
/* status printers                                                     */
/* ------------------------------------------------------------------ */

static void Console_PrintHelp(void)
{
    LOGF("\r\n=== CH32V307 mecanum robot console ===\r\n");
    LOGF("V <vx> <vy> <wz>      drive twist: mm/s, mm/s, rad/s\r\n");
    LOGF("S                     stop, back to IDLE\r\n");
    LOGF("X                     immediate coast, all wheels\r\n");
    LOGF("B                     brake, all wheels\r\n");
    LOGF("W <n> <duty>          open loop one wheel, n=1..4\r\n");
    LOGF("WTEST <n> [duty] [ms] both directions, report encoder delta\r\n");
    LOGF("M <d1> <d2> <d3> <d4> open loop four wheels\r\n");
    LOGF("RAMP <n>              slew limit, duty per period, 0 = off\r\n");
    LOGF("INV <n> <0|1>         motor direction invert\r\n");
    LOGF("MODE <0|1>            0 = dual PWM, 1 = PWM + DIR\r\n");
    LOGF("E [R]                 encoder table, R resets counters\r\n");
    LOGF("SIGN <n> <1|-1>       encoder sign calibration\r\n");
    LOGF("SPD <n> <rpm>         closed-loop speed test, one wheel\r\n");
    LOGF("SPD ALL <rpm>         closed-loop speed test, all wheels\r\n");
    LOGF("PID <n|ALL> [kp ki kd] read or set speed loop gains\r\n");
    LOGF("OL <n> <gain>         open loop duty per rpm fallback\r\n");
    LOGF("CLR                   clear latched faults\r\n");
    LOGF("POSE [R]              print or reset odometry\r\n");
    LOGF("VOFA <n|ALL|OFF> [ms] FireWater PID plot, n=1..4\r\n");
    LOGF("TELEM <on|off|ms|csv> telemetry stream control\r\n");
    LOGF("SERVO [us]            servo status or set channel B pulse (500..2500 us)\r\n");
    LOGF("SERVO HELP            servo side, channel, move, sweep and hold commands\r\n");
    LOGF("TURN <deg> [rpm]      relative turntable move\r\n");
    LOGF("TURN RUN <CW|CCW> [rpm] continuous turntable rotation\r\n");
    LOGF("TURN STOP|ZERO|PPR [n] turntable stop/zero/pulses-per-rev\r\n");
    LOGF("TURN PULSES <n> [hz]  direct stepper pulse move\r\n");
    LOGF("PPRCAL <pulses> <deg> turntable ppr from a measured move\r\n");
    LOGF("STEST [pulses] [hz]   stepper forward + reverse bench test\r\n");
    LOGF("DIR <0|1>             force stepper DIR level (wiring check)\r\n");
    LOGF("STEP <0|1>            force stepper STEP level (wiring check)\r\n");
    LOGF("PINS                  motor, stepper and servo pin map\r\n");
    LOGF("P                     status\r\n");
    LOGF("CFG                   configuration summary\r\n");
    LOGF("H or ?                this help\r\n");
}

static void Console_PrintStatus(void)
{
    chassis_status_t status;
    char             buffer[4][16];
    uint8_t          wheel;

    CHASSIS_GetStatus(&status);

    LOGF("mode=%s faults=0x%04x driveable=%u telemetry=%u rx_drop=%lu\r\n",
         CHASSIS_ModeName((chassis_mode_t)status.mode),
         (unsigned int)status.faults,
         (unsigned int)((status.omni_ready >= 0.5f) ? 1U : 0U),
         (unsigned int)TELEMETRY_GetEnabled(),
         (unsigned long)COMM_GetOverflowCount());

    LOGF("cmd  vx=%d mm/s vy=%d mm/s wz=%d mrad/s\r\n",
         (int)status.vx_cmd, (int)status.vy_cmd, (int)(status.wz_cmd * 1000.0f));
    LOGF("meas vx=%d mm/s vy=%d mm/s wz=%d mrad/s\r\n",
         (int)status.vx_meas, (int)status.vy_meas, (int)(status.wz_meas * 1000.0f));
    LOGF("pose x=%d mm y=%d mm yaw=%d mrad\r\n",
         (int)status.pose.x_mm, (int)status.pose.y_mm,
         (int)(status.pose.yaw_rad * 1000.0f));

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        float kp;
        float ki;
        float kd;

        WHEEL_CONTROL_GetGains(wheel, &kp, &ki, &kd);

        LOGF("%s rpm=%5d target=%5d duty=%5d state=%-8s pid=%s/%s/%s ol=%s\r\n",
             s_wheel_name[wheel],
             (int)status.rpm[wheel],
             (int)status.target_rpm[wheel],
             (int)status.duty[wheel],
             CHASSIS_WheelStateName(status.wheel_state[wheel]),
             Console_Fixed3(kp, buffer[0], sizeof(buffer[0])),
             Console_Fixed3(ki, buffer[1], sizeof(buffer[1])),
             Console_Fixed3(kd, buffer[2], sizeof(buffer[2])),
             Console_Fixed3(WHEEL_CONTROL_GetOpenLoopGain(wheel),
                            buffer[3], sizeof(buffer[3])));
    }
}

static void Console_PrintEncoders(void)
{
    encoder_status_t status;
    uint8_t          wheel;

    LOGF("enc  mode   raw   count  delta    rpm  A B  moved  samples\r\n");

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        ENCODER_GetStatus(wheel, &status);
        LOGF("%s %s %5u %7ld %6ld %6d  %u %u  %u/%u  %lu/%lu\r\n",
             s_wheel_name[wheel],
             (ENCODER_HasQuadrature(wheel) != 0U) ? "quad " : "1chan",
             (unsigned int)status.raw,
             (long)status.count,
             (long)status.delta,
             (int)status.rpm,
             (unsigned int)status.level_a,
             (unsigned int)status.level_b,
             (unsigned int)status.toggled_a,
             (unsigned int)status.toggled_b,
             (unsigned long)status.transitions_a,
             (unsigned long)status.transitions_b);
    }

    LOGF("counts/rev:");
    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        LOGF(" %s=%lu", s_wheel_name[wheel],
             (unsigned long)ENCODER_GetCountsPerRev(wheel));
    }
    LOGF("\r\n");
}

static void Console_PrintConfig(void)
{
    RCC_ClocksTypeDef clocks;
    char              buffer[16];

    RCC_GetClocksFreq(&clocks);

    LOGF("core=%lu Hz pclk1=%lu Hz pclk2=%lu Hz\r\n",
         (unsigned long)clocks.HCLK_Frequency,
         (unsigned long)clocks.PCLK1_Frequency,
         (unsigned long)clocks.PCLK2_Frequency);
    LOGF("motor pwm=%lu Hz timer_clk=%lu Hz period=%u steps slew=%d\r\n",
         (unsigned long)MOTOR_PWM_FREQUENCY_HZ,
         (unsigned long)MOTOR_GetTimerClockHz(),
         (unsigned int)MOTOR_GetPwmPeriod(),
         (int)MOTOR_GetSlewLimit());
    LOGF("control=%u ms telemetry=%u ms command timeout=%u ms\r\n",
         (unsigned int)ROBOT_CONTROL_PERIOD_MS,
         (unsigned int)TELEMETRY_GetPeriodMs(),
         (unsigned int)ROBOT_COMMAND_TIMEOUT_MS);
    LOGF("chassis radius=%s mm half_l=%d mm half_w=%d mm\r\n",
         Console_Fixed3(ROBOT_WHEEL_RADIUS_MM, buffer, sizeof(buffer)),
         (int)ROBOT_HALF_LENGTH_MM,
         (int)ROBOT_HALF_WIDTH_MM);
    LOGF("limits wheel=%d rpm wheel=%d mm/s yaw=%d mrad/s\r\n",
         (int)ROBOT_MAX_WHEEL_RPM,
         (int)Mecanum_WheelRpmToMmS(ROBOT_MAX_WHEEL_RPM),
         (int)(ROBOT_MAX_YAW_RATE_RAD_S * 1000.0f));
    LOGF("drive mode=%s, all wheels driveable: %s\r\n",
         (MOTOR_GetDriveMode() == MOTOR_MODE_DUAL_PWM) ? "dual PWM" : "PWM + DIR",
         (CHASSIS_IsReady() != 0U) ? "yes" : "no");
}

/* "TIM2_CH3" or "GPIO", depending on whether IN2 kept a timer channel. */
static const char *Console_ChannelText(const motor_wiring_t *motor, char *buffer,
                                       size_t length, uint8_t in1)
{
    uint8_t channel = (in1 != 0U) ? motor->in1_channel : motor->in2_channel;

    if(channel == 0U)
    {
        (void)snprintf(buffer, length, "GPIO");
    }
    else
    {
        (void)snprintf(buffer, length, "%s_CH%u", motor->timer,
                       (unsigned int)channel);
    }

    return buffer;
}

/*
 * Prints the live pin map so a bench check can confirm that the firmware pins
 * match the power board (IN1/IN2 of MOTOR1..MOTOR4, STEP/DIR of the driver).
 */
static void Console_PrintPins(void)
{
    static const char *const board_channel[WHEEL_COUNT] =
    {
        "MOTOR3/ENC3", "MOTOR4/ENC4", "MOTOR1/ENC1", "MOTOR2/ENC2"
    };
    motor_wiring_t   motor;
    stepper_wiring_t stepper;
    char             in1[16];
    char             in2[16];
    uint8_t          wheel;

    LOGF("wheel      IN1                 IN2                 mode     board\r\n");

    for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
    {
        if(MOTOR_GetWiring(wheel, &motor) == 0U)
        {
            continue;
        }

        LOGF("%-10s %s%-3u %-11s %s%-3u %-11s %-8s %s\r\n",
             s_wheel_name[wheel],
             motor.in1_port, (unsigned int)motor.in1_pin,
             Console_ChannelText(&motor, in1, sizeof(in1), 1U),
             motor.in2_port, (unsigned int)motor.in2_pin,
             Console_ChannelText(&motor, in2, sizeof(in2), 0U),
             (motor.pwm_dir != 0U) ? "PWM+DIR" : "dual PWM",
             board_channel[wheel]);
    }

    STEPPER_GetWiring(&stepper);
    LOGF("stepper    %s%-3u %-11s DIR %s%u\r\n",
         stepper.step_port, (unsigned int)stepper.step_pin,
         stepper.step_timer, stepper.dir_port,
         (unsigned int)stepper.dir_pin);
    LOGF("servo      ARM_A/B/C = PB4/PB5/PB0 TIM3_CH1/2/3, PD2 = ARM_SWITCH\r\n");
}

/* Servo parsing is strict and independent of the legacy chassis parser. */
static uint8_t Console_ServoNumber(const char *s, uint32_t max, uint32_t *out)
{
    uint32_t value = 0;
    if(!s || !*s) return 0;
    while(*s) {
        uint32_t digit;
        if(*s < '0' || *s > '9') return 0;
        digit = (uint32_t)(*s++ - '0');
        if(digit > max || value > (max - digit) / 10U) return 0;
        value = value * 10U + digit;
    }
    *out = value;
    return 1;
}

static uint8_t Console_ServoMask(const char *s)
{
    if(!s || strcmp(s, "ALL") == 0) return SERVO_MASK_ALL;
    if(strcmp(s, "A") == 0) return 1;
    if(strcmp(s, "B") == 0) return 2;
    if(strcmp(s, "C") == 0) return 4;
    return 0;
}

static void Console_PrintServoHelp(void)
{
    LOGF("SERVO                 pulse, motion, side and timer status\r\n");
    LOGF("SERVO <500..2500>     set current side channel B pulse in us\r\n");
    LOGF("SERVO SIDE <R|L>      select right or left arm\r\n");
    LOGF("SERVO <A|B|C|ALL> <us>  set channel pulse\r\n");
    LOGF("SERVO MOVE <A|B|C|ALL> <us> <ms>  smooth move\r\n");
    LOGF("SERVO CENTER [A|B|C|ALL]  return to configured center\r\n");
    LOGF("SERVO SWEEP [ms]      scan current side ALL, 1000..2000 us; default %lu ms\r\n",
         (unsigned long)SERVO_SWEEP_PERIOD_MS);
    LOGF("SERVO HOLD            stop scanning/moving, hold current pulse\r\n");
    LOGF("SERVO OFF [A|B|C|ALL]  disable PWM output\r\n");
    LOGF("SERVO POSE <A_us> <B_us> <C_us> <ms>  move three channels\r\n");
    LOGF("SERVO SWEEP <A|B|C|ALL> <min_us> <max_us> <ms>  custom scan\r\n");
    LOGF("us=500..2500 within configured limits; ms=0..%lu (sweep >=20)\r\n",
         (unsigned long)ROBOT_SERVO_MAX_MOTION_MS);
    LOGF("ALL=current bank installed channels; bare us=channel B; OK=queued\r\n");
    LOGF("A=claw (PB4), B=middle (PB5), C=base (PB0); center=1500 us by default\r\n");
}

static void Console_PrintServo(void)
{
    servo_status_t s;
    uint8_t b, c;
    static const char *const motion[] = {"hold", "move", "sweep"};
    SERVO_GetStatus(&s);
    LOGF("servo config=%s map=%s pwm=%luHz queue=%u\r\n",
         s.config_valid ? "ok" : "INVALID", ROBOT_SERVO_MAP_VERIFIED ? "verified" : "UNVERIFIED",
         (unsigned long)(s.timer_hz / ((uint32_t)s.prescaler+1U) / ((uint32_t)s.period+1U)), s.queued);
    LOGF("side requested=%s control=%s routed=%s switch=%u invert=%u\r\n",
         s.requested_side_right ? "R" : "L", s.side_right ? "R" : "L",
         s.hardware_side_right ? "R" : "L", (unsigned)((s.gpiod_odr >> 2)&1U),
         (unsigned)ROBOT_SERVO_SIDE_INVERT);
    for(b = 0; b < 2; b++) {
        for(c = 0; c < 3; c++) {
            uint16_t low=0, center=0, high=0;
            SERVO_GetLimits(b,c,&low,&center,&high);
            LOGF(" %s %s/%s PB%u cfg=%s pwm=%s cmd=%u target=%u %s limits=%u/%u/%u us\r\n",
                 b ? "L" : "R", SERVO_ChannelName(c), SERVO_HeaderName(b,c),
                 (unsigned)((c==0)?4:((c==1)?5:0)),
                 (s.installed_mask[b] & (1U<<c)) ? "enabled" : "absent",
                 s.enabled[b][c] ? "on" : "off", s.saved_us[b][c], s.target_us[b][c],
                 motion[s.motion[b][c]], low,center,high);
        }
    }
    LOGF("cmd is software pulse, not position feedback; only routed bank receives PWM\r\n");
    LOGF("TIM3 PSC=%u ARR=%u CCR=%u/%u/%u CTLR1=%04x\r\n",
         s.prescaler,s.period,s.compare[0],s.compare[1],s.compare[2],s.timer_control);
}

static void Console_HandleServo(void)
{
    char *args[7], *token;
    uint8_t n=0, mask;
    uint32_t v[4]={0};
    uint16_t us[3];
    servo_result_t result;
    while((token=Console_NextToken()) != NULL) {
        if(n == 7) goto syntax;
        args[n++]=token;
    }
    if(!n || (n==1 && strcmp(args[0],"MAP")==0)) { Console_PrintServo(); return; }
    if(n==1 && strcmp(args[0],"HELP")==0) { Console_PrintServoHelp(); return; }
    if(strcmp(args[0],"SIDE")==0) {
        if(n==1) { Console_PrintServo(); return; }
        if(n!=2) goto syntax;
        if(strcmp(args[1],"R")==0 || strcmp(args[1],"RIGHT")==0) result=SERVO_SetSideRight(1);
        else if(strcmp(args[1],"L")==0 || strcmp(args[1],"LEFT")==0) result=SERVO_SetSideRight(0);
        else goto syntax;
    } else if(strcmp(args[0],"CENTER")==0 || strcmp(args[0],"HOLD")==0 || strcmp(args[0],"OFF")==0) {
        if(n>2 || !(mask=Console_ServoMask(n==2 ? args[1] : NULL))) goto syntax;
        if(strcmp(args[0],"CENTER")==0) result=SERVO_Center(mask);
        else if(strcmp(args[0],"OFF")==0) result=SERVO_Off(mask);
        else result=SERVO_Hold(mask);
    } else if(strcmp(args[0],"ANGLE")==0) {
        const char *angle;
        int16_t sign = 1;
        if(n!=4 || !(mask=Console_ServoMask(args[1]))) goto syntax;
        angle = args[2];
        if(*angle=='-') { sign=-1; angle++; }
        else if(*angle=='+') angle++;
        if(!Console_ServoNumber(angle,135,&v[0]) ||
           !Console_ServoNumber(args[3],ROBOT_SERVO_MAX_MOTION_MS,&v[1])) goto syntax;
        result=SERVO_MoveAngle(mask,(int16_t)(sign*(int16_t)v[0]),v[1]);
    } else if(strcmp(args[0],"MOVE")==0) {
        if(n!=4 || !(mask=Console_ServoMask(args[1])) ||
           !Console_ServoNumber(args[2],2500,&v[0]) || v[0]<500 ||
           !Console_ServoNumber(args[3],ROBOT_SERVO_MAX_MOTION_MS,&v[1])) goto syntax;
        us[0]=us[1]=us[2]=(uint16_t)v[0];
        result=SERVO_Move(mask,us,v[1]);
    } else if(strcmp(args[0],"POSE")==0) {
        uint8_t c;
        if(n!=5) goto syntax;
        for(c=0;c<3;c++) {
            if(!Console_ServoNumber(args[c+1],2500,&v[c]) || v[c]<500) goto syntax;
            us[c]=(uint16_t)v[c];
        }
        if(!Console_ServoNumber(args[4],ROBOT_SERVO_MAX_MOTION_MS,&v[3])) goto syntax;
        result=SERVO_Move(SERVO_MASK_ALL,us,v[3]);
    } else if(strcmp(args[0],"SWEEP")==0) {
        if(n<=2) {
            v[0]=SERVO_SWEEP_PERIOD_MS;
            if(n==2 && !Console_ServoNumber(args[1],ROBOT_SERVO_MAX_MOTION_MS,&v[0])) goto syntax;
            result=SERVO_StartSweep(SERVO_SWEEP_MIN_US,SERVO_SWEEP_MAX_US,v[0]);
        } else {
            if(n!=5 || !(mask=Console_ServoMask(args[1])) ||
               !Console_ServoNumber(args[2],2500,&v[0]) ||
               !Console_ServoNumber(args[3],2500,&v[1]) ||
               !Console_ServoNumber(args[4],ROBOT_SERVO_MAX_MOTION_MS,&v[2])) goto syntax;
            result=SERVO_Sweep(mask,(uint16_t)v[0],(uint16_t)v[1],v[2]);
        }
    } else if(n==2 && (mask=Console_ServoMask(args[0])) != 0) {
        if(!Console_ServoNumber(args[1],2500,&v[0]) || v[0]<500) goto syntax;
        us[0]=us[1]=us[2]=(uint16_t)v[0]; result=SERVO_Move(mask,us,0);
    } else if(n==1 && Console_ServoNumber(args[0],2500,&v[0]) && v[0]>=500) {
        result=SERVO_SetPulseUs((uint16_t)v[0]);
    } else goto syntax;
    LOGF("%s servo %s\r\n", result==SERVO_OK ? "ok" : "err:", SERVO_ResultName(result));
    return;
syntax:
    LOGF("err: invalid SERVO arguments\r\n");
    Console_PrintServoHelp();
}

static void Console_PrintTurntable(void)
{
    stepper_status_t status;
    char             position[16];
    char             target[16];
    float            position_deg;
    float            target_deg;

    STEPPER_GetStatus(&status);

    position_deg = (status.steps_per_rev == 0U)
                       ? 0.0f
                       : ((float)status.position_steps * 360.0f /
                          (float)status.steps_per_rev);
    target_deg = (status.steps_per_rev == 0U)
                     ? 0.0f
                     : ((float)status.target_steps * 360.0f /
                        (float)status.steps_per_rev);

    LOGF("turn state=%s dir=%d pos=%s deg (%ld pulses) target=%s deg\r\n",
         STEPPER_StateName((stepper_state_t)status.state),
         (int)status.direction,
         Console_Fixed3(position_deg, position, sizeof(position)),
         (long)status.position_steps,
         Console_Fixed3(target_deg, target, sizeof(target)));
    LOGF("     rate=%lu/%lu Hz psc=%u arr=%u ccr=%u ppr=%lu\r\n",
         (unsigned long)status.rate_hz,
         (unsigned long)status.target_rate_hz,
         (unsigned int)status.prescaler,
         (unsigned int)status.period,
         (unsigned int)status.compare,
         (unsigned long)status.steps_per_rev);
}

static uint8_t Console_ParseDirection(const char *text, int8_t *direction)
{
    int32_t value;

    if((text == NULL) || (direction == NULL))
    {
        return 0U;
    }

    if(strcmp(text, "CW") == 0)
    {
        *direction = 1;
        return 1U;
    }

    if(strcmp(text, "CCW") == 0)
    {
        *direction = -1;
        return 1U;
    }

    if((Console_ParseInt(text, &value) != 0U) &&
       ((value == 1) || (value == -1)))
    {
        *direction = (int8_t)value;
        return 1U;
    }

    return 0U;
}

static void Console_HandleTurn(void)
{
    char   *token = Console_NextToken();
    int32_t value;
    float   degrees;
    float   rpm;
    uint8_t ok;

    if(token == NULL)
    {
        Console_PrintTurntable();
        return;
    }

    if(strcmp(token, "STOP") == 0)
    {
        STEPPER_Stop();
        LOGF("ok turntable stopped\r\n");
        return;
    }

    if(strcmp(token, "ZERO") == 0)
    {
        STEPPER_Zero();
        LOGF("ok turntable zeroed\r\n");
        return;
    }

    if(strcmp(token, "PPR") == 0)
    {
        char *ppr_token = Console_NextToken();

        if(ppr_token == NULL)
        {
            LOGF("turntable ppr=%lu\r\n",
                 (unsigned long)STEPPER_GetStepsPerRev());
            return;
        }

        if((Console_ParseInt(ppr_token, &value) != 0U) && (value > 0))
        {
            STEPPER_SetStepsPerRev((uint32_t)value);
            LOGF("ok turntable ppr=%ld\r\n", (long)value);
        }
        else
        {
            LOGF("err: TURN PPR <pulses_per_rev>\r\n");
        }
        return;
    }

    if(strcmp(token, "PULSES") == 0)
    {
        char *pulse_token = Console_NextToken();
        char *rate_token = Console_NextToken();
        int32_t pulses;
        uint32_t rate_hz;

        if((pulse_token == NULL) ||
           (Console_ParseInt(pulse_token, &pulses) == 0U))
        {
            LOGF("err: TURN PULSES <n> [hz]\r\n");
            return;
        }

        if(rate_token == NULL)
        {
            rate_hz = STEPPER_RpmToRate(ROBOT_STEPPER_DEFAULT_RPM);
        }
        else if((Console_ParseInt(rate_token, &value) != 0U) &&
                (value > 0))
        {
            rate_hz = (uint32_t)value;
        }
        else
        {
            LOGF("err: TURN PULSES <n> [hz]\r\n");
            return;
        }

        if(STEPPER_MovePulses(pulses, rate_hz) == 0U)
        {
            LOGF("err: pulse move rejected, check rate\r\n");
            return;
        }

        LOGF("ok turntable pulses=%ld rate=%lu Hz\r\n",
             (long)pulses, (unsigned long)rate_hz);
        return;
    }

    if(strcmp(token, "RUN") == 0)
    {
        int8_t direction;
        char  *direction_token = Console_NextToken();
        char  *rpm_token;

        if(Console_ParseDirection(direction_token, &direction) == 0U)
        {
            LOGF("err: TURN RUN <CW|CCW> [rpm]\r\n");
            return;
        }

        rpm_token = Console_NextToken();
        if((rpm_token == NULL) ||
           (Console_ParseFloat(rpm_token, &rpm) == 0U) ||
           (rpm <= 0.0f))
        {
            rpm = ROBOT_STEPPER_DEFAULT_RPM;
        }

        ok = STEPPER_Run(direction, rpm);
        if(ok == 0U)
        {
            LOGF("err: turntable rate is outside the timer range\r\n");
            return;
        }

        LOGF("ok turntable run %s rpm=%d\r\n",
             (direction > 0) ? "CW" : "CCW", (int)rpm);
        return;
    }

    if((Console_ParseFloat(token, &degrees) != 0U))
    {
        char  *rpm_token = Console_NextToken();

        if((rpm_token == NULL) ||
           (Console_ParseFloat(rpm_token, &rpm) == 0U) ||
           (rpm <= 0.0f))
        {
            rpm = ROBOT_STEPPER_DEFAULT_RPM;
        }

        if(STEPPER_MoveDegrees(degrees, rpm) == 0U)
        {
            LOGF("err: turntable move rejected, check rate\r\n");
            return;
        }

        LOGF("ok turntable move %d deg at %d rpm\r\n",
             (int)degrees, (int)rpm);
        return;
    }

    LOGF("err: TURN [<deg> [rpm]|RUN|STOP|ZERO|PPR|PULSES]\r\n");
}

/*
 * Bench test for one bridge: drive the requested duty forward, then the same
 * duty in reverse, and report what the encoder saw in each phase. A wheel
 * whose bridge input is not really connected moves one way only, or not at
 * all, and the counts expose it immediately.
 */
static void Console_HandleWheelTest(void)
{
    int32_t index;
    int32_t duty;
    int32_t hold_ms;
    int32_t start;
    uint8_t wheel;

    if((Console_NextInt(&index) == 0U) || (index < 1) ||
       (index > (int32_t)WHEEL_COUNT))
    {
        LOGF("err: WTEST <n> [duty] [ms], n = 1..4\r\n");
        return;
    }

    wheel = (uint8_t)(index - 1U);

    if((Console_NextInt(&duty) == 0U) || (duty < 50) ||
       (duty > (int32_t)MOTOR_DUTY_MAX))
    {
        duty = (int32_t)(MOTOR_DUTY_MAX / 3);
    }

    if((Console_NextInt(&hold_ms) == 0U) || (hold_ms < 50) ||
       (hold_ms > 10000))
    {
        hold_ms = 800;
    }

    LOGF("WTEST %s duty=%d hold=%d ms per direction\r\n",
         s_wheel_name[wheel], (int)duty, (int)hold_ms);

    start = ENCODER_GetCount(wheel);

    CHASSIS_CommandOpenLoop(wheel, (int16_t)duty);
    vTaskDelay(pdMS_TO_TICKS((uint32_t)hold_ms));
    LOGF("  duty=%+d moved=%ld counts rpm=%d\r\n",
         (int)duty,
         (long)(ENCODER_GetCount(wheel) - start),
         (int)ENCODER_GetRpm(wheel));

    CHASSIS_CommandOpenLoop(wheel, (int16_t)(-duty));
    vTaskDelay(pdMS_TO_TICKS((uint32_t)hold_ms));
    LOGF("  duty=%+d moved=%ld counts rpm=%d (net from start)\r\n",
         (int)(-duty),
         (long)(ENCODER_GetCount(wheel) - start),
         (int)ENCODER_GetRpm(wheel));

    CHASSIS_CommandStop();
    LOGF("  WTEST done, motors coasting\r\n");
}

static void Console_WaitStepperIdle(void)
{
    stepper_status_t status;
    uint32_t         waited_ms = 0U;

    do
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        STEPPER_GetStatus(&status);
        waited_ms += 20U;
    } while((status.state != STEPPER_STATE_IDLE) && (waited_ms < 20000U));
}

/*
 * Bench test for the stepper wiring: N pulses one way, then N back, printing
 * the pulse counter and the DIR pin so a swapped or missing direction line is
 * obvious.
 */
static void Console_HandleStepperTest(void)
{
    stepper_wiring_t wiring;
    stepper_status_t status;
    int32_t          value;
    int32_t          pulses = 200;
    int32_t          rate_hz = 400;
    int32_t          start_pos;

    if((Console_NextInt(&value) != 0U) && (value > 0) && (value <= 100000))
    {
        pulses = value;
    }

    if((Console_NextInt(&value) != 0U) && (value >= 50) &&
       (value <= (int32_t)ROBOT_STEPPER_MAX_RATE_HZ))
    {
        rate_hz = value;
    }

    STEPPER_GetWiring(&wiring);
    STEPPER_GetStatus(&status);
    start_pos = status.position_steps;

    LOGF("STEST pulses=%d rate=%d Hz STEP=%s%u DIR=%s%u level=%u pos=%ld\r\n",
         (int)pulses, (int)rate_hz,
         wiring.step_port, (unsigned int)wiring.step_pin,
         wiring.dir_port, (unsigned int)wiring.dir_pin,
         (unsigned int)STEPPER_GetDirLevel(),
         (long)status.position_steps);

    if(STEPPER_MovePulses(pulses, (uint32_t)rate_hz) == 0U)
    {
        LOGF("err: STEST forward move rejected\r\n");
        return;
    }

    Console_WaitStepperIdle();
    STEPPER_GetStatus(&status);
    LOGF("  +%d pulses -> pos=%ld moved=%ld DIR level=%u\r\n",
         (int)pulses, (long)status.position_steps,
         (long)(status.position_steps - start_pos),
         (unsigned int)STEPPER_GetDirLevel());

    if(STEPPER_MovePulses(-pulses, (uint32_t)rate_hz) == 0U)
    {
        LOGF("err: STEST reverse move rejected\r\n");
        return;
    }

    Console_WaitStepperIdle();
    STEPPER_GetStatus(&status);
    LOGF("  -%d pulses -> pos=%ld moved=%ld DIR level=%u\r\n",
         (int)pulses, (long)status.position_steps,
         (long)(status.position_steps - start_pos),
         (unsigned int)STEPPER_GetDirLevel());
    LOGF("  STEST done; the pulse counter is generated, so watch the turntable:\r\n");
    LOGF("  it must turn both ways, otherwise DIR never reaches the driver\r\n");
}

/* ------------------------------------------------------------------ */
/* command handlers                                                    */
/* ------------------------------------------------------------------ */

static void Console_HandleEncoder(void)
{
    char *token = Console_NextToken();

    if((token != NULL) &&
       ((strcmp(token, "R") == 0) || (strcmp(token, "RESET") == 0)))
    {
        ENCODER_ResetCounters();
        ENCODER_ResetDiagnostics();
        CHASSIS_ResetPose();
        LOGF("ok encoder counters, diagnostics and pose reset\r\n");
        return;
    }

    Console_PrintEncoders();

    if(token != NULL)
    {
        LOGF("usage: E [R]\r\n");
    }
}

static void Console_HandlePid(void)
{
    char   *first;
    char   *token;
    int32_t index = 0;
    float   kp;
    float   ki;
    float   kd;
    char    buffer[3][16];
    uint8_t wheel = 0U;
    uint8_t all = 0U;

    first = Console_NextToken();
    if(first == NULL)
    {
        LOGF("err: PID <n|ALL> [kp ki kd]\r\n");
        return;
    }

    if(strcmp(first, "ALL") == 0)
    {
        all = 1U;
    }
    else
    {
        if((Console_ParseInt(first, &index) == 0U) || (index < 1) ||
           (index > (int32_t)WHEEL_COUNT))
        {
            LOGF("err: PID <n|ALL> [kp ki kd]\r\n");
            return;
        }

        wheel = (uint8_t)(index - 1U);
    }

    WHEEL_CONTROL_GetGains(wheel, &kp, &ki, &kd);

    token = Console_NextToken();
    if(token != NULL)
    {
        char *ki_token = Console_NextToken();
        char *kd_token = Console_NextToken();

        if((ki_token != NULL) && (kd_token != NULL) &&
           (Console_ParseFloat(token, &kp) != 0U) &&
           (Console_ParseFloat(ki_token, &ki) != 0U) &&
           (Console_ParseFloat(kd_token, &kd) != 0U))
        {
            if(all != 0U)
            {
                uint8_t i;

                for(i = 0U; i < WHEEL_COUNT; i++)
                {
                    WHEEL_CONTROL_SetGains(i, kp, ki, kd);
                }
            }
            else
            {
                WHEEL_CONTROL_SetGains(wheel, kp, ki, kd);
            }
        }
        else
        {
            LOGF("err: PID <n|ALL> [kp ki kd]\r\n");
            return;
        }
    }

    if(all != 0U)
    {
        LOGF("pid ALL kp=%s ki=%s kd=%s\r\n",
             Console_Fixed3(kp, buffer[0], sizeof(buffer[0])),
             Console_Fixed3(ki, buffer[1], sizeof(buffer[1])),
             Console_Fixed3(kd, buffer[2], sizeof(buffer[2])));
    }
    else
    {
        LOGF("pid %u kp=%s ki=%s kd=%s\r\n",
             (unsigned int)wheel + 1U,
             Console_Fixed3(kp, buffer[0], sizeof(buffer[0])),
             Console_Fixed3(ki, buffer[1], sizeof(buffer[1])),
             Console_Fixed3(kd, buffer[2], sizeof(buffer[2])));
    }
}

static void Console_HandleSpeed(void)
{
    char   *first;
    int32_t index;
    float   rpm;
    uint8_t wheel;

    first = Console_NextToken();
    if(first == NULL)
    {
        LOGF("err: SPD <n|ALL> <rpm>\r\n");
        return;
    }

    if(strcmp(first, "ALL") == 0)
    {
        if(Console_NextFloat(&rpm) == 0U)
        {
            LOGF("err: SPD ALL <rpm>\r\n");
            return;
        }

        for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
        {
            CHASSIS_CommandWheelRpm(wheel, rpm);
        }

        LOGF("ok SPD ALL rpm=%d\r\n", (int)rpm);
        return;
    }

    if((Console_ParseInt(first, &index) == 0U) || (index < 1) ||
       (index > (int32_t)WHEEL_COUNT) || (Console_NextFloat(&rpm) == 0U))
    {
        LOGF("err: SPD <n|ALL> <rpm>\r\n");
        return;
    }

    wheel = (uint8_t)(index - 1U);
    CHASSIS_CommandWheelRpm(wheel, rpm);
    LOGF("ok SPD %s rpm=%d\r\n", s_wheel_name[wheel], (int)rpm);
}

static void Console_HandleVofa(void)
{
    char   *token;
    int32_t value;
    uint8_t wheel = TELEMETRY_VOFA_ALL;

    token = Console_NextToken();
    if(token == NULL)
    {
        LOGF("err: VOFA <n|ALL|OFF> [period_ms]\r\n");
        return;
    }

    if(strcmp(token, "OFF") == 0)
    {
        TELEMETRY_SetEnabled(0U);
        LOGF("ok VOFA off\r\n");
        return;
    }

    if(strcmp(token, "ALL") != 0)
    {
        if((Console_ParseInt(token, &value) == 0U) || (value < 1) ||
           (value > (int32_t)WHEEL_COUNT))
        {
            LOGF("err: VOFA <n|ALL|OFF> [period_ms]\r\n");
            return;
        }

        wheel = (uint8_t)(value - 1);
    }

    token = Console_NextToken();
    if(token != NULL)
    {
        if((Console_ParseInt(token, &value) == 0U) ||
           (value < 10) || (value > 5000))
        {
            LOGF("err: VOFA <n|ALL|OFF> [period_ms]\r\n");
            return;
        }

        TELEMETRY_SetPeriodMs((uint32_t)value);
    }

    TELEMETRY_SetVofaWheel(wheel);
    TELEMETRY_SetFormat(TELEMETRY_FORMAT_VOFA);
    TELEMETRY_SetEnabled(1U);

    LOGF("ok VOFA %s period=%u ms FireWater\r\n",
         (wheel == TELEMETRY_VOFA_ALL)
             ? "ALL"
             : s_wheel_name[wheel],
         (unsigned int)TELEMETRY_GetPeriodMs());
}

static void Console_HandleLine(char *line)
{
    char   *command;
    char   *token;
    int32_t a;
    int32_t b;
    float   fa;
    float   fb;
    float   fc;
    char   *p;

    for(p = line; *p != '\0'; p++)
    {
        *p = (char)toupper((unsigned char)*p);
    }

    command = strtok(line, " \t");
    if(command == NULL)
    {
        return;
    }

    if((strcmp(command, "H") == 0) || (strcmp(command, "?") == 0) ||
       (strcmp(command, "HELP") == 0))
    {
        Console_PrintHelp();
    }
    else if((strcmp(command, "P") == 0) || (strcmp(command, "STATUS") == 0))
    {
        Console_PrintStatus();
    }
    else if(strcmp(command, "V") == 0)
    {
        if((Console_NextFloat(&fa) != 0U) &&
           (Console_NextFloat(&fb) != 0U) &&
           (Console_NextFloat(&fc) != 0U))
        {
            if(CHASSIS_IsReady() == 0U)
            {
                LOGF("warn: a wheel has no speed feedback, DRIVE will be refused\r\n");
                LOGF("      wire the missing channel or set OL <n> <gain>\r\n");
            }

            CHASSIS_CommandTwist(fa, fb, fc);
            LOGF("ok twist vx=%d vy=%d wz=%d mrad/s\r\n",
                 (int)fa, (int)fb, (int)(fc * 1000.0f));
        }
        else
        {
            LOGF("err: V <vx_mm_s> <vy_mm_s> <wz_rad_s>\r\n");
        }
    }
    else if((strcmp(command, "S") == 0) ||
            (strcmp(command, "X") == 0) ||
            (strcmp(command, "STOP") == 0))
    {
        CHASSIS_CommandStop();
        LOGF("ok stop, motors coasting\r\n");
    }
    else if(strcmp(command, "B") == 0)
    {
        CHASSIS_CommandStop();
        MOTOR_BrakeAll();
        LOGF("ok brake\r\n");
    }
    else if(strcmp(command, "W") == 0)
    {
        if((Console_NextInt(&a) != 0U) && (Console_NextInt(&b) != 0U) &&
           (a >= 1) && (a <= (int32_t)WHEEL_COUNT))
        {
            CHASSIS_CommandOpenLoop((uint8_t)(a - 1), (int16_t)b);
            LOGF("ok %s duty=%d\r\n", s_wheel_name[a - 1], (int)b);
        }
        else
        {
            LOGF("err: W <n> <duty>, n = 1..4\r\n");
        }
    }
    else if(strcmp(command, "SPD") == 0)
    {
        Console_HandleSpeed();
    }
    else if(strcmp(command, "WTEST") == 0)
    {
        Console_HandleWheelTest();
    }
    else if(strcmp(command, "STEST") == 0)
    {
        Console_HandleStepperTest();
    }
    else if(strcmp(command, "DIR") == 0)
    {
        if((Console_NextInt(&a) != 0U) && ((a == 0) || (a == 1)))
        {
            if(STEPPER_SetDirLevel((uint8_t)a) == 0U)
            {
                LOGF("err: DIR is latched while moving, send TURN STOP first\r\n");
            }
            else
            {
                LOGF("ok stepper DIR=%d (%s) readback=%u\r\n",
                     (int)a,
                     (a != 0) ? "high" : "low",
                     (unsigned int)STEPPER_GetDirLevel());
            }
        }
        else
        {
            stepper_wiring_t wiring;

            STEPPER_GetWiring(&wiring);
            LOGF("stepper DIR=%s%u level=%u\r\n",
                 wiring.dir_port, (unsigned int)wiring.dir_pin,
                 (unsigned int)STEPPER_GetDirLevel());
            LOGF("usage: DIR <0|1>, 1 = positive step direction\r\n");
        }
    }
    else if(strcmp(command, "PINS") == 0)
    {
        Console_PrintPins();
    }
    else if(strcmp(command, "M") == 0)
    {
        int32_t duty[WHEEL_COUNT];
        uint8_t wheel;
        uint8_t valid = 1U;

        for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
        {
            if(Console_NextInt(&duty[wheel]) == 0U)
            {
                valid = 0U;
                break;
            }
        }

        if(valid != 0U)
        {
            for(wheel = 0U; wheel < WHEEL_COUNT; wheel++)
            {
                CHASSIS_CommandOpenLoop(wheel, (int16_t)duty[wheel]);
            }

            LOGF("ok open loop %d %d %d %d\r\n",
                 (int)duty[0], (int)duty[1], (int)duty[2], (int)duty[3]);
        }
        else
        {
            LOGF("err: M <d1> <d2> <d3> <d4>\r\n");
        }
    }
    else if(strcmp(command, "RAMP") == 0)
    {
        if((Console_NextInt(&a) != 0U) && (a >= 0) && (a <= MOTOR_DUTY_MAX))
        {
            MOTOR_SetSlewLimit((int16_t)a);
            LOGF("ok ramp=%d duty per control period\r\n", (int)a);
        }
        else
        {
            LOGF("err: RAMP <0..%d>\r\n", (int)MOTOR_DUTY_MAX);
        }
    }
    else if(strcmp(command, "INV") == 0)
    {
        if((Console_NextInt(&a) != 0U) && (Console_NextInt(&b) != 0U) &&
           (a >= 1) && (a <= (int32_t)WHEEL_COUNT) && ((b == 0) || (b == 1)))
        {
            MOTOR_SetInvert((uint8_t)(a - 1), (uint8_t)b);
            LOGF("ok INV %d %d\r\n", (int)a, (int)b);
        }
        else
        {
            LOGF("err: INV <n> <0|1>\r\n");
        }
    }
    else if(strcmp(command, "MODE") == 0)
    {
        if((Console_NextInt(&a) != 0U) && ((a == 0) || (a == 1)))
        {
            MOTOR_SetDriveMode((a == 0) ? MOTOR_MODE_DUAL_PWM : MOTOR_MODE_PWM_DIR);
            LOGF("ok MODE %d (%s)\r\n", (int)a,
                 (a == 0) ? "dual PWM" : "PWM + DIR");
        }
        else
        {
            LOGF("err: MODE <0|1>\r\n");
        }
    }
    else if(strcmp(command, "E") == 0)
    {
        Console_HandleEncoder();
    }
    else if(strcmp(command, "SIGN") == 0)
    {
        if((Console_NextInt(&a) != 0U) && (Console_NextInt(&b) != 0U) &&
           (a >= 1) && (a <= (int32_t)WHEEL_COUNT) && ((b == 1) || (b == -1)))
        {
            ENCODER_SetSign((uint8_t)(a - 1), (int8_t)b);
            LOGF("ok SIGN %d %d\r\n", (int)a, (int)b);
        }
        else
        {
            LOGF("err: SIGN <n> <1|-1>\r\n");
        }
    }
    else if(strcmp(command, "PID") == 0)
    {
        Console_HandlePid();
    }
    else if(strcmp(command, "VOFA") == 0)
    {
        Console_HandleVofa();
    }
    else if(strcmp(command, "OL") == 0)
    {
        if((Console_NextInt(&a) != 0U) && (a >= 1) &&
           (a <= (int32_t)WHEEL_COUNT) &&
           (Console_NextFloat(&fa) != 0U) && (fa >= 0.0f))
        {
            WHEEL_CONTROL_SetOpenLoopGain((uint8_t)(a - 1), fa);
            LOGF("ok OL %d %d mDuty/rpm\r\n", (int)a, (int)(fa * 1000.0f));
        }
        else
        {
            LOGF("err: OL <n> <duty_per_rpm>\r\n");
        }
    }
    else if(strcmp(command, "CLR") == 0)
    {
        CHASSIS_ClearFaults();
        LOGF("ok faults cleared\r\n");
    }
    else if(strcmp(command, "POSE") == 0)
    {
        token = Console_NextToken();
        if((token != NULL) && (strcmp(token, "R") == 0))
        {
            CHASSIS_ResetPose();
            LOGF("ok pose reset\r\n");
        }
        else
        {
            chassis_status_t status;

            CHASSIS_GetStatus(&status);
            LOGF("pose x=%d mm y=%d mm yaw=%d mrad\r\n",
                 (int)status.pose.x_mm,
                 (int)status.pose.y_mm,
                 (int)(status.pose.yaw_rad * 1000.0f));
        }
    }
    else if(strcmp(command, "TELEM") == 0)
    {
        token = Console_NextToken();

        if(token == NULL)
        {
            LOGF("telemetry=%s format=%s period=%u ms\r\n",
                 (TELEMETRY_GetEnabled() != 0U) ? "on" : "off",
                 (TELEMETRY_GetFormat() == TELEMETRY_FORMAT_VOFA)
                     ? "VOFA"
                     : "CSV",
                 (unsigned int)TELEMETRY_GetPeriodMs());
        }
        else if(strcmp(token, "ON") == 0)
        {
            TELEMETRY_SetEnabled(1U);
            LOGF("ok telemetry on\r\n");
        }
        else if(strcmp(token, "OFF") == 0)
        {
            TELEMETRY_SetEnabled(0U);
            LOGF("ok telemetry off\r\n");
        }
        else if(strcmp(token, "CSV") == 0)
        {
            TELEMETRY_SetFormat(TELEMETRY_FORMAT_CSV);
            TELEMETRY_SetEnabled(1U);
            LOGF("ok telemetry CSV\r\n");
        }
        else if((Console_ParseInt(token, &a) != 0U) && (a >= 10) && (a <= 5000))
        {
            TELEMETRY_SetPeriodMs((uint32_t)a);
            TELEMETRY_SetEnabled(1U);
            LOGF("ok telemetry %d ms\r\n", (int)a);
        }
        else
        {
            LOGF("err: TELEM <on|off|period_ms|csv>\r\n");
        }
    }
    else if(strcmp(command, "CFG") == 0)
    {
        Console_PrintConfig();
    }
    else if(strcmp(command, "SERVO") == 0)
    {
        Console_HandleServo();
    }
    else if(strcmp(command, "TURN") == 0)
    {
        Console_HandleTurn();
    }
    else if(strcmp(command, "PPRCAL") == 0)
    {
        int32_t pulses;
        float   degrees;
        char    buffer[16];

        if((Console_NextInt(&pulses) != 0U) && (pulses > 0) &&
           (Console_NextFloat(&degrees) != 0U) && (degrees > 0.0f))
        {
            float ppr_f = ((float)pulses * 360.0f) / degrees;

            if((ppr_f >= 100.0f) && (ppr_f <= 200000.0f))
            {
                uint32_t ppr = (uint32_t)(ppr_f + 0.5f);

                STEPPER_SetStepsPerRev(ppr);
                LOGF("ok turntable ppr=%lu (%ld pulses moved %s deg)\r\n",
                     (unsigned long)ppr, (long)pulses,
                     Console_Fixed3(degrees, buffer, sizeof(buffer)));
            }
            else
            {
                LOGF("err: computed ppr out of range, check the angle\r\n");
            }
        }
        else
        {
            LOGF("usage: PPRCAL <pulses> <measured_degrees>\r\n");
            LOGF("       run TURN PULSES <pulses> <hz> first, then measure\r\n");
        }
    }
    else if(strcmp(command, "STEP") == 0)
    {
        if((Console_NextInt(&a) != 0U) && ((a == 0) || (a == 1)))
        {
            if(STEPPER_SetStepLevel((uint8_t)a) == 0U)
            {
                LOGF("err: STEP is busy while moving, send TURN STOP first\r\n");
            }
            else
            {
                LOGF("ok stepper STEP=%d readback=%u\r\n",
                     (int)a, (unsigned int)STEPPER_GetStepLevel());
            }
        }
        else
        {
            stepper_wiring_t wiring;

            STEPPER_GetWiring(&wiring);
            LOGF("stepper STEP=%s%u level=%u\r\n",
                 wiring.step_port, (unsigned int)wiring.step_pin,
                 (unsigned int)STEPPER_GetStepLevel());
            LOGF("usage: STEP <0|1>, 1 = +3.3 V on the module STEP pin\r\n");
        }
    }
    else
    {
        LOGF("err: unknown '%s', send H for help\r\n", command);
    }
}

void CONSOLE_Init(void)
{
    COMM_Init(ROBOT_CONSOLE_BAUDRATE);
}

/* Report boot-test milestones in task context, never from the PWM ISR. */
static void Console_ReportServoTest(void)
{
    static uint8_t previous[3] = {0,0,0};
    servo_status_t s;
    uint8_t c;
    if(!ROBOT_SERVO_BOOT_TEST) return;
    SERVO_GetStatus(&s);
    for(c=0;c<3;c++) {
        uint8_t phase = s.boot_test_phase[0][c];
        uint16_t low, center, high;
        if(phase == previous[c]) continue;
        SERVO_GetLimits(0,c,&low,&center,&high);
        if(phase == 1) {
            LOGF("servo test R/%s: start, %u -> %u us (1500 ms)\r\n",
                 SERVO_ChannelName(c), s.saved_us[0][c], low);
        } else if(phase == 2) {
            LOGF("servo test R/%s: min %u us reached; -> %u us (3000 ms)\r\n",
                 SERVO_ChannelName(c), low, high);
        } else if(phase == 3) {
            LOGF("servo test R/%s: max %u us reached; -> %u us (1500 ms)\r\n",
                 SERVO_ChannelName(c), high, center);
        } else if(previous[c] == 3 && s.enabled[0][c] &&
                  s.motion[0][c] == SERVO_HOLD && s.saved_us[0][c] == center) {
            LOGF("servo test R/%s: complete, hold %u us\r\n", SERVO_ChannelName(c), center);
        } else {
            LOGF("servo test R/%s: interrupted, command=%u us PWM=%s\r\n",
                 SERVO_ChannelName(c), s.saved_us[0][c], s.enabled[0][c] ? "on" : "off");
        }
        previous[c] = phase;
    }
}

void CONSOLE_Task(void *argument)
{
    char line[COMM_LINE_MAX];

    (void)argument;

    for(;;)
    {
        Console_ReportServoTest();
        while(COMM_PollLine(line, sizeof(line)) != 0U)
        {
            Console_HandleLine(line);
        }

        vTaskDelay(pdMS_TO_TICKS(ROBOT_CONSOLE_POLL_MS));
    }
}
