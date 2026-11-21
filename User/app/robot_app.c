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

typedef enum
{
    LINK_IDLE, LINK_RETRACT, LINK_TURN_PICK, LINK_PICK, LINK_TURN_PLACE,
    LINK_EXTEND, LINK_OPEN, LINK_RETRACT_END, LINK_TURN_HOME, LINK_ARM, LINK_LEVEL,
    LINK_STOPPING, LINK_DONE, LINK_FAULT
} link_phase_t;

static const char *const s_link_names[] = {
    "idle", "retract", "turn_pick", "pick", "turn_place", "extend",
    "open", "retract_end", "turn_home", "arm", "arm_level", "stopping", "done", "fault"
};
static volatile link_phase_t s_link_phase;
static volatile uint8_t s_link_stop, s_link_zero;
static const char *volatile s_link_fault = "-";
static int32_t s_link_pick, s_link_place;
static uint32_t s_link_rate;
static uint8_t s_link_home_only, s_link_return_home;
static servo_action_t s_link_arm_action;
static uint16_t s_link_arm_pose[3];
static uint16_t s_link_level_start, s_link_level_target;
static TickType_t s_link_level_started, s_link_level_duration;
static const uint16_t s_level_poses[][2] = {
    {1000U, 2100U}, {1200U, 1900U}, {1300U, 1800U},
    {1400U, 1600U}, {1800U, 1200U}
};
#define LEVEL_POSE_COUNT (sizeof(s_level_poses) / sizeof(s_level_poses[0]))
static uint8_t s_link_issued;
static TickType_t s_link_deadline, s_link_settle;
static uint8_t s_link_settling;

uint8_t ROBOT_LINK_Active(void)
{
    link_phase_t phase = s_link_phase;
    return phase >= LINK_RETRACT && phase <= LINK_STOPPING;
}

uint8_t ROBOT_LINK_Zero(void)
{
    stepper_status_t step;
    STEPPER_GetStatus(&step);
    if(ROBOT_LINK_Active() || step.state != STEPPER_STATE_IDLE) return 0U;
    STEPPER_Zero();
    s_link_zero = 1U;
    return 1U;
}

void ROBOT_LINK_InvalidateZero(void)
{
    s_link_zero = 0U;
}

static uint8_t Link_ServoIdle(const servo_status_t *servo)
{
    return !servo->queued && !servo->action && servo->side_right &&
           servo->hardware_side_right && servo->requested_side_right &&
           servo->motion[0][0] == SERVO_HOLD &&
           servo->motion[0][1] == SERVO_HOLD && servo->motion[0][2] == SERVO_HOLD;
}

static void Link_Enter(link_phase_t phase)
{
    s_link_issued = 0U;
    s_link_settling = 0U;
    s_link_phase = phase;
}

static uint8_t Link_Ready(const stepper_status_t *step)
{
    chassis_status_t chassis;
    servo_status_t servo;
    CHASSIS_GetStatus(&chassis);
    SERVO_GetStatus(&servo);
    return !ROBOT_LINK_Active() && chassis.mode == CHASSIS_MODE_IDLE &&
           servo.config_valid && servo.installed_mask[0] == SERVO_MASK_ALL &&
           Link_ServoIdle(&servo) && step->state == STEPPER_STATE_IDLE;
}

static uint8_t Link_Start(int32_t pick_deg, int32_t place_deg, uint32_t rate,
                          uint8_t home_only, uint8_t return_home)
{
    stepper_status_t step;
    int32_t pick, place, low, high;
    uint32_t ppr;

    if(pick_deg < -ROBOT_LINK_MAX_ANGLE_DEG || pick_deg > ROBOT_LINK_MAX_ANGLE_DEG ||
       place_deg < -ROBOT_LINK_MAX_ANGLE_DEG || place_deg > ROBOT_LINK_MAX_ANGLE_DEG) return 0U;

    ppr = STEPPER_GetStepsPerRev();
    pick = (int32_t)(((int64_t)pick_deg * ppr) / 360);
    place = (int32_t)(((int64_t)place_deg * ppr) / 360);
    low = pick < place ? pick : place;
    high = pick > place ? pick : place;

    if(ROBOT_LINK_Active() || !s_link_zero || pick == place ||
       rate < ROBOT_STEPPER_START_RATE_HZ || rate > ROBOT_STEPPER_MAX_RATE_HZ ||
       (int64_t)high - low > INT32_MAX) return 0U;
    STEPPER_GetStatus(&step);
    if(!Link_Ready(&step) ||
       step.position_steps < (low < 0 ? low : 0) ||
       step.position_steps > (high > 0 ? high : 0) ||
       (int64_t)pick - step.position_steps < -INT32_MAX ||
       (int64_t)pick - step.position_steps > INT32_MAX) return 0U;

    s_link_pick = pick;
    s_link_place = place;
    s_link_rate = rate;
    s_link_home_only = home_only;
    s_link_return_home = return_home;
    s_link_stop = 0U;
    s_link_fault = "-";
    Link_Enter(LINK_RETRACT);
    return 1U;
}

uint8_t ROBOT_LINK_Start(int32_t pick_deg, int32_t place_deg, float rpm)
{
    if(!(rpm >= ROBOT_STEPPER_MIN_RPM && rpm <= ROBOT_STEPPER_MAX_RPM)) return 0U;
    return Link_Start(pick_deg, place_deg, STEPPER_RpmToRate(rpm), 0U, 0U);
}

uint8_t ROBOT_APP_Run(uint8_t home_only)
{
    if(STEPPER_GetStepsPerRev() == ROBOT_STEPPER_STEPS_PER_REV) return 0U;
    return Link_Start(home_only ? ROBOT_LINK_MAX_ANGLE_DEG : ROBOT_APP_PICK_ANGLE_DEG,
                      home_only ? -ROBOT_LINK_MAX_ANGLE_DEG : ROBOT_APP_PLACE_ANGLE_DEG,
                      ROBOT_APP_TURN_RATE_HZ, home_only, 1U);
}

uint8_t ROBOT_APP_Arm(servo_action_t action)
{
    stepper_status_t step;
    STEPPER_GetStatus(&step);
    if(!Link_Ready(&step)) return 0U;
    s_link_arm_action = action;
    s_link_stop = 0U;
    s_link_fault = "-";
    Link_Enter(LINK_ARM);
    return 1U;
}

uint8_t ROBOT_APP_ArmPose(uint16_t a_us, uint16_t b_us)
{
    uint16_t a_min, a_max, b_min, b_max, center;
    SERVO_GetLimits(0U, 0U, &a_min, &center, &a_max);
    SERVO_GetLimits(0U, 1U, &b_min, &center, &b_max);
    if(a_us < a_min || a_us > a_max || b_us < b_min || b_us > b_max ||
       ROBOT_LINK_Active()) return 0U;
    s_link_arm_pose[0] = a_us;
    s_link_arm_pose[1] = b_us;
    return ROBOT_APP_Arm(SERVO_ACTION_NONE);
}

static uint16_t Link_LevelB(uint16_t a_us)
{
    unsigned int i;
    for(i = 1U; i < LEVEL_POSE_COUNT; i++) {
        const uint16_t *lo = s_level_poses[i - 1U];
        const uint16_t *hi = s_level_poses[i];
        if(a_us <= hi[0]) {
            uint32_t span = hi[0] - lo[0];
            uint32_t drop = (uint32_t)(a_us - lo[0]) * (lo[1] - hi[1]);
            return (uint16_t)(lo[1] - (drop + span / 2U) / span);
        }
    }
    return s_level_poses[LEVEL_POSE_COUNT - 1U][1];
}

uint8_t ROBOT_APP_ArmLevel(uint16_t a_us)
{
    stepper_status_t step;
    servo_status_t servo;
    uint16_t current;
    STEPPER_GetStatus(&step);
    if(!Link_Ready(&step)) return 0U;
    SERVO_GetStatus(&servo);
    current = servo.saved_us[0][0];
    if(a_us < s_level_poses[0][0] || a_us > s_level_poses[LEVEL_POSE_COUNT - 1U][0] ||
       current < s_level_poses[0][0] || current > s_level_poses[LEVEL_POSE_COUNT - 1U][0] ||
       servo.saved_us[0][1] != Link_LevelB(current)) return 0U;
    s_link_level_start = current;
    s_link_level_target = a_us;
    s_link_level_duration = pdMS_TO_TICKS(1000U + 4U *
        (uint32_t)(a_us > current ? a_us - current : current - a_us));
    s_link_stop = 0U;
    s_link_fault = "-";
    Link_Enter(LINK_LEVEL);
    return 1U;
}

void ROBOT_LINK_Stop(void)
{
    taskENTER_CRITICAL();
    if(ROBOT_LINK_Active()) s_link_stop = 1U;
    taskEXIT_CRITICAL();
}

void ROBOT_LINK_PrintStatus(void)
{
    stepper_status_t step;
    link_phase_t phase = s_link_phase;
    int32_t target;
    STEPPER_GetStatus(&step);
    target = phase == LINK_TURN_PICK ? s_link_pick :
             phase == LINK_TURN_PLACE ? s_link_place :
             phase == LINK_TURN_HOME ? 0 : step.position_steps;
    LOGF("link %s zero=%u position=%ld angle_deg=%ld target_deg=%ld pick=%ld place=%ld reason=%s\r\n",
         s_link_names[phase], (unsigned)s_link_zero, (long)step.position_steps,
         (long)(((int64_t)step.position_steps * 360) / step.steps_per_rev),
         (long)(((int64_t)target * 360) / step.steps_per_rev),
         (long)s_link_pick, (long)s_link_place, s_link_fault);
}

static void Link_Abort(const char *reason)
{
    stepper_status_t step;
    STEPPER_GetStatus(&step);
    if(step.state != STEPPER_STATE_IDLE) ROBOT_LINK_InvalidateZero();
    STEPPER_Stop();
    s_link_fault = reason;
    Link_Enter(LINK_STOPPING);
}

static void Link_UpdateLevel(TickType_t now, const servo_status_t *servo)
{
    TickType_t elapsed;
    int32_t delta = (int32_t)s_link_level_target - s_link_level_start;
    uint32_t t, blend;
    uint16_t pulse[3];

    if(!s_link_issued) {
        s_link_level_started = now;
        s_link_issued = 1U;
    }
    elapsed = now - s_link_level_started;
    if(elapsed >= s_link_level_duration) {
        pulse[0] = s_link_level_target;
    } else {
        t = (uint32_t)(((uint64_t)elapsed * 10000U) / s_link_level_duration);
        blend = (uint32_t)(((uint64_t)t * t * (30000U - 2U * t)) / 100000000U);
        pulse[0] = (uint16_t)(s_link_level_start + (delta * (int32_t)blend) / 10000);
    }
    pulse[1] = Link_LevelB(pulse[0]);
    pulse[2] = 0U;
    if(servo->queued) return;
    if(servo->saved_us[0][0] != pulse[0] || servo->saved_us[0][1] != pulse[1]) {
        if(SERVO_Move(3U, pulse, 0U) != SERVO_OK) Link_Abort("servo_rejected");
    } else if(elapsed >= s_link_level_duration && Link_ServoIdle(servo)) {
        Link_Enter(LINK_DONE);
    }
}

static void Link_Update(void)
{
    static const servo_action_t actions[] = {
        [LINK_RETRACT] = SERVO_ACTION_RETRACT, [LINK_PICK] = SERVO_ACTION_PICK,
        [LINK_EXTEND] = SERVO_ACTION_EXTEND, [LINK_OPEN] = SERVO_ACTION_OPEN,
        [LINK_RETRACT_END] = SERVO_ACTION_RETRACT
    };
    servo_status_t servo;
    stepper_status_t step;
    chassis_status_t chassis;
    TickType_t now = xTaskGetTickCount();
    link_phase_t phase = s_link_phase;
    uint8_t turning;
    uint8_t complete;
    int32_t target;

    if(!ROBOT_LINK_Active()) return;
    if(phase != LINK_STOPPING) {
        CHASSIS_GetStatus(&chassis);
        if(s_link_stop || chassis.mode != CHASSIS_MODE_IDLE) {
            if(chassis.mode != CHASSIS_MODE_IDLE) CHASSIS_CommandStop();
            Link_Abort(s_link_stop ? "stopped" : "chassis_active");
            phase = LINK_STOPPING;
        }
    }
    SERVO_GetStatus(&servo);
    if(phase == LINK_STOPPING) {
        if(!s_link_issued) s_link_issued = SERVO_Hold(SERVO_MASK_ALL) == SERVO_OK;
        else if(Link_ServoIdle(&servo)) Link_Enter(LINK_FAULT);
        return;
    }
    if(phase == LINK_LEVEL) {
        Link_UpdateLevel(now, &servo);
        return;
    }

    turning = phase == LINK_TURN_PICK || phase == LINK_TURN_PLACE || phase == LINK_TURN_HOME;
    target = phase == LINK_TURN_PICK ? s_link_pick :
             phase == LINK_TURN_PLACE ? s_link_place : 0;
    STEPPER_GetStatus(&step);
    if(!s_link_issued) {
        uint32_t timeout_ms = 7000U;
        if(turning) {
            int32_t delta = target - step.position_steps;
            uint32_t distance = (uint32_t)(delta < 0 ? -delta : delta);
            uint64_t budget_ms = 3000U + (1000ULL * distance) / s_link_rate +
                (1000ULL * s_link_rate) / ROBOT_STEPPER_ACCEL_HZ_PER_S +
                (1000ULL * s_link_rate) / ROBOT_STEPPER_DECEL_HZ_PER_S;
            if(budget_ms > 120000U) {
                Link_Abort("turn_too_long");
                return;
            }
            timeout_ms = (uint32_t)budget_ms;
            if(delta && !STEPPER_MovePulses(delta, s_link_rate)) {
                Link_Abort("turn_rejected");
                return;
            }
        } else if(phase == LINK_ARM && s_link_arm_action == SERVO_ACTION_NONE) {
            /* A lowers the arm, B compensates pitch, C retains its command. */
            if(SERVO_Move(3U, s_link_arm_pose, ROBOT_SERVO_ACTION_MS) != SERVO_OK) {
                Link_Abort("servo_rejected");
                return;
            }
        } else if(SERVO_Action(phase == LINK_ARM ? s_link_arm_action : actions[phase],
                               ROBOT_SERVO_ACTION_MS) != SERVO_OK) {
            Link_Abort("servo_rejected");
            return;
        }
        s_link_deadline = now + pdMS_TO_TICKS(timeout_ms);
        s_link_issued = 1U;
        return;
    }
    if((int32_t)(now - s_link_deadline) >= 0) {
        Link_Abort(turning ? "turn_timeout" : "servo_timeout");
        return;
    }
    complete = turning ? (step.state == STEPPER_STATE_IDLE && step.position_steps == target)
                       : Link_ServoIdle(&servo);
    if(!complete) {
        s_link_settling = 0U;
    } else if(!s_link_settling) {
        s_link_settle = now;
        s_link_settling = 1U;
    } else if(now - s_link_settle >= pdMS_TO_TICKS(300U)) {
        if(phase == LINK_RETRACT && s_link_home_only) Link_Enter(LINK_TURN_HOME);
        else if(phase == LINK_ARM || phase == LINK_TURN_HOME ||
                (phase == LINK_RETRACT_END && !s_link_return_home)) Link_Enter(LINK_DONE);
        else Link_Enter((link_phase_t)(phase + 1));
    }
}

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
        Link_Update();
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

    LOGF(" servo PWM %lu Hz | type PINS for hardware mapping\r\n",
         (unsigned long)SERVO_PWM_FREQUENCY_HZ);
    if(ROBOT_SERVO_BOOT_TEST) {
        LOGF(" servo BOOT TEST: right A/B/C full range once, then center (~6s)\r\n");
        LOGF("       SERVO HOLD cancels motion; SERVO OFF disables PWM\r\n");
    }
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
    BaseType_t created;

    created = xTaskCreate(Robot_ControlTask, "control", CONTROL_TASK_STACK_WORDS,
                          NULL, CONTROL_TASK_PRIORITY, NULL);
    configASSERT(created == pdPASS);

    created = xTaskCreate(CONSOLE_Task, "console", CONSOLE_TASK_STACK_WORDS,
                          NULL, CONSOLE_TASK_PRIORITY, NULL);
    configASSERT(created == pdPASS);

    created = xTaskCreate(TELEMETRY_Task, "telemetry", TELEMETRY_TASK_STACK_WORDS,
                          NULL, TELEMETRY_TASK_PRIORITY, NULL);
    configASSERT(created == pdPASS);
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
