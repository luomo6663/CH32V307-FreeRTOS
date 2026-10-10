#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/*
 * Single place for every tunable in the firmware.
 *
 * Anything that depends on wiring, mechanics or tuning lives here so the
 * control modules below stay free of magic numbers.
 */

/* ---------------- firmware / RTOS ---------------- */

#define ROBOT_FIRMWARE_VERSION              "1.0.0"

#define ROBOT_CONSOLE_BAUDRATE              115200U
#define ROBOT_CONSOLE_POLL_MS               5U

#define ROBOT_CONTROL_PERIOD_MS             10U
#define ROBOT_CONTROL_FREQUENCY_HZ          (1000U / ROBOT_CONTROL_PERIOD_MS)

#define ROBOT_TELEMETRY_PERIOD_MS           100U

/* ---------------- motors ---------------- */

#define ROBOT_MOTOR_PWM_FREQUENCY_HZ        24000U
#define ROBOT_MOTOR_DUTY_MAX                1000

/* Duty counts per control period used to slew-limit the bridge. 0 = instant. */
#define ROBOT_MOTOR_DEFAULT_SLEW           0

/* Direction calibration: 1 swaps the meaning of positive/negative duty. */
#define ROBOT_MOTOR0_INVERT                 0
#define ROBOT_MOTOR1_INVERT                 1
#define ROBOT_MOTOR2_INVERT                 0
#define ROBOT_MOTOR3_INVERT                 1

/* ---------------- encoders ---------------- */

/*
 * Wheel 1 sits on an 11 PPR magnetic encoder through the 32:1 gearbox.
 * Verify these against the real hardware before trusting odometry.
 */
#define ROBOT_ENCODER_PPR                   11U
#define ROBOT_ENCODER_GEAR_RATIO            32U

/* Flip to -1 when a positive motor command makes the encoder count decrease. */
#define ROBOT_ENCODER0_SIGN                 -1
#define ROBOT_ENCODER1_SIGN                 -1
#define ROBOT_ENCODER2_SIGN                 -1
#define ROBOT_ENCODER3_SIGN                 -1

/* Latch a flag when an encoder pin changes level. Maintenance diagnostic. */
#define ROBOT_ENCODER_LEVEL_WATCH           1

/* Timer input-capture digital filter, 0 = off, 0x0F = maximum filtering. */
#define ROBOT_ENCODER_INPUT_FILTER          0x0FU

/* ---------------- chassis geometry ---------------- */

#define ROBOT_WHEEL_RADIUS_MM               32.5f
#define ROBOT_HALF_LENGTH_MM                70.0f
#define ROBOT_HALF_WIDTH_MM                 75.0f

#define ROBOT_MAX_WHEEL_RPM                 300.0f
#define ROBOT_MAX_LINEAR_MM_S               500.0f
#define ROBOT_MAX_YAW_RATE_RAD_S            3.0f

/* Twist commands older than this stop the chassis while in DRIVE mode. */
#define ROBOT_COMMAND_TIMEOUT_MS            1000U

/* SPD commands older than this stop all wheels while in SPEED mode. */
#define ROBOT_SPEED_TEST_TIMEOUT_MS         5000U

/* ---------------- per-wheel speed loop ---------------- */

#define ROBOT_PID_KP_W1                     8.0f
#define ROBOT_PID_KI_W1                     3.0f
#define ROBOT_PID_KD_W1                     0.0f

#define ROBOT_PID_KP_W2                     8.0f
#define ROBOT_PID_KI_W2                     4.0f
#define ROBOT_PID_KD_W2                     0.0f

#define ROBOT_PID_KP_W3                     6.0f
#define ROBOT_PID_KI_W3                     2.0f
#define ROBOT_PID_KD_W3                     0.0f

#define ROBOT_PID_KP_W4                     8.0f
#define ROBOT_PID_KI_W4                     8.0f
#define ROBOT_PID_KD_W4                     0.0f

#define ROBOT_PID_DERIV_FILTER              0.2f
#define ROBOT_PID_OUTPUT_MIN                (-1000.0f)
#define ROBOT_PID_OUTPUT_MAX                (1000.0f)

#define ROBOT_PID_STALL_RPM                 3.0f
#define ROBOT_PID_STALL_LIMIT               50U
#define ROBOT_PID_DIR_FAULT_LIMIT           10U
#define ROBOT_PID_MEASURE_MAX_RPM           400.0f
#define ROBOT_PID_MEASURE_FILTER            0.5f
#define ROBOT_PID_MIN_TARGET_RPM            1.0f

/*
 * Open-loop fallback for a wheel whose quadrature input is missing. All four
 * wheels are quadrature today, so this only matters after a channel breaks:
 * with the gain at 0 the chassis reports NO_FEEDBACK instead of driving blind.
 */
#define ROBOT_WHEEL3_OPEN_LOOP_GAIN         0.0f

/* ---------------- servo ---------------- */

/*
 * PB5 = TIM3_CH2 after the TIM3 partial remap.
 *
 * TBS-K20 (Hangzhou Zhongling) is a PWM position servo:
 *   5-8.4 V, 50-330 Hz, 500-2500 us pulse, 1500 us center, 270 degrees.
 * The manual sweep stays inside a conservative 1000-2000 us range.
 */
#define ROBOT_SERVO_PWM_FREQUENCY_HZ        50U
#define ROBOT_SERVO_SWEEP_MIN_US             1000U
#define ROBOT_SERVO_SWEEP_MAX_US             2000U
#define ROBOT_SERVO_SWEEP_PERIOD_MS          2000U

/* ---------------- stepper turntable ---------------- */

/*
 * Single STEP/DIR driver on fixed pins:
 *   PA0 -> STEP (TIM5_CH1)
 *   PA1 -> DIR
 *
 * STEPS_PER_REV is the number of driver pulses for one full turntable
 * revolution. It already includes microstepping and any gear ratio:
 *
 *   steps_per_rev = motor_full_steps_per_rev *
 *                   driver_microsteps *
 *                   turntable_gear_ratio
 *
 * Example: STH-42C2009-01 is a 0.9 degree motor, so it has 400 full
 * steps/rev. With a TMC2209 set to 1/8 microstep, use 400 * 8 = 3200.
 * With 1/16 microstep, use 400 * 16 = 6400.
 */
#define ROBOT_STEPPER_STEPS_PER_REV          3200U
#define ROBOT_STEPPER_DEFAULT_RPM            5.0f
#define ROBOT_STEPPER_MAX_RPM                30.0f
#define ROBOT_STEPPER_MIN_RPM                0.1f
#define ROBOT_STEPPER_MAX_RATE_HZ            20000U
#define ROBOT_STEPPER_PULSE_WIDTH_US         3U
#define ROBOT_STEPPER_DIR_INVERT             0U
#define ROBOT_STEPPER_DIR_SETUP_US           5U
#define ROBOT_STEPPER_START_RATE_HZ          20U
#define ROBOT_STEPPER_ACCEL_HZ_PER_S         500U
#define ROBOT_STEPPER_DECEL_HZ_PER_S         500U

#endif /* __ROBOT_CONFIG_H */
