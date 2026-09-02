#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* All field-tunable values live in this file. */

/* Select the active competition task without changing either task's tuning. */
#define APP_TASK_LINE_FOLLOW             2U
#define APP_TASK_BALL_STATIC             3U
#define APP_ACTIVE_TASK                  APP_TASK_BALL_STATIC

/* PA0 ... PA7 must be wired from the left-most to the right-most sensor. */
#define LINE_SENSOR_COUNT              8U
#define LINE_WHITE_DEFAULT             {3153U, 3093U, 2448U, 2273U, 3177U, 2630U, 2538U, 2799U}
#define LINE_BLACK_DEFAULT             {1191U, 1080U, 1067U, 1055U, 1489U, 1080U, 1117U,  993U}
#define LINE_SENSOR_FILTER_SHIFT       1U
#define LINE_SENSOR_MIN_SPAN           200
#define LINE_SENSOR_DETECT_LEVEL       400U
#define LINE_ERROR_DEADBAND             100
#define LINE_CROSS_SENSOR_LEVEL        500U
#define LINE_CROSS_MIN_SENSORS           3U

/* 10 ms control period. Speed units are encoder counts per control period. */
#define APP_CONTROL_PERIOD_MS          10U
#define APP_SPEED_PI_BENCH_ENABLE      0U
#define LINE_BASE_SPEED                34
#define LINE_START_SPEED                8
#define LINE_START_RAMP_MS             800U
#define LINE_APPROACH_SPEED            5
#define LINE_MAX_TARGET_SPEED          40
#define LINE_KP_NUM                    4
#define LINE_KD_NUM                    1
#define LINE_GAIN_DEN                  1000
#define LINE_STEERING_SIGN             -1

/* Wheel-speed PI output is signed motor duty in permille. */
#define SPEED_PI_KP                    20.0f
#define SPEED_PI_KI                    1.0f
#define SPEED_PI_INTEGRAL_LIMIT        250.0f
#define SPEED_FILTER_ALPHA             0.50f
#define MOTOR_DUTY_MAX                 1000

/* Change a sign if a positive target makes that side run backwards. */
#define CHASSIS_LEFT_MOTOR_SIGN        1
#define CHASSIS_RIGHT_MOTOR_SIGN       1
#define CHASSIS_LEFT_ENCODER_SIGN      -1
#define CHASSIS_RIGHT_ENCODER_SIGN     1

/* Finish-line sensors and start key use pull-ups and are active low. */
#define APP_AUTO_START_ENABLE          1U
#define APP_AUTO_START_DELAY_MS        3000U
#define APP_INPUT_ACTIVE_STATE         GPIO_PIN_RESET
#define APP_INPUT_DEBOUNCE_TICKS       3U
#define APP_LEAVE_LINE_TICKS           10U
#define APP_FINISH_LINE_TICKS          1U
#define APP_FINISH_ARM_TIME_MS         3000U
#define APP_LINE_LOST_STOP_MS          500U

/* Encoder counts to travel after the front finish sensors first see A line. */
#define APP_STOP_ADVANCE_COUNTS        250

#endif
