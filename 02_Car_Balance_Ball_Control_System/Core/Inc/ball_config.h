#ifndef BALL_CONFIG_H
#define BALL_CONFIG_H

#include "stm32f4xx_hal.h"

/* Debug order: monitor -> direction test -> angle loop -> cascade. */
#define BALL_MODE_SENSOR_MONITOR          0U
#define BALL_MODE_STEPPER_DIRECTION_TEST  1U
#define BALL_MODE_ANGLE_LOOP              2U
#define BALL_MODE_CASCADE                 3U
#define BALL_CONTROL_MODE                BALL_MODE_SENSOR_MONITOR

/* WDD35D4 angle sensor: terminal 2 -> PB0/ADC2_IN8, terminals 1/3 -> 3.3 V/GND. */
#define BALL_ANGLE_ADC_ZERO               2740.0f
#define BALL_ANGLE_SENSOR_SIGN            (+1.0f)
#define BALL_ANGLE_FILTER_ALPHA           0.15f
#define BALL_ANGLE_ADC_VALID_MIN          80U
#define BALL_ANGLE_ADC_VALID_MAX          4015U

/* D36A: STEP=PC6(TIM3_CH1), DIR=PC8, EN=PB14; DIP 1/2/3 = OFF/OFF/OFF (16 microsteps). */
#define BALL_STEPPER_ENABLE_ACTIVE_LEVEL  GPIO_PIN_SET
#define BALL_STEPPER_POSITIVE_DIR_LEVEL   GPIO_PIN_SET
#define BALL_STEPPER_MAX_HZ               1500.0f
#define BALL_STEPPER_MIN_HZ                 20.0f
#define BALL_STEPPER_DEADBAND_HZ             5.0f
#define BALL_STEPPER_REVERSE_CONFIRM_CYCLES 3U
#define BALL_STEPPER_TEST_HZ              200.0f
#define BALL_STEPPER_TEST_MS              250U

/* MaixCAM TX -> PD2/UART5_RX, 3.3 V logic, common GND. */
#define BALL_VISION_BAUD                  115200U
#define BALL_VISION_TIMEOUT_MS            150U
#define BALL_VISION_POSITION_LIMIT_X10    1250
#define BALL_VISION_VELOCITY_ALPHA        0.30f

/* 100 Hz angle loop and 50 Hz position loop. */
#define BALL_CONTROL_PERIOD_MS            10U
#define BALL_POSITION_LOOP_DIVIDER        2U
#define BALL_AUTO_START_DELAY_MS          3000U
#define BALL_START_POSITION_TOLERANCE_MM  10.0f

/* Item 3 trajectory: O -> -5 cm -> +5 cm, then keep holding +5 cm. */
#define BALL_POSITIVE_TARGET_MM           50.0f
#define BALL_NEGATIVE_TARGET_MM           -50.0f
#define BALL_FIRST_BRAKE_LOOKAHEAD_S      1.1f
#define BALL_TARGET_SLEW_MM_S             250.0f
#define BALL_WAYPOINT_TOLERANCE_MM         5.0f
#define BALL_WAYPOINT_SPEED_MM_S          30.0f
#define BALL_WAYPOINT_HOLD_MS             150U
#define BALL_FINAL_TOLERANCE_MM             5.0f
#define BALL_FINAL_SPEED_MM_S              20.0f
#define BALL_FINAL_HOLD_MS                300U
#define BALL_SEQUENCE_TIMEOUT_MS         5000U

/* Independent soft capture for the +5 cm endpoint. */
#define BALL_FINAL_CAPTURE_ENTRY_MM        40.0f
#define BALL_FINAL_CAPTURE_LOOKAHEAD_S      0.05f
#define BALL_FINAL_KP_DEFAULT               1.4f
#define BALL_FINAL_KI_DEFAULT               0.0f
#define BALL_FINAL_KD_DEFAULT               0.50f
#define BALL_FINAL_ANGLE_LIMIT_ADC         35.0f
#define BALL_FINAL_HOLD_ANGLE_LIMIT_ADC    20.0f

/* Initial values copied from the reference cascade; tune on this mechanism. */
#define BALL_POSITION_KP_DEFAULT           4.5f
#define BALL_POSITION_KI_DEFAULT           0.2f
#define BALL_POSITION_KD_DEFAULT           0.2f
#define BALL_POSITION_CONTROL_SIGN        (-1.0f)
#define BALL_POSITION_D_FILTER_HZ          2.0f
#define BALL_POSITION_I_LIMIT            500.0f
#define BALL_ANGLE_TARGET_LIMIT_ADC       50.0f

/* When the ball is travelling toward its target, keep velocity damping out
   of the long transfer and fade it in only near the braking point. */
#define BALL_POSITION_D_FULL_ERROR_MM      6.0f
#define BALL_POSITION_D_OFF_ERROR_MM      20.0f

/* Extra tilt used only when position error exists but the ball is nearly
   stationary.  It helps the ball cross small surface bumps without making
   the whole trajectory permanently aggressive. */
#define BALL_BREAKAWAY_ERROR_MM             8.0f
#define BALL_BREAKAWAY_SPEED_MM_S          12.0f
#define BALL_BREAKAWAY_BOOST_ADC_DEFAULT   20.0f
#define BALL_BREAKAWAY_ANGLE_LIMIT_ADC     75.0f

#define BALL_ANGLE_KP_DEFAULT              3.0f
#define BALL_ANGLE_KI_DEFAULT              0.0f
#define BALL_ANGLE_KD_DEFAULT              0.0f
#define BALL_ANGLE_I_LIMIT               500.0f

#endif
