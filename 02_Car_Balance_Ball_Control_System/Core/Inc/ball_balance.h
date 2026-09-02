#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include "stm32f4xx_hal.h"

typedef enum
{
  BALL_STATE_MONITOR = 0,
  BALL_STATE_WAIT_START,
  BALL_STATE_DIRECTION_TEST,
  BALL_STATE_ANGLE_HOLD,
  BALL_STATE_SETTLE,
  BALL_STATE_MOVE_POSITIVE,
  BALL_STATE_MOVE_NEGATIVE,
  BALL_STATE_DONE,
  BALL_STATE_ERROR
} BallState_t;

typedef enum
{
  BALL_ERROR_NONE = 0,
  BALL_ERROR_ANGLE_ADC,
  BALL_ERROR_VISION_TIMEOUT,
  BALL_ERROR_SEQUENCE_TIMEOUT
} BallError_t;

typedef struct
{
  BallState_t state;
  BallError_t error;
  uint32_t elapsed_ms;
  uint16_t angle_adc_raw;
  float angle_adc_filtered;
  float angle_feedback_adc;
  float target_angle_adc;
  float ball_position_mm;
  float ball_velocity_mm_s;
  float position_d_scale;
  float position_angle_limit_adc;
  float requested_position_mm;
  float target_position_mm;
  int32_t stepper_command_hz;
  uint32_t vision_good_frames;
  uint32_t vision_parse_errors;
  uint8_t vision_valid;
  uint8_t angle_valid;
  uint8_t stepper_enabled;
  uint8_t breakaway_active;
} BallDebug_t;

HAL_StatusTypeDef BallBalance_Init(void);
void BallBalance_Step(uint32_t now_ms);
const BallDebug_t *BallBalance_GetDebug(void);

extern volatile BallDebug_t g_ball_debug;
extern volatile float g_ball_position_kp;
extern volatile float g_ball_position_ki;
extern volatile float g_ball_position_kd;
extern volatile float g_ball_breakaway_boost_adc;
extern volatile float g_ball_final_kp;
extern volatile float g_ball_final_ki;
extern volatile float g_ball_final_kd;
extern volatile float g_ball_final_angle_limit_adc;
extern volatile float g_ball_final_hold_angle_limit_adc;
extern volatile float g_ball_final_capture_lookahead_s;
extern volatile float g_ball_angle_kp;
extern volatile float g_ball_angle_ki;
extern volatile float g_ball_angle_kd;

#endif
