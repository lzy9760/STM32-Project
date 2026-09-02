#ifndef BALL_VISION_H
#define BALL_VISION_H

#include "stm32f4xx_hal.h"

typedef struct
{
  float position_mm;
  float velocity_mm_s;
  uint32_t last_frame_ms;
  uint32_t good_frames;
  uint32_t parse_errors;
  uint16_t dropped_bytes;
  uint8_t valid;
} BallVisionData_t;

HAL_StatusTypeDef BallVision_Init(void);
void BallVision_Update(uint32_t now_ms);
const BallVisionData_t *BallVision_GetData(void);
uint8_t BallVision_IsFresh(uint32_t now_ms);

#endif
