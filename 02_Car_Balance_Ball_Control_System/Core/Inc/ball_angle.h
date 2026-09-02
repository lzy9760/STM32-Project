#ifndef BALL_ANGLE_H
#define BALL_ANGLE_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef BallAngle_Init(void);
uint8_t BallAngle_Read(uint16_t *raw_adc, float *filtered_adc);

#endif
