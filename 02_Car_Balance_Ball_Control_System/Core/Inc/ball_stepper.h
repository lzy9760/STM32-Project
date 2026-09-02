#ifndef BALL_STEPPER_H
#define BALL_STEPPER_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef BallStepper_Init(void);
void BallStepper_SetEnabled(uint8_t enabled);
int32_t BallStepper_SetCommand(float command_hz);
void BallStepper_Stop(void);
uint8_t BallStepper_IsEnabled(void);
int32_t BallStepper_GetCommand(void);

#endif
