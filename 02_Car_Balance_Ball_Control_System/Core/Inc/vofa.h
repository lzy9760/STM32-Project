#ifndef VOFA_H
#define VOFA_H

#include "stm32f4xx_hal.h"

void Vofa_Init(UART_HandleTypeDef *huart);
void Vofa_Send4(float channel0, float channel1,
                float channel2, float channel3);
void Vofa_SendWheelSpeed(float left_target, float left_actual,
                         float right_target, float right_actual);

#endif
