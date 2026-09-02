#ifndef OLED_H
#define OLED_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef Oled_Init(I2C_HandleTypeDef *hi2c);
uint8_t Oled_IsPresent(void);
void Oled_WriteText(uint8_t row, uint8_t column, const char *text);

#endif
