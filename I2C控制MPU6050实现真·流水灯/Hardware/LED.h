#ifndef __BSP_LED_H
#define __BSP_LED_H

#include "stm32f10x.h"

#define LED_NUM 8

void LED_Init(void);
void LED_Set(uint8_t index, uint8_t state);
void LED_All_Off(void);
void LED_Breath_Control(uint8_t index, uint8_t brightness);

#endif
