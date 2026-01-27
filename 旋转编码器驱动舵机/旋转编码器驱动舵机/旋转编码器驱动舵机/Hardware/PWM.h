#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

// PWM 初始化
void PWM_Init(void);

// 设置 PWM 比较值（CCR）
void PWM_SetCompare(uint16_t Compare);

#endif
