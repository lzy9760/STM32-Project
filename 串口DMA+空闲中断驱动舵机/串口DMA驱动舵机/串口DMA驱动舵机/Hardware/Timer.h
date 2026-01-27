#ifndef __TIMER_H
#define __TIMER_H
//Timer.h
#include "stm32f10x.h"

// 定时1秒的标志（主循环判断是否发送ONLINE）
extern u8 Timer1s_Flag;

// 初始化TIM3，配置1秒定时中断
void TIM3_Init(void);

#endif
