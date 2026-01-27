#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include <stdio.h>
//Serial.h

// 串口接收缓冲区（最大64字节）
#define USART1_RX_BUF_SIZE 64
extern u8 USART1_RX_BUF[USART1_RX_BUF_SIZE];
extern u16 USART1_RX_LEN;  // 实际接收长度
extern u8 USART1_RX_Flag;  // 接收完成标志
extern u16 USART1_RX_Count; // 接收数据次数

// 串口+DMA初始化（波特率115200）
void Serial_Init(void);

// 串口打印函数（类似printf）
void Serial_Printf(const char* fmt, ...);

#endif
