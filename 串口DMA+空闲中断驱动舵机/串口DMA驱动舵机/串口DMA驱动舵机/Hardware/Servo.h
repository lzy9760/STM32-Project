#ifndef __SERVO_H
#define __SERVO_H  // 头文件保护，确保宏只定义一次
#include "stm32f10x.h"

// 补充类型定义（如果全局没定义，这里加）
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

// 舵机角度范围（0~180度，宏定义必须写在这里）
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// 函数声明（必须写在这里，供其他文件调用）
void Servo_Init(void);     // 舵机初始化
void Servo_SetAngle(u8 angle); // 设置舵机角度

#endif
