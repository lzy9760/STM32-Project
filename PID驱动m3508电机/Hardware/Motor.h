//Motor.h
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

typedef struct
{
    uint16_t angle;        // 机械角度 (0~8191)
    int16_t  speed;        // 转速 rpm
    int16_t  current;      // 实际电流
    uint8_t  temp;  // 温度
} M3508_Info_t;

extern M3508_Info_t Motor1;

void M3508_Parse(uint8_t data[8]);
int16_t OpenLoop_Current_From_Speed(int16_t target_rpm);

#endif

