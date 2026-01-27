#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

uint8_t MPU6050_Init(void);
void MPU6050_ReadData(int16_t *accel, int16_t *gyro);

#endif


