#ifndef __MPU6050_H
#define __MPU6050_H
//MPU6050.h

#include "stm32f10x.h"


#define MPU6050_ADDR 0x68


void MPU6050_Init(void);
void MPU6050_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az);
void MPU6050_Read_Gyro(int16_t *gx, int16_t *gy, int16_t *gz);


#endif
