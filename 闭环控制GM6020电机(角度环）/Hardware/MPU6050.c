#include "MPU6050.h"
#include "MyI2C.h"

#define MPU6050_ADDR   0xD0
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B
#define ACCEL_CONFIG   0x1C

uint8_t MPU6050_Init(void)
{
    MyI2C_WriteReg(MPU6050_ADDR, PWR_MGMT_1, 0x00);
    MyI2C_WriteReg(MPU6050_ADDR, ACCEL_CONFIG, 0x00);
    return 1;
}

void MPU6050_ReadData(int16_t *accel, int16_t *gyro)
{
    uint8_t buf[14];

    MyI2C_ReadRegs(MPU6050_ADDR, ACCEL_XOUT_H, buf, 14);

    for (uint8_t i = 0; i < 3; i++)
    {
        accel[i] = (buf[i * 2] << 8) | buf[i * 2 + 1];
        gyro[i]  = (buf[8 + i * 2] << 8) | buf[9 + i * 2];
    }
}


