#include "MPU6050.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"
//MPU6050.c

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT 0x3B
#define MPU6050_GYRO_XOUT 0x43


void MPU6050_Init(void)
{
// 唤醒MPU6050（关闭睡眠模式）
    I2C_WriteByte(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    // 配置加速度量程：±2g（0x00）
    I2C_WriteByte(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);
    // 配置陀螺仪量程：±250dps（0x00）
    I2C_WriteByte(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);
}


/**
 * @brief  读取MPU6050三轴加速度数据
 * @note   MPU6050的加速度数据为16位有符号数，每个轴占2字节（高8位+低8位），共3轴×2=6字节
 *         寄存器地址：ACCEL_XOUT_H(0x3B)、ACCEL_XOUT_L(0x3C)
 *                     ACCEL_YOUT_H(0x3D)、ACCEL_YOUT_L(0x3E)
 *                     ACCEL_ZOUT_H(0x3F)、ACCEL_ZOUT_L(0x40)
 * @param  ax：指向存储X轴加速度数据的int16_t变量指针（输出参数）
 * @param  ay：指向存储Y轴加速度数据的int16_t变量指针（输出参数）
 * @param  az：指向存储Z轴加速度数据的int16_t变量指针（输出参数）
 * @retval 无
 */
void MPU6050_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    // 定义8位缓冲区：存储从MPU6050读取的6个原始字节数据
    uint8_t buf[6];
    
    // 从MPU6050的ACCEL_XOUT(0x3B)寄存器开始，连续读取6个字节到buf
    // 参数：设备地址(0x68)、起始寄存器地址、接收缓冲区、读取长度
    I2C_ReadBytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT, buf, 6);

    // 16位数据拼接：高8位左移8位 + 低8位（合成完整的16位有符号数）
    // buf[0] = X轴加速度高8位，buf[1] = X轴加速度低8位
    *ax = (buf[0] << 8) | buf[1];
    // buf[2] = Y轴加速度高8位，buf[3] = Y轴加速度低8位
    *ay = (buf[2] << 8) | buf[3];
    // buf[4] = Z轴加速度高8位，buf[5] = Z轴加速度低8位
    *az = (buf[4] << 8) | buf[5];
}

/**
 * @brief  读取MPU6050三轴陀螺仪（角速度）数据
 * @note   MPU6050的陀螺仪数据为16位有符号数，每个轴占2字节（高8位+低8位），共3轴×2=6字节
 *         寄存器地址：GYRO_XOUT_H(0x43)、GYRO_XOUT_L(0x44)
 *                     GYRO_YOUT_H(0x45)、GYRO_YOUT_L(0x46)
 *                     GYRO_ZOUT_H(0x47)、GYRO_ZOUT_L(0x48)
 * @param  gx：指向存储X轴陀螺仪数据的int16_t变量指针（输出参数）
 * @param  gy：指向存储Y轴陀螺仪数据的int16_t变量指针（输出参数）
 * @param  gz：指向存储Z轴陀螺仪数据的int16_t变量指针（输出参数）
 * @retval 无
 */
void MPU6050_Read_Gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    // 定义8位缓冲区：存储从MPU6050读取的6个原始字节数据
    uint8_t buf[6];
    
    // 从MPU6050的GYRO_XOUT(0x43)寄存器开始，连续读取6个字节到buf
    I2C_ReadBytes(MPU6050_ADDR, MPU6050_GYRO_XOUT, buf, 6);

    // 16位数据拼接：高8位左移8位 + 低8位（合成完整的16位有符号数）
    // buf[0] = X轴陀螺仪高8位，buf[1] = X轴陀螺仪低8位
    *gx = (buf[0] << 8) | buf[1];
    // buf[2] = Y轴陀螺仪高8位，buf[3] = Y轴陀螺仪低8位
    *gy = (buf[2] << 8) | buf[3];
    // buf[4] = Z轴陀螺仪高8位，buf[5] = Z轴陀螺仪低8位
    *gz = (buf[4] << 8) | buf[5];
}
