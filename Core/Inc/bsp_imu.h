#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "main.h"
#include <stdint.h>

typedef struct
{
  float gyro_dps[3];//陀螺仪数据（度/秒）
  float accel_mps2[3];//加速度计数据（米/秒^2）
  uint8_t online;//IMU是否在线（1：在线，0：离线）
} BSP_ImuSample_t;

typedef HAL_StatusTypeDef (*BSP_ImuReadFn_t)(BSP_ImuSample_t *sample);
//初始化IMU
void BSP_Imu_Init(void);
void BSP_Imu_RegisterReadFn(BSP_ImuReadFn_t read_fn);
HAL_StatusTypeDef BSP_Imu_Update(BSP_ImuSample_t *sample);
uint8_t BSP_Imu_IsOnline(void);

#endif /* BSP_IMU_H */
