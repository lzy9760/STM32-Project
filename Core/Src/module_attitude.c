#include "module_attitude.h"

#include <math.h>

#define MODULE_RAD2DEG 57.2957795f//弧度转角度的系数

static float Module_Attitude_WrapDeg(float deg)
{
  while (deg > 180.0f)//将角度归一化到[-180, 180]范围内
  {
    deg -= 360.0f;
  }

  while (deg < -180.0f)
  {
    deg += 360.0f;
  }

  return deg;
}

void Module_Attitude_Init(ModuleAttitude_t *attitude)
{//初始化姿态模块
  if (attitude == 0)
  {
    return;
  }

  attitude->roll_deg = 0.0f;//
  attitude->pitch_deg = 0.0f;
  attitude->yaw_deg = 0.0f;
  attitude->roll_rate_dps = 0.0f;
  attitude->pitch_rate_dps = 0.0f;
  attitude->yaw_rate_dps = 0.0f;
}

void Module_Attitude_Update(ModuleAttitude_t *attitude,
                            const BSP_ImuSample_t *imu,
                            float dt_s)
{//更新姿态模块，根据IMU数据和时间步长dt_s更新姿态信息
  const float alpha = 0.98f;//互补滤波器系数，用于融合加速度计和陀螺仪数据
  float accel_roll;
  float accel_pitch;
  float ay_az_norm;

  if (attitude == 0 || imu == 0 || dt_s <= 0.0f)
  {
    return;
  }

  attitude->roll_rate_dps = imu->gyro_dps[0];//更新横滚角速度（度/秒）
  attitude->pitch_rate_dps = imu->gyro_dps[1];//更新俯仰角速度（度/秒）
  attitude->yaw_rate_dps = imu->gyro_dps[2];//更新偏航角速度（度/秒）

  //角度积分更新，角度=角速度*时间步长
  attitude->roll_deg += attitude->roll_rate_dps * dt_s;//更新横滚角度（度）
  attitude->pitch_deg += attitude->pitch_rate_dps * dt_s;//更新俯仰角度（度）
  attitude->yaw_deg += attitude->yaw_rate_dps * dt_s;//更新偏航角度（度）
  //计算Y-Z平面的加速度范数ay_az_norm
  ay_az_norm = sqrtf(imu->accel_mps2[1] * imu->accel_mps2[1] +
                     imu->accel_mps2[2] * imu->accel_mps2[2]);//计算ay_az_norm（加速度在y轴和z轴方向的范数）

  accel_roll = atan2f(imu->accel_mps2[1], imu->accel_mps2[2]) * MODULE_RAD2DEG;//计算横滚角度（度）
  accel_pitch = atan2f(-imu->accel_mps2[0], ay_az_norm) * MODULE_RAD2DEG;//计算俯仰角度（度）
  
  attitude->roll_deg = alpha * attitude->roll_deg + (1.0f - alpha) * accel_roll;//使用互补滤波器融合加速度计和陀螺仪数据，更新横滚角度（度）
  attitude->pitch_deg = alpha * attitude->pitch_deg + (1.0f - alpha) * accel_pitch;//使用互补滤波器融合加速度计和陀螺仪数据，更新俯仰角度（度）
  attitude->yaw_deg = Module_Attitude_WrapDeg(attitude->yaw_deg);//将偏航角度归一化到[-180, 180]范围内
}
