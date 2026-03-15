#include "bsp_imu.h"

#include "bsp_board.h"
#include <string.h>

static uint8_t s_imu_online = 0U;
static BSP_ImuReadFn_t s_imu_read_fn = 0;

void BSP_Imu_Init(void)
{//初始化IMU
  BSP_Board_SetImuCsAccel(1U);
  BSP_Board_SetImuCsGyro(1U);

  /* BMI088 real SPI transactions can be connected by registering read_fn. */
  s_imu_online = 0U;
}

void BSP_Imu_RegisterReadFn(BSP_ImuReadFn_t read_fn)
{//注册IMU读取函数
  s_imu_read_fn = read_fn;
}

HAL_StatusTypeDef BSP_Imu_Update(BSP_ImuSample_t *sample)
{//更新IMU样本
  uint32_t ms;
  int32_t ramp;

  if (sample == 0)
  {//如果样本指针为空，返回HAL_ERROR
    return HAL_ERROR;
  }

  if (s_imu_read_fn != 0)//如果注册了IMU读取函数
  {
    HAL_StatusTypeDef ret;

    ret = s_imu_read_fn(sample);//调用注册的IMU读取函数读取样本
    if (ret == HAL_OK)
    {
      s_imu_online = sample->online ? 1U : 0U;//如果样本指示IMU在线，设置s_imu_online为1U，否则为0U
    }
    else
    {
      s_imu_online = 0U;//如果读取失败，设置s_imu_online为0U
    }

    return ret;
  }

  memset(sample, 0, sizeof(*sample));

  /* If BMI088 interrupt lines are toggling, we regard board-level signal alive. */
  sample->online = (uint8_t)(BSP_Board_IsImuAccelIntActive() || BSP_Board_IsImuGyroIntActive());

  ms = HAL_GetTick();
  ramp = (int32_t)((ms / 8U) % 200U) - 100;//计算 ramp 值，范围为 -100 到 100

  sample->gyro_dps[0] = 0.0f;
  sample->gyro_dps[1] = 0.0f;
  sample->gyro_dps[2] = (float)ramp * 0.01f;//设置陀螺仪样本的 z 轴旋转速率（度/秒）

  sample->accel_mps2[0] = 0.0f;
  sample->accel_mps2[1] = 0.0f;
  sample->accel_mps2[2] = 9.81f;//设置加速度计样本的 z 轴加速度（米/秒^2）

  s_imu_online = sample->online;//根据样本指示IMU是否在线，设置s_imu_online
  return HAL_OK;
}

uint8_t BSP_Imu_IsOnline(void)
{
  return s_imu_online;//返回IMU是否在线状态
}
