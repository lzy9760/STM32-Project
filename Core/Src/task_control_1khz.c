#include "task_control_1khz.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_mode.h"
#include "bsp_gimbal_motor.h"
#include "bsp_imu.h"
#include "bsp_remote.h"
#include "bsp_vision_link.h"
#include "module_attitude.h"
#include "protocol_cfg.h"
#include "task_cfg.h"

#include <string.h>

void Task_Control1kHz_Entry(void)
{
  const float dt_s = 0.001f;
  TickType_t last_wake;//上次任务唤醒时间，单位：tick
  BSP_ImuSample_t imu_sample;//IMU样本，包括加速度、角速度、欧拉角、时间戳等
  BSP_RemoteState_t remote_state;//遥控器状态，包括按键、轴值等
  BSP_VisionTarget_t vision_target;//视觉目标，包括目标位置、角度、大小等

  memset(&imu_sample, 0, sizeof(imu_sample));//将IMU样本结构体imu_sample的所有字节初始化为0
  memset(&remote_state, 0, sizeof(remote_state));//将遥控器状态结构体remote_state的所有字节初始化为0
  memset(&vision_target, 0, sizeof(vision_target));//将视觉目标结构体vision_target的所有字节初始化为0

  last_wake = xTaskGetTickCount();//获取当前任务的tick计数，作为任务的初始唤醒时间

  for (;;)
  {
    AppRuntime_t *runtime;
    TickType_t loop_start;
    TickType_t loop_end;
    uint8_t has_target;

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));//任务延时，等待下一个周期
    loop_start = xTaskGetTickCount();//获取当前任务的tick计数，作为循环开始时间

    runtime = APP_Mode_Runtime();//获取当前运行时环境，包括控制模式、状态、参数等
    if (runtime == 0)//如果运行时环境为空，继续下一个循环
    {
      continue;
    }

    if (BSP_Imu_Update(&imu_sample) == HAL_OK)//如果IMU更新成功
    {
      Module_Attitude_Update(&runtime->attitude, &imu_sample, dt_s);//根据IMU样本更新姿态估计
    }

    BSP_Remote_GetState(&remote_state);//获取遥控器状态
    has_target = BSP_Vision_GetLatestTarget(&vision_target, PROTOCOL_VISION_TARGET_TIMEOUT_MS);//获取最新的视觉目标

    APP_Gimbal_ControlStep(&remote_state, &vision_target, has_target, &imu_sample, dt_s);//根据遥控器状态、视觉目标、IMU样本和时间步长更新云台控制
    APP_Chassis_ControlStep(&remote_state, dt_s);//根据遥控器状态和时间步长更新底盘控制

    APP_Mode_SetOnlineFlags(imu_sample.online, has_target, BSP_GimbalMotor_IsLimited());//根据IMU样本、视觉目标和云台电机是否受限，设置运行时环境的在线标志位

    loop_end = xTaskGetTickCount();//获取当前任务的tick计数，作为循环结束时间
    if ((loop_end - loop_start) > pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS))//如果循环执行时间超过了周期时间，增加控制超运行次数
    {
      APP_Mode_IncControlOverrun();//增加控制超运行次数
    }
  }
}
