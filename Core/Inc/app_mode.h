#ifndef APP_MODE_H
#define APP_MODE_H

#include "bsp_gimbal_motor.h"
#include "module_attitude.h"
#include "module_chassis.h"
#include "module_gimbal.h"
#include <stdint.h>

typedef struct
{
  ModuleAttitude_t attitude;//模块姿态
  ModuleGimbalCtrl_t gimbal_ctrl;//模块云台控制
  ModuleGimbalOutput_t gimbal_out;//模块云台输出
  ModuleChassisCtrl_t chassis_ctrl;//模块底盘控制
  ModuleChassisOut_t chassis_out;//模块底盘输出
  BSP_GimbalFeedback_t gimbal_feedback;//云台反馈

  uint8_t imu_online;//IMU是否在线
  uint8_t vision_online;//视觉是否在线
  uint8_t gimbal_limited;//云台是否受限
  uint8_t force_safe;//是否强制安全模式
  uint8_t can_online;//CAN是否在线
  uint32_t ctrl_loop_overrun;//控制循环溢出次数
} AppRuntime_t;

void APP_Mode_Bringup(void);
AppRuntime_t *APP_Mode_Runtime(void);

uint8_t APP_Mode_IsControlEnabled(void);//获取底盘控制使能状态
void APP_Mode_SetForceSafe(uint8_t force_safe);//设置强制安全模式
void APP_Mode_SetOnlineFlags(uint8_t imu_online, uint8_t vision_online, uint8_t gimbal_limited);//设置在线标志位
void APP_Mode_IncControlOverrun(void);//增加控制循环溢出次数

#endif /* APP_MODE_H */
