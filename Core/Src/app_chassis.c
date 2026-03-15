#include "app_chassis.h"

#include "app_mode.h"
#include "bsp_chassis_motor.h"
#include "module_chassis.h"
#include "robot_param.h"

void APP_Chassis_ControlStep(const BSP_RemoteState_t *remote_state, float dt_s)
{//处理底盘控制步骤
  AppRuntime_t *runtime;//获取当前应用运行时状态
  BSP_RemoteState_t remote_fallback;//定义遥控器回退状态
  const BSP_RemoteState_t *remote_use;//定义遥控器使用状态指针
  uint8_t control_enable;

  runtime = APP_Mode_Runtime();//获取当前应用运行时状态
  if (runtime == 0)
  {
    return;
  }

  control_enable = APP_Mode_IsControlEnabled();//获取底盘控制使能状态

  BSP_ChassisMotor_SetOutputEnable(control_enable);
  BSP_ChassisMotor_SetCanTxEnable((uint8_t)(control_enable && runtime->can_online && (ROBOT_CAN_CHASSIS_TX_ENABLE != 0U)));//设置底盘电机CAN发送使能状态

  if (remote_state == 0)
  {
    remote_fallback.online = 0U;//如果遥控器状态为空，设置遥控器回退状态为离线
    remote_fallback.ch_norm[0] = 0.0f;//设置遥控器回退状态通道0归一化值为0
    remote_fallback.ch_norm[1] = 0.0f;//设置遥控器回退状态通道1归一化值为0
    remote_fallback.ch_norm[2] = 0.0f;//设置遥控器回退状态通道2归一化值为0
    remote_fallback.ch_norm[3] = 0.0f;//设置遥控器回退状态通道3归一化值为0
    remote_fallback.sw_left = 3U;//设置遥控器回退状态左开关为3（未按下）
    remote_fallback.sw_right = 3U;//设置遥控器回退状态右开关为3（未按下）
    remote_use = &remote_fallback;//使用遥控器回退状态
  }
  else
  {
    remote_use = remote_state;//使用遥控器状态
  }

  Module_Chassis_SetControlGate(&runtime->chassis_ctrl, control_enable, 1U);//设置底盘控制门使能状态
  Module_Chassis_SetMeasuredPowerW(&runtime->chassis_ctrl, BSP_ChassisMotor_GetEstimatedPowerW());//设置底盘测量功率（W）
  Module_Chassis_Update(&runtime->chassis_ctrl, remote_use, &runtime->chassis_out);//更新底盘控制输出
  BSP_ChassisMotor_SetCommand(runtime->chassis_out.vx_cmd_mps,
                              runtime->chassis_out.vy_cmd_mps,
                              runtime->chassis_out.wz_cmd_radps);
  BSP_ChassisMotor_Update(dt_s);//更新底盘电机状态
}
