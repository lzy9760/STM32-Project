#include "app_gimbal.h"

#include "app_mode.h"
#include "bsp_gimbal_motor.h"
#include "module_gimbal.h"
#include "robot_param.h"

#include <string.h>

void APP_Gimbal_ControlStep(const BSP_RemoteState_t *remote_state,//处理遥控器状态
                            const BSP_VisionTarget_t *vision_target,//处理视觉目标
                            uint8_t has_target,//是否有目标
                            const BSP_ImuSample_t *imu_sample,//处理IMU样本
                            float dt_s)//处理时间间隔
{//处理云台控制步骤
  AppRuntime_t *runtime;
  ModuleGimbalInput_t input;
  uint8_t control_enable;
  uint8_t gimbal_enable;

  runtime = APP_Mode_Runtime();//获取系统运行时状态指针
  if (runtime == 0)//如果系统运行时状态指针为空
  {
    return;
  }

  control_enable = APP_Mode_IsControlEnabled();//获取控制使能状态
  BSP_GimbalMotor_SetCanTxEnable((uint8_t)(control_enable && runtime->can_online && (ROBOT_CAN_GIMBAL_TX_ENABLE != 0U)));//设置CAN发送使能状态

  BSP_GimbalMotor_GetFeedback(&runtime->gimbal_feedback);//获取云台反馈

  memset(&input, 0, sizeof(input));//清空输入结构体

  if (remote_state != 0 && remote_state->online != 0U)//如果遥控器状态有效
  {
    gimbal_enable = (remote_state->sw_right != 3U) ? 1U : 0U;
  }
  else
  {
    gimbal_enable = 1U;
  }

  input.enable = (uint8_t)(gimbal_enable && control_enable);
  input.target_valid = (uint8_t)(has_target && (vision_target != 0) && (vision_target->target_valid != 0U));

  if (vision_target != 0 && input.target_valid != 0U)//如果视觉目标有效
  {
    input.vision_yaw_err_deg = vision_target->yaw_err_deg;//设置视觉目标yaw轴误差
    input.vision_pitch_err_deg = vision_target->pitch_err_deg;//设置视觉目标pitch轴误差
  }

  input.yaw_deg = runtime->gimbal_feedback.yaw_deg;//设置云台角度（yaw轴）
  input.pitch_deg = runtime->gimbal_feedback.pitch_deg;//设置云台角度（pitch轴）

  if (imu_sample != 0 && imu_sample->online != 0U)//如果IMU样本有效
  {
    input.yaw_rate_dps = imu_sample->gyro_dps[2];//设置云台角速度（yaw轴）
    input.pitch_rate_dps = imu_sample->gyro_dps[1];//设置云台角速度（pitch轴）
  }
  else
  {
    input.yaw_rate_dps = runtime->gimbal_feedback.yaw_rate_dps;
    input.pitch_rate_dps = runtime->gimbal_feedback.pitch_rate_dps;
  }

  input.body_roll_deg = runtime->attitude.roll_deg;

  Module_Gimbal_Update(&runtime->gimbal_ctrl, &input, dt_s, &runtime->gimbal_out);//更新云台控制输出
  BSP_GimbalMotor_CommandCurrent(runtime->gimbal_out.yaw_current_cmd,
                                 runtime->gimbal_out.pitch_current_cmd);//发送电流命令
  BSP_GimbalMotor_Update(dt_s);//更新云台电机状态
}
