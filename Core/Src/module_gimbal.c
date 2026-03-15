#include "module_gimbal.h"//云台控制模块，包含云台的初始化、更新状态、设置目标角度、设置模式等函数

static float Module_Gimbal_Clamp(float value, float min_value, float max_value)
{//将value限制在[min_value, max_value]范围内
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static float Module_Gimbal_WrapDeg(float deg)
{//将角度归一化到[-180, 180]范围内
  while (deg > 180.0f)
  {
    deg -= 360.0f;
  }

  while (deg < -180.0f)
  {
    deg += 360.0f;
  }

  return deg;
}

void Module_Gimbal_Init(ModuleGimbalCtrl_t *ctrl)
{//初始化云台控制模块，设置PID参数和模式
  if (ctrl == 0)
  {
    return;
  }

  Module_PID_Init(&ctrl->yaw_angle_pid, 8.0f, 0.0f, 0.05f, 220.0f, 100.0f);
  Module_PID_Init(&ctrl->yaw_rate_pid, 0.22f, 0.03f, 0.0f, 8.0f, 6.0f);
  Module_PID_Init(&ctrl->pitch_angle_pid, 9.0f, 0.0f, 0.06f, 220.0f, 100.0f);
  Module_PID_Init(&ctrl->pitch_rate_pid, 0.24f, 0.03f, 0.0f, 8.0f, 6.0f);

  ctrl->mode = MODULE_GIMBAL_MODE_SAFE;
  ctrl->yaw_target_deg = 0.0f;
  ctrl->pitch_target_deg = 0.0f;
  ctrl->patrol_rate_dps = 35.0f;
  ctrl->patrol_dir = 1.0f;
}

void Module_Gimbal_Update(ModuleGimbalCtrl_t *ctrl,
                          const ModuleGimbalInput_t *input,
                          float dt_s,
                          ModuleGimbalOutput_t *output)
{//更新云台控制模块状态，根据输入和时间步长计算输出
  float yaw_error;
  float pitch_error;
  float yaw_rate_sp;
  float pitch_rate_sp;

  if (ctrl == 0 || input == 0 || output == 0 || dt_s <= 0.0f)
  {
    return;
  }

  if (input->enable == 0U)
  {
    ctrl->mode = MODULE_GIMBAL_MODE_SAFE;
    ctrl->yaw_target_deg = input->yaw_deg;
    ctrl->pitch_target_deg = input->pitch_deg;

    Module_PID_Reset(&ctrl->yaw_angle_pid);//重置横滚角度PID控制器
    Module_PID_Reset(&ctrl->yaw_rate_pid);//重置横滚角速度PID控制器
    Module_PID_Reset(&ctrl->pitch_angle_pid);//重置俯仰角度PID控制器
    Module_PID_Reset(&ctrl->pitch_rate_pid);//重置俯仰角速度PID控制器

    output->yaw_current_cmd = 0.0f;//设置横滚角度指令为0
    output->pitch_current_cmd = 0.0f;//设置俯仰角度指令为0
    output->yaw_target_deg = ctrl->yaw_target_deg;//设置横滚目标角度为当前目标角度
    output->pitch_target_deg = ctrl->pitch_target_deg;//设置俯仰目标角度为当前目标角度
    output->mode = ctrl->mode;//设置云台模式为安全模式
    return;
  }

  if (input->target_valid)//如果目标有效，target_valid为1
  {
    ctrl->mode = MODULE_GIMBAL_MODE_AUTO_AIM;//设置云台模式为自动瞄准模式
    ctrl->yaw_target_deg += input->vision_yaw_err_deg * 0.45f;//根据视觉目标yaw轴误差更新目标角度
    ctrl->pitch_target_deg += input->vision_pitch_err_deg * 0.45f;//根据视觉目标pitch轴误差更新目标角度
  }//0.45f是增益系数，用于调整视觉目标误差对目标角度的影响
  else//target_valid为0
  {
    ctrl->mode = MODULE_GIMBAL_MODE_PATROL;//设置云台模式为巡逻模式
    ctrl->yaw_target_deg += ctrl->patrol_rate_dps * ctrl->patrol_dir * dt_s;
  //巡逻边界检测
    if (ctrl->yaw_target_deg > 135.0f)
    {
      ctrl->yaw_target_deg = 135.0f;
      ctrl->patrol_dir = -1.0f;
    }
    else if (ctrl->yaw_target_deg < -135.0f)
    {
      ctrl->yaw_target_deg = -135.0f;
      ctrl->patrol_dir = 1.0f;
    }

    ctrl->pitch_target_deg = -4.0f;
  }

  /* Compensate pitch slightly when body is rolled. */
  //Pitch轴角度补偿，根据横滚角度调整俯仰目标角度，当机体倾斜时，补偿Pitch角度
  ctrl->pitch_target_deg -= input->body_roll_deg * 0.03f;
  //限制目标角度
  ctrl->yaw_target_deg = Module_Gimbal_Clamp(ctrl->yaw_target_deg, -145.0f, 145.0f);
  ctrl->pitch_target_deg = Module_Gimbal_Clamp(ctrl->pitch_target_deg, -20.0f, 30.0f);
  //计算角度误差
  yaw_error = Module_Gimbal_WrapDeg(ctrl->yaw_target_deg - input->yaw_deg);
  pitch_error = ctrl->pitch_target_deg - input->pitch_deg;
  //计算角速度设定值（角度PID）
  yaw_rate_sp = Module_PID_Update(&ctrl->yaw_angle_pid, yaw_error, dt_s);
  pitch_rate_sp = Module_PID_Update(&ctrl->pitch_angle_pid, pitch_error, dt_s);
  //计算电流指令（角速度PID）
  output->yaw_current_cmd = Module_PID_Update(&ctrl->yaw_rate_pid,
                                              yaw_rate_sp - input->yaw_rate_dps,
                                              dt_s);
  output->pitch_current_cmd = Module_PID_Update(&ctrl->pitch_rate_pid,
                                                pitch_rate_sp - input->pitch_rate_dps,
                                                dt_s);
  //设置输出目标角度和模式
  output->yaw_target_deg = ctrl->yaw_target_deg;
  output->pitch_target_deg = ctrl->pitch_target_deg;
  output->mode = ctrl->mode;
}
