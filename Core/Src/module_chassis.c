#include "module_chassis.h"

#define MODULE_CHASSIS_POWER_BUFFER_MAX_J 60.0f
#define MODULE_CHASSIS_POWER_CUT_TIME_S   5.0f

static float Module_Chassis_Abs(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float Module_Chassis_Clamp(float value, float min_value, float max_value)
{//限制值在指定范围内
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

static float Module_Chassis_ApplyDeadband(float value, float deadband)
{//应用死区
  if (Module_Chassis_Abs(value) < deadband)
  {
    return 0.0f;
  }

  return value;
}

static float Module_Chassis_SelectPowerLimit(ModuleChassisRulePreset_t preset,
                                             ModuleChassisPowerMode_t mode)
{//根据规则预设和功率模式选择功率限制
  if (preset == MODULE_CHASSIS_RULE_INFANTRY_DUEL)//如果规则预设为步兵对打
  {
    return 120.0f;
  }

  if (mode == MODULE_CHASSIS_POWER_MODE_POWER_FIRST)//如果功率模式为功率优先
  {
    return 90.0f;
  }

  return 75.0f;
}

static float Module_Chassis_EstimatePower(const ModuleChassisCtrl_t *ctrl,
                                          float vx_mps,
                                          float vy_mps,
                                          float wz_radps)
{//计算底盘电机功率（W）
  float translational;
  float rotational;

  //平移功率=（|vx|+|vy|）*功率代理增益v
  translational = (Module_Chassis_Abs(vx_mps) + Module_Chassis_Abs(vy_mps)) * ctrl->power_proxy_gain_v;//计算平移功率（W）
  //旋转功率=|wz|*功率代理增益wz
  rotational = Module_Chassis_Abs(wz_radps) * ctrl->power_proxy_gain_wz;//计算旋转功率（W）

  return translational + rotational;
}

static void Module_Chassis_UpdatePowerScale(ModuleChassisCtrl_t *ctrl)
{//更新功率缩放比例
  float ratio;

  if (ctrl->power_cut_left_s > 0.0f || ctrl->control_enabled == 0U || ctrl->robot_online == 0U)
  {//如果功率缓冲区为空或控制未启用或机器人不在线，则功率缩放比例为0
    ctrl->power_scale = 0.0f;
    return;
  }

  if (ctrl->power_buffer_max_j < 1e-3f)
  {//如果功率缓冲区最大功率为0，则功率缩放比例为1
    ctrl->power_scale = 1.0f;
    return;
  }

  ratio = ctrl->power_buffer_j / ctrl->power_buffer_max_j;
  ratio = Module_Chassis_Clamp(ratio, 0.0f, 1.0f);

  if (ratio >= 0.15f)
  {//如果功率缓冲区最大功率大于等于0.15，则功率缩放比例为1
    ctrl->power_scale = 1.0f;
  }
  else
  {//否则，功率缩放比例为功率缓冲区最大功率除以0.15
    ctrl->power_scale = ratio / 0.15f;
  }
}

void Module_Chassis_Init(ModuleChassisCtrl_t *ctrl)
{//初始化底盘控制结构体
  if (ctrl == 0)
  {
    return;
  }

  ctrl->max_v_mps = 1.8f;//最大 translational 速度（m/s）
  ctrl->max_wz_radps = 5.0f;//最大 rotational 速度（rad/s）
  ctrl->deadband = 0.03f;//死区
  ctrl->ctrl_dt_s = 0.001f;//控制时间步长（s）

  ctrl->rule_preset = MODULE_CHASSIS_RULE_3V3_INFANTRY;//规则预设
  ctrl->power_mode = MODULE_CHASSIS_POWER_MODE_HP_FIRST;//功率模式

  ctrl->power_limit_w = 75.0f;//功率限制（W）
  ctrl->power_buffer_max_j = MODULE_CHASSIS_POWER_BUFFER_MAX_J;//功率缓冲区最大功率（J）
  ctrl->power_buffer_j = MODULE_CHASSIS_POWER_BUFFER_MAX_J;
  ctrl->power_guard_period_s = 0.1f;//功率保护周期（s）
  ctrl->power_guard_acc_s = 0.0f;//功率保护加速度（s）
  ctrl->power_cut_left_s = 0.0f;//功率保护剩余时间（s）
  ctrl->power_scale = 1.0f;//功率缩放比例

  ctrl->power_proxy_gain_v = 35.0f;//功率代理增益v（W/m/s）
  ctrl->power_proxy_gain_wz = 8.0f;//功率代理增益wz（W/rad/s）
  ctrl->external_power_w = 0.0f;//外部功率（W）
  ctrl->external_power_valid = 0U;//外部功率是否有效（0：无效，1：有效）

  ctrl->control_enabled = 1U;//控制是否启用（0：禁用，1：启用）
  ctrl->robot_online = 1U;//机器人是否在线（0：不在线，1：在线）
}

void Module_Chassis_SetRulePreset(ModuleChassisCtrl_t *ctrl, ModuleChassisRulePreset_t preset)
{//设置底盘规则预设
  if (ctrl == 0)
  {
    return;
  }

  ctrl->rule_preset = preset;
  if (preset == MODULE_CHASSIS_RULE_INFANTRY_DUEL)
  {
    ctrl->power_mode = MODULE_CHASSIS_POWER_MODE_FIXED;
  }
  else if (ctrl->power_mode == MODULE_CHASSIS_POWER_MODE_FIXED)
  {
    ctrl->power_mode = MODULE_CHASSIS_POWER_MODE_HP_FIRST;//如果规则预设为步兵对打，则功率模式为生命值优先
  }

  ctrl->power_limit_w = Module_Chassis_SelectPowerLimit(ctrl->rule_preset, ctrl->power_mode);
  ctrl->power_buffer_j = ctrl->power_buffer_max_j;
  ctrl->power_cut_left_s = 0.0f;
  Module_Chassis_UpdatePowerScale(ctrl);
}

void Module_Chassis_SetPowerMode(ModuleChassisCtrl_t *ctrl, ModuleChassisPowerMode_t mode)
{//设置底盘功率模式
  if (ctrl == 0)
  {
    return;
  }

  if (ctrl->rule_preset == MODULE_CHASSIS_RULE_INFANTRY_DUEL)
  {
    ctrl->power_mode = MODULE_CHASSIS_POWER_MODE_FIXED;//如果规则预设为步兵对打，则功率模式为固定功率
  }
  else
  {
    ctrl->power_mode = mode;
  }
  
  ctrl->power_limit_w = Module_Chassis_SelectPowerLimit(ctrl->rule_preset, ctrl->power_mode);
  ctrl->power_buffer_j = ctrl->power_buffer_max_j;
  ctrl->power_cut_left_s = 0.0f;
  Module_Chassis_UpdatePowerScale(ctrl);
}

void Module_Chassis_SetControlGate(ModuleChassisCtrl_t *ctrl, uint8_t control_enabled, uint8_t robot_online)
{//设置底盘控制门，控制是否启用和机器人是否在线
  if (ctrl == 0)
  {
    return;
  }

  ctrl->control_enabled = (control_enabled != 0U) ? 1U : 0U;//设置控制是否启用（0：禁用，1：启用）
  ctrl->robot_online = (robot_online != 0U) ? 1U : 0U;//设置机器人是否在线（0：不在线，1：在线）
  Module_Chassis_UpdatePowerScale(ctrl);//更新功率缩放比例
}

void Module_Chassis_SetMeasuredPowerW(ModuleChassisCtrl_t *ctrl, float measured_power_w)
{//设置底盘测量功率（W），如果功率有效，则更新外部功率和外部功率有效标志
  if (ctrl == 0)
  {
    return;
  }

  ctrl->external_power_w = measured_power_w;//设置外部功率（W）
  ctrl->external_power_valid = 1U;//设置外部功率有效标志（1：有效）
}

void Module_Chassis_UpdatePowerGuard(ModuleChassisCtrl_t *ctrl, float measured_power_w, float dt_s)
{//更新底盘功率保护
  if (ctrl == 0 || dt_s <= 0.0f)
  {
    return;
  }

  if (ctrl->power_cut_left_s > 0.0f)
  {
    ctrl->power_cut_left_s -= dt_s;//更新功率保护剩余时间
    if (ctrl->power_cut_left_s < 0.0f)
    {
      ctrl->power_cut_left_s = 0.0f;
    }
  }

  ctrl->power_guard_acc_s += dt_s;

  while (ctrl->power_guard_acc_s >= ctrl->power_guard_period_s)
  {
    ctrl->power_guard_acc_s -= ctrl->power_guard_period_s;

    if (ctrl->power_cut_left_s > 0.0f)
    {
      continue;
    }

    ctrl->power_buffer_j -= (measured_power_w - ctrl->power_limit_w) * ctrl->power_guard_period_s;//更新功率缓冲区能量
    ctrl->power_buffer_j = Module_Chassis_Clamp(ctrl->power_buffer_j, 0.0f, ctrl->power_buffer_max_j);//限制功率缓冲区能量在[0, max_j]范围内

    if (ctrl->power_buffer_j <= 0.0f && measured_power_w > ctrl->power_limit_w)
    {//如果功率缓冲区能量小于等于0且测量功率大于功率限制，则触发功率保护
      ctrl->power_cut_left_s = MODULE_CHASSIS_POWER_CUT_TIME_S;//设置功率保护剩余时间为默认值
      ctrl->power_buffer_j = 0.0f;//设置功率缓冲区能量为0
      break;
    }
  }

  Module_Chassis_UpdatePowerScale(ctrl);
}

void Module_Chassis_Update(ModuleChassisCtrl_t *ctrl,
                           const BSP_RemoteState_t *remote,
                           ModuleChassisOut_t *out)
{//更新底盘状态
  float gain;
  float vx_raw;
  float vy_raw;
  float wz_raw;
  float measured_power_w;
  float output_scale;

  if (ctrl == 0 || remote == 0 || out == 0)
  {
    return;
  }

  out->remote_online = remote->online;
  out->fast_mode = 0U;

  vx_raw = 0.0f;
  vy_raw = 0.0f;
  wz_raw = 0.0f;

  if (remote->online != 0U)
  {
    out->fast_mode = (remote->sw_left == 1U) ? 1U : 0U;//设置是否快速模式（1：是，0：否）
    gain = out->fast_mode ? 1.0f : 0.6f;//设置增益（快速模式：1.0，慢速模式：0.6）
    //计算垂直方向速度（m/s）=（通道0值-死区）*最大速度（m/s）*增益（快速模式：1.0，慢速模式：0.6）
    vy_raw = Module_Chassis_ApplyDeadband(remote->ch_norm[0], ctrl->deadband) * ctrl->max_v_mps * gain;//计算垂直方向速度（m/s）
    //计算水平方向速度（m/s）=（通道1值-死区）*最大速度（m/s）*增益（快速模式：1.0，慢速模式：0.6）
    vx_raw = Module_Chassis_ApplyDeadband(remote->ch_norm[1], ctrl->deadband) * ctrl->max_v_mps * gain;//计算水平方向速度（m/s）
    //计算角速度（rad/s）=（通道2值-死区）*最大角速度（rad/s）
    wz_raw = Module_Chassis_ApplyDeadband(remote->ch_norm[2], ctrl->deadband) * ctrl->max_wz_radps;//计算角速度（rad/s）

    if (remote->sw_right == 2U)//如果右开关为中间位置
    {
      /* Right switch middle: keep yaw still for precise aiming. */
      wz_raw = 0.0f;//设置角速度（rad/s）为0，保持偏航稳定
    }
  }

  if (ctrl->external_power_valid != 0U)//如果外部功率有效
  {
    measured_power_w = ctrl->external_power_w;
    ctrl->external_power_valid = 0U;
  }
  else
  {
    measured_power_w = Module_Chassis_EstimatePower(ctrl, vx_raw, vy_raw, wz_raw);//估计底盘电机功率（W）
  }

  Module_Chassis_UpdatePowerGuard(ctrl, measured_power_w, ctrl->ctrl_dt_s);//更新功率保护

  output_scale = ctrl->power_scale;
  if (remote->online == 0U)
  {
    output_scale = 0.0f;
  }

  out->vx_cmd_mps = vx_raw * output_scale;//计算水平方向速度指令（m/s）
  out->vy_cmd_mps = vy_raw * output_scale;//计算垂直方向速度指令（m/s）
  out->wz_cmd_radps = wz_raw * output_scale;//计算角速度指令（rad/s）

  out->power_limit_w = ctrl->power_limit_w;//设置功率限制（W）
  out->power_buffer_j = ctrl->power_buffer_j;//设置功率缓冲区能量（J）
  out->power_scale = ctrl->power_scale;//设置功率缩放比例
  out->power_cut_active = (ctrl->power_cut_left_s > 0.0f) ? 1U : 0U;//设置功率保护是否激活（1：激活）
}
