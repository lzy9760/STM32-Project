#ifndef MODULE_CHASSIS_H
#define MODULE_CHASSIS_H

#include "bsp_remote.h"
#include <stdint.h>

typedef enum
{
  MODULE_CHASSIS_RULE_3V3_INFANTRY = 0,//3V3步兵
  MODULE_CHASSIS_RULE_INFANTRY_DUEL = 1//步兵对打
} ModuleChassisRulePreset_t;

typedef enum
{
  MODULE_CHASSIS_POWER_MODE_HP_FIRST = 0,//生命值优先
  MODULE_CHASSIS_POWER_MODE_POWER_FIRST = 1,//功率优先
  MODULE_CHASSIS_POWER_MODE_FIXED = 2//固定功率
} ModuleChassisPowerMode_t;

typedef struct
{
  float max_v_mps;//最大速度（m/s）
  float max_wz_radps;//最大角速度（rad/s）
  float deadband;//死区
  float ctrl_dt_s;//控制周期（s）

  ModuleChassisRulePreset_t rule_preset;//规则预设
  ModuleChassisPowerMode_t power_mode;//功率模式

  float power_limit_w;//功率限制（W）
  float power_buffer_j;//功率缓冲区（J）
  float power_buffer_max_j;//功率缓冲区最大功率（J）
  float power_guard_period_s;//功率保护周期（s）
  float power_guard_acc_s;//功率保护加速度（s）
  float power_cut_left_s;//功率保护剩余时间（s）
  float power_scale;//功率缩放比例

  float power_proxy_gain_v;//功率代理增益（v）
  float power_proxy_gain_wz;//功率代理增益（wz）
  float external_power_w;//外部功率（W）
  uint8_t external_power_valid;//外部功率是否有效

  uint8_t control_enabled;//控制是否启用
  uint8_t robot_online;
} ModuleChassisCtrl_t;

typedef struct
{
  float vx_cmd_mps;//水平方向速度指令（m/s）
  float vy_cmd_mps;//垂直方向速度指令（m/s）
  float wz_cmd_radps;//角速度指令（rad/s）
  uint8_t remote_online;//遥控器是否在线
  uint8_t fast_mode;//是否快速模式

  float power_limit_w;//功率限制（W）
  float power_buffer_j;//功率缓冲区（J）
  float power_scale;//功率缩放比例
  uint8_t power_cut_active;//功率保护是否激活（1：激活）
} ModuleChassisOut_t;

void Module_Chassis_Init(ModuleChassisCtrl_t *ctrl);
void Module_Chassis_SetRulePreset(ModuleChassisCtrl_t *ctrl, ModuleChassisRulePreset_t preset);
void Module_Chassis_SetPowerMode(ModuleChassisCtrl_t *ctrl, ModuleChassisPowerMode_t mode);
void Module_Chassis_SetControlGate(ModuleChassisCtrl_t *ctrl, uint8_t control_enabled, uint8_t robot_online);
void Module_Chassis_SetMeasuredPowerW(ModuleChassisCtrl_t *ctrl, float measured_power_w);
void Module_Chassis_UpdatePowerGuard(ModuleChassisCtrl_t *ctrl, float measured_power_w, float dt_s);
void Module_Chassis_Update(ModuleChassisCtrl_t *ctrl,
                           const BSP_RemoteState_t *remote,
                           ModuleChassisOut_t *out);

#endif