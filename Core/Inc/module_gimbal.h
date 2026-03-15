#ifndef MODULE_GIMBAL_H
#define MODULE_GIMBAL_H

#include "module_pid.h"
#include <stdint.h>

typedef enum
{
  MODULE_GIMBAL_MODE_SAFE = 0,
  MODULE_GIMBAL_MODE_PATROL = 1,
  MODULE_GIMBAL_MODE_AUTO_AIM = 2
} ModuleGimbalMode_t;

typedef struct
{
  uint8_t enable;
  uint8_t target_valid;

  float vision_yaw_err_deg;
  float vision_pitch_err_deg;

  float yaw_deg;
  float pitch_deg;
  float yaw_rate_dps;
  float pitch_rate_dps;

  float body_roll_deg;
} ModuleGimbalInput_t;

typedef struct
{
  float yaw_current_cmd;
  float pitch_current_cmd;

  float yaw_target_deg;
  float pitch_target_deg;

  ModuleGimbalMode_t mode;
} ModuleGimbalOutput_t;

typedef struct
{
  ModulePid_t yaw_angle_pid;
  ModulePid_t yaw_rate_pid;
  ModulePid_t pitch_angle_pid;
  ModulePid_t pitch_rate_pid;

  ModuleGimbalMode_t mode;
  float yaw_target_deg;
  float pitch_target_deg;

  float patrol_rate_dps;
  float patrol_dir;
} ModuleGimbalCtrl_t;

void Module_Gimbal_Init(ModuleGimbalCtrl_t *ctrl);
void Module_Gimbal_Update(ModuleGimbalCtrl_t *ctrl,
                          const ModuleGimbalInput_t *input,
                          float dt_s,
                          ModuleGimbalOutput_t *output);

#endif /* MODULE_GIMBAL_H */
