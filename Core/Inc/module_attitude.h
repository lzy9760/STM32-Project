#ifndef MODULE_ATTITUDE_H
#define MODULE_ATTITUDE_H

#include "bsp_imu.h"

typedef struct
{
  float roll_deg;//横滚角度（度）
  float pitch_deg;//俯仰角度（度）
  float yaw_deg;//偏航角度（度）

  float roll_rate_dps;//横滚角速度（度/秒）
  float pitch_rate_dps;//俯仰角速度（度/秒）
  float yaw_rate_dps;//偏航角速度（度/秒）
} ModuleAttitude_t;

void Module_Attitude_Init(ModuleAttitude_t *attitude);
void Module_Attitude_Update(ModuleAttitude_t *attitude,
                            const BSP_ImuSample_t *imu,
                            float dt_s);

#endif /* MODULE_ATTITUDE_H */
