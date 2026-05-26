#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

void Chassis_SetMotion(int16_t forward, int16_t strafe, int16_t yaw);
void Chassis_SetNormalizedMotion(float forward, float strafe, float yaw);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CHASSIS_H */
