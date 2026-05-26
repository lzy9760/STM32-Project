#ifndef __APP_ODOMETRY_H
#define __APP_ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

typedef struct
{
  float x_mm;
  float y_mm;
  float yaw_deg;
} Odometry_Pose_t;

void Odometry_Init(void);
void Odometry_Reset(float x_mm, float y_mm, float yaw_deg);
void Odometry_Update(void);
Odometry_Pose_t Odometry_GetPose(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ODOMETRY_H */
