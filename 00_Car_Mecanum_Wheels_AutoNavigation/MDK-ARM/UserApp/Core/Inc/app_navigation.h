#ifndef __APP_NAVIGATION_H
#define __APP_NAVIGATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"
#include "app_odometry.h"

void Navigation_Init(void);
void Navigation_Task(void);
uint8_t Navigation_IsActive(void);
void Navigation_StartTo(float target_x_mm, float target_y_mm, float max_speed_mm_s);
void Navigation_Stop(void);
Odometry_Pose_t Navigation_GetPose(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_NAVIGATION_H */
