#ifndef __APP_MPU6050_H
#define __APP_MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

void    MPU6050_Init(void);
void    MPU6050_Update(void);
float   MPU6050_GetYaw(void);
void    MPU6050_NotifyMoving(uint8_t moving);
int16_t MPU6050_GetHeadingCorrection(int16_t manual_yaw);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MPU6050_H */
