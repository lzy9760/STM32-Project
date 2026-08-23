#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include "at32f403a_407_board.h"

typedef struct
{
    float x_mm;
    float y_mm;
    float yaw_deg;
} OdometryPose;

void Odometry_Init(void);
void Odometry_Update(float delta_time_s);
OdometryPose Odometry_GetPose(void);

#endif
