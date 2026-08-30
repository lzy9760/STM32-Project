#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#include "at32f403a_407_board.h"
#include "odometry.h"


#define NAV_AUTO_START_DELAY_MS      5000U
#define NAV_HEART_FORWARD_SCALE_MM   3000.0f
#define NAV_HEART_HALF_WIDTH_MM      1500.0f
#define NAV_MAX_SPEED_MM_S           1500.0f

typedef enum
{
    NAV_STATE_WAIT_DELAY = 0,
    NAV_STATE_WAIT_IMU,
    NAV_STATE_RUNNING,
    NAV_STATE_FINISHED,
    NAV_STATE_IMU_FAULT,
    NAV_STATE_TIMEOUT
} NavigationState;

void Navigation_Init(void);
void Navigation_Task(uint32_t now_ms);
uint8_t Navigation_GetMotion(float *vx_m_s,
                             float *vy_m_s,
                             float *vw_rad_s);
NavigationState Navigation_GetState(void);
OdometryPose Navigation_GetPose(void);

#endif
