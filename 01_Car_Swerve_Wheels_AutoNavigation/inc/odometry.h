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
void Odometry_Update(float delta_time_s, uint32_t now_ms);
OdometryPose Odometry_GetPose(void);

extern volatile uint8_t g_odometry_ekf_active;
extern volatile uint8_t g_odometry_gyro_fusion_active;
extern volatile uint8_t g_odometry_gyro_sign_locked;
extern volatile uint8_t g_odometry_gyro_sign_fault;
extern volatile uint8_t g_odometry_gyro_measurement_accepted;
extern volatile uint8_t g_odometry_yaw_anchor_accepted;
extern volatile float g_odometry_gyro_runtime_sign;
extern volatile float g_odometry_ekf_yaw_deg;
extern volatile float g_odometry_ekf_bias_dps;
extern volatile float g_odometry_ekf_p_yaw_deg2;
extern volatile float g_odometry_ekf_p_rate_dps2;
extern volatile float g_odometry_ekf_p_bias_dps2;
extern volatile float g_odometry_wheel_yaw_rate_dps;
extern volatile float g_odometry_imu_gyro_rate_dps;
extern volatile float g_odometry_gyro_innovation_dps;
extern volatile float g_odometry_yaw_innovation_deg;
extern volatile float g_odometry_yaw_rate_error_dps;
extern volatile uint32_t g_odometry_gyro_age_ms;
extern volatile uint32_t g_odometry_gyro_accept_count;
extern volatile uint32_t g_odometry_gyro_reject_count;
extern volatile uint32_t g_odometry_yaw_anchor_accept_count;
extern volatile uint32_t g_odometry_yaw_anchor_reject_count;
extern volatile float g_odometry_shadow_ekf_x_mm;
extern volatile float g_odometry_shadow_ekf_y_mm;
extern volatile float g_odometry_shadow_hwt_x_mm;
extern volatile float g_odometry_shadow_hwt_y_mm;
extern volatile float g_odometry_shadow_hwt_minus_ekf_x_mm;
extern volatile float g_odometry_shadow_hwt_minus_ekf_y_mm;
extern volatile float g_odometry_shadow_yaw_diff_deg;
extern volatile float g_odometry_shadow_max_abs_yaw_diff_deg;
extern volatile float g_odometry_shadow_gyro_yaw_deg;
extern volatile float g_odometry_shadow_gyro_x_mm;
extern volatile float g_odometry_shadow_gyro_y_mm;
extern volatile float g_odometry_shadow_hwt_minus_gyro_yaw_deg;
extern volatile float g_odometry_shadow_max_abs_hwt_minus_gyro_yaw_deg;
extern volatile uint8_t g_odometry_shadow_gyro_valid;
extern volatile uint32_t g_odometry_shadow_gyro_invalid_count;
extern volatile uint8_t g_odometry_trapezoid_active;
extern volatile float g_odometry_midpoint_yaw_deg;

#endif
