#ifndef __HWT9018_H__
#define __HWT9018_H__

#include "at32f403a_407_board.h"

typedef struct
{
    float yaw_deg;
    float gyro_z_deg_s;
    uint32_t angle_frame_count;
    uint32_t gyro_frame_count;
    uint32_t angle_timestamp_ms;
    uint32_t gyro_timestamp_ms;
    uint8_t yaw_valid;
    uint8_t gyro_valid;
    uint8_t gyro_bias_valid;
} HWT9018Snapshot;

/* HWT901B-TTL default serial setting: 115200, 8N1. */
void HWT9018_Init(void);
void HWT9018_ZeroYaw(void);
float HWT9018_GetYawDeg(void);
float HWT9018_GetGyroZDegS(void);
uint8_t HWT9018_YawValid(void);
uint8_t HWT9018_GyroValid(void);
uint8_t HWT9018_GyroBiasValid(void);
uint32_t HWT9018_GetAngleFrameCount(void);
uint32_t HWT9018_GetGyroFrameCount(void);
void HWT9018_GetSnapshot(HWT9018Snapshot *snapshot);

extern volatile float g_gyro_z_dps;
extern volatile float g_hwt_yaw_deg;
extern volatile float g_hwt_gyro_bias_dps;
extern volatile uint32_t g_angle_frame_count;
extern volatile uint32_t g_gyro_frame_count;
extern volatile uint32_t g_angle_frame_timestamp_ms;
extern volatile uint32_t g_gyro_frame_timestamp_ms;
extern volatile uint32_t g_hwt_checksum_error_count;
extern volatile uint8_t g_gyro_bias_valid;
extern volatile uint8_t g_hwt_output_config_sent;

#endif
