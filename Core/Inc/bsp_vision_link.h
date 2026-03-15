#ifndef BSP_VISION_LINK_H
#define BSP_VISION_LINK_H

#include <stdint.h>

typedef struct
{
  uint8_t target_valid;
  float yaw_err_deg;
  float pitch_err_deg;
  uint32_t timestamp_ms;
} BSP_VisionTarget_t;

typedef struct
{
  uint8_t mode;
  float yaw_deg;
  float pitch_deg;
  float roll_deg;
  float yaw_rate_dps;
  float pitch_rate_dps;
} BSP_VisionTelemetry_t;

void BSP_Vision_Init(void);
void BSP_Vision_FeedRx(const uint8_t *data, uint16_t len);
uint8_t BSP_Vision_GetLatestTarget(BSP_VisionTarget_t *target, uint32_t timeout_ms);
void BSP_Vision_SendTelemetry(const BSP_VisionTelemetry_t *telemetry);
uint16_t BSP_Vision_PopTx(uint8_t *out, uint16_t max_len);

#endif /* BSP_VISION_LINK_H */
