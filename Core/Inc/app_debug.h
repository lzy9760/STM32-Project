#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include "bsp_vision_link.h"
#include <stdint.h>

void APP_Debug_FillVisionTelemetry(BSP_VisionTelemetry_t *telemetry);
void APP_Debug_UpdateIndicator(uint32_t monitor_tick_ms);
void APP_Debug_Heartbeat(void);

void APP_VisionTxBytesHook(const uint8_t *data, uint16_t len);

#endif /* APP_DEBUG_H */
