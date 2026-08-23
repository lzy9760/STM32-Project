#ifndef __HWT9018_H__
#define __HWT9018_H__

#include "at32f403a_407_board.h"

/* HWT901B-TTL default serial setting: 115200, 8N1. */
void HWT9018_Init(void);
void HWT9018_ZeroYaw(void);
float HWT9018_GetYawDeg(void);
uint8_t HWT9018_YawValid(void);
uint32_t HWT9018_GetAngleFrameCount(void);

#endif
