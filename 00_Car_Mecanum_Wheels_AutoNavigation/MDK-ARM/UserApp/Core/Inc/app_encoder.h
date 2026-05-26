#ifndef __APP_ENCODER_H
#define __APP_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

#define ENCODER_U1    0U
#define ENCODER_U2    1U
#define ENCODER_U3    2U
#define ENCODER_U4    3U
#define ENCODER_COUNT 4U

void Encoder_Init(void);
void Encoder_Scan(void);
int32_t Encoder_GetCount(uint8_t encoder_id);
void Encoder_ResetCount(uint8_t encoder_id);
void Encoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ENCODER_H */
