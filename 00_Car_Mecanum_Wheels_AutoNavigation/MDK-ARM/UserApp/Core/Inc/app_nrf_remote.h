#ifndef __APP_NRF_REMOTE_H
#define __APP_NRF_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

typedef struct
{
  int16_t left_x;
  int16_t left_y;
  int16_t right_x;
  int16_t right_y;
} NrfRemote_Data_t;

void NrfRemote_Init(void);
uint8_t NrfRemote_Read(NrfRemote_Data_t *data);
uint8_t NrfRemote_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_NRF_REMOTE_H */
