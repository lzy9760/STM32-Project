#ifndef APP_CHASSIS_REMOTE_H
#define APP_CHASSIS_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "app_nrf24l01.h"

typedef struct
{
  uint8_t nrf_ok;
  uint8_t link_active;
  uint8_t packet[APP_NRF24L01_PAYLOAD_WIDTH];
  int16_t motor_duty[4];
  uint32_t rx_count;
  uint32_t error_count;
  uint32_t lost_count;
} ChassisRemoteStatus_t;

void ChassisRemote_Init(void);
void ChassisRemote_Task(void);
void ChassisRemote_GetStatus(ChassisRemoteStatus_t *status);
const char *ChassisRemote_GetStateName(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CHASSIS_REMOTE_H */
