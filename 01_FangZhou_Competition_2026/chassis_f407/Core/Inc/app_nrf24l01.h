#ifndef APP_NRF24L01_H
#define APP_NRF24L01_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define APP_NRF24L01_PAYLOAD_WIDTH 5U

typedef enum
{
  APP_NRF24L01_RX_EMPTY = 0,
  APP_NRF24L01_RX_OK = 1,
  APP_NRF24L01_RX_ERROR = 2
} AppNrf24l01RxResult_t;

uint8_t AppNrf24l01_Init(void);
AppNrf24l01RxResult_t AppNrf24l01_Receive(uint8_t packet[APP_NRF24L01_PAYLOAD_WIDTH]);
uint8_t AppNrf24l01_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_NRF24L01_H */
