#ifndef __APP_NRF24L01_H
#define __APP_NRF24L01_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

#define NRF24_PAYLOAD_SIZE 4U

void Nrf24_CE_Low(void);
void Nrf24_CE_High(void);
uint8_t Nrf24_Init(void);
uint8_t Nrf24_IsReady(void);
uint8_t Nrf24_IsIrqActive(void);
uint8_t Nrf24_ReadPayload(uint8_t *payload, uint8_t length);
uint8_t Nrf24_WriteAckPayload(const uint8_t *payload, uint8_t length);
uint8_t Nrf24_SendPayload(const uint8_t *address, const uint8_t *payload, uint8_t length);
uint8_t Nrf24_GetStatus(void);
uint8_t Nrf24_ReadStatusRegister(void);
uint8_t Nrf24_ReadFifoStatusRegister(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_NRF24L01_H */
