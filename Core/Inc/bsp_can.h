#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include <stdint.h>

typedef void (*BSP_CanRxCallback_t)(uint16_t std_id, const uint8_t *data, uint8_t dlc);

HAL_StatusTypeDef BSP_CAN_Init(void);
CAN_HandleTypeDef *BSP_CAN_GetHandle(void);

void BSP_CAN_RegisterRxCallback(BSP_CanRxCallback_t callback);

HAL_StatusTypeDef BSP_CAN_SendStdData(uint16_t std_id, const uint8_t *data, uint8_t len);
HAL_StatusTypeDef BSP_CAN_SendCurrentGroup1(int16_t m1, int16_t m2, int16_t m3, int16_t m4); /* StdID 0x200 */
HAL_StatusTypeDef BSP_CAN_SendCurrentGroup2(int16_t m5, int16_t m6, int16_t m7, int16_t m8); /* StdID 0x1FF */

#endif /* BSP_CAN_H */
