#include "bsp_can.h"

#include <string.h>

static CAN_HandleTypeDef s_hcan1;
static BSP_CanRxCallback_t s_rx_callback = 0;

static HAL_StatusTypeDef BSP_CAN_SendCurrentFrame(uint16_t std_id,
                                                   int16_t c1,
                                                   int16_t c2,
                                                   int16_t c3,
                                                   int16_t c4)
{
  uint8_t data[8];

  data[0] = (uint8_t)((uint16_t)c1 >> 8U);
  data[1] = (uint8_t)((uint16_t)c1 & 0xFFU);
  data[2] = (uint8_t)((uint16_t)c2 >> 8U);
  data[3] = (uint8_t)((uint16_t)c2 & 0xFFU);
  data[4] = (uint8_t)((uint16_t)c3 >> 8U);
  data[5] = (uint8_t)((uint16_t)c3 & 0xFFU);
  data[6] = (uint8_t)((uint16_t)c4 >> 8U);
  data[7] = (uint8_t)((uint16_t)c4 & 0xFFU);

  return BSP_CAN_SendStdData(std_id, data, 8U);
}

HAL_StatusTypeDef BSP_CAN_Init(void)
{
  CAN_FilterTypeDef filter;

  memset(&s_hcan1, 0, sizeof(s_hcan1));
  s_hcan1.Instance = CAN1;
  s_hcan1.Init.Prescaler = 6U;
  s_hcan1.Init.Mode = CAN_MODE_NORMAL;
  s_hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  s_hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  s_hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  s_hcan1.Init.TimeTriggeredMode = DISABLE;
  s_hcan1.Init.AutoBusOff = ENABLE;
  s_hcan1.Init.AutoWakeUp = DISABLE;
  s_hcan1.Init.AutoRetransmission = ENABLE;
  s_hcan1.Init.ReceiveFifoLocked = DISABLE;
  s_hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&s_hcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  memset(&filter, 0, sizeof(filter));
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&s_hcan1, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_Start(&s_hcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_ActivateNotification(&s_hcan1,
                                   CAN_IT_RX_FIFO0_MSG_PENDING |
                                   CAN_IT_BUSOFF |
                                   CAN_IT_ERROR_WARNING |
                                   CAN_IT_ERROR_PASSIVE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

CAN_HandleTypeDef *BSP_CAN_GetHandle(void)
{
  return &s_hcan1;
}

void BSP_CAN_RegisterRxCallback(BSP_CanRxCallback_t callback)
{
  s_rx_callback = callback;
}

HAL_StatusTypeDef BSP_CAN_SendStdData(uint16_t std_id, const uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;

  if (data == 0 || len > 8U)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&s_hcan1) == 0U)
  {
    return HAL_BUSY;
  }

  memset(&tx_header, 0, sizeof(tx_header));
  tx_header.StdId = (uint32_t)(std_id & 0x07FFU);
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;

  return HAL_CAN_AddTxMessage(&s_hcan1, &tx_header, (uint8_t *)data, &tx_mailbox);
}

HAL_StatusTypeDef BSP_CAN_SendCurrentGroup1(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
  return BSP_CAN_SendCurrentFrame(0x200U, m1, m2, m3, m4);
}

HAL_StatusTypeDef BSP_CAN_SendCurrentGroup2(int16_t m5, int16_t m6, int16_t m7, int16_t m8)
{
  return BSP_CAN_SendCurrentFrame(0x1FFU, m5, m6, m7, m8);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];

  if (hcan == 0 || hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
  {
    return;
  }

  if (s_rx_callback != 0 && rx_header.IDE == CAN_ID_STD)
  {
    s_rx_callback((uint16_t)rx_header.StdId, data, rx_header.DLC);
  }
}
