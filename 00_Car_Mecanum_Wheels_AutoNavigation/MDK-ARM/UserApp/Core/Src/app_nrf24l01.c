#include "app_nrf24l01.h"
#include "main.h"

#define NRF_CE_Pin             GPIO_PIN_8
#define NRF_CE_GPIO_Port       GPIOA
#define NRF_CSN_Pin            GPIO_PIN_8
#define NRF_CSN_GPIO_Port      GPIOB
#define NRF_IRQ_Pin            GPIO_PIN_9
#define NRF_IRQ_GPIO_Port      GPIOB
#define NRF_SCK_Pin            GPIO_PIN_3
#define NRF_SCK_GPIO_Port      GPIOB
#define NRF_MISO_Pin           GPIO_PIN_4
#define NRF_MISO_GPIO_Port     GPIOB
#define NRF_MOSI_Pin           GPIO_PIN_5
#define NRF_MOSI_GPIO_Port     GPIOB

#define NRF_CHANNEL            0x4CU
#define NRF_ADDR_WIDTH         5U
#define NRF_SETUP_RETR_VALUE   0x15U

#define NRF_CMD_R_REGISTER     0x00U
#define NRF_CMD_W_REGISTER     0x20U
#define NRF_CMD_R_RX_PL_WID    0x60U
#define NRF_CMD_R_RX_PAYLOAD   0x61U
#define NRF_CMD_ACTIVATE       0x50U
#define NRF_CMD_W_TX_PAYLOAD   0xA0U
#define NRF_CMD_W_ACK_PAYLOAD  0xA8U
#define NRF_CMD_FLUSH_TX       0xE1U
#define NRF_CMD_FLUSH_RX       0xE2U
#define NRF_CMD_NOP            0xFFU

#define NRF_REG_CONFIG         0x00U
#define NRF_REG_EN_AA          0x01U
#define NRF_REG_EN_RXADDR      0x02U
#define NRF_REG_SETUP_AW       0x03U
#define NRF_REG_SETUP_RETR     0x04U
#define NRF_REG_RF_CH          0x05U
#define NRF_REG_RF_SETUP       0x06U
#define NRF_REG_STATUS         0x07U
#define NRF_REG_RX_ADDR_P0     0x0AU//接收地址寄存器0
#define NRF_REG_TX_ADDR        0x10U
#define NRF_REG_RX_PW_P0       0x11U
#define NRF_REG_FIFO_STATUS    0x17U
#define NRF_REG_DYNPD          0x1CU
#define NRF_REG_FEATURE        0x1DU

#define NRF_CONFIG_PRIM_RX     0x01U
#define NRF_CONFIG_PWR_UP      0x02U
#define NRF_CONFIG_CRCO        0x04U
#define NRF_CONFIG_EN_CRC      0x08U

#define NRF_FEATURE_EN_ACK_PAY 0x02U
#define NRF_FEATURE_EN_DPL     0x04U
#define NRF_DYNPD_DPL_P0       0x01U
#define NRF_ENABLE_ACK_PAYLOAD 0U

#define NRF_STATUS_RX_DR       0x40U
#define NRF_STATUS_TX_OK       0x20U
#define NRF_STATUS_MAX_RT      0x10U
#define NRF_STATUS_IRQ_MASK    (NRF_STATUS_RX_DR | NRF_STATUS_TX_OK | NRF_STATUS_MAX_RT)
#define NRF_FIFO_RX_EMPTY      0x01U
#define NRF_FIFO_TX_FULL       0x20U

static uint8_t g_nrf_ready;
static uint8_t g_nrf_last_status;
static uint8_t g_nrf_tx_address[NRF_ADDR_WIDTH];

void Nrf24_CE_Low(void)
{
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

void Nrf24_CE_High(void)
{
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

static void Nrf24_CSN_Low(void)
{
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

static void Nrf24_CSN_High(void)
{
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

uint8_t Nrf24_IsIrqActive(void)
{
  return (HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void Nrf24_SpiDelay(void)
{
  volatile uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
  }
}

static uint8_t Nrf24_TransferByte(uint8_t data)
{
  uint8_t i;
  uint8_t rx;

  rx = 0U;
  for (i = 0U; i < 8U; i++)
  {
    if ((data & 0x80U) != 0U)
    {
      HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port, NRF_MOSI_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port, NRF_MOSI_Pin, GPIO_PIN_RESET);
    }

    data <<= 1U;
    Nrf24_SpiDelay();
    HAL_GPIO_WritePin(NRF_SCK_GPIO_Port, NRF_SCK_Pin, GPIO_PIN_SET);
    Nrf24_SpiDelay();

    rx <<= 1U;
    if (HAL_GPIO_ReadPin(NRF_MISO_GPIO_Port, NRF_MISO_Pin) == GPIO_PIN_SET)
    {
      rx |= 0x01U;
    }

    HAL_GPIO_WritePin(NRF_SCK_GPIO_Port, NRF_SCK_Pin, GPIO_PIN_RESET);
    Nrf24_SpiDelay();
  }

  return rx;
}

static uint8_t Nrf24_ReadRegister(uint8_t reg)
{
  uint8_t value;

  Nrf24_CSN_Low();
  g_nrf_last_status = Nrf24_TransferByte((uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1FU)));
  value = Nrf24_TransferByte(NRF_CMD_NOP);
  Nrf24_CSN_High();

  return value;
}

static uint8_t Nrf24_WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t status;

  Nrf24_CSN_Low();
  status = Nrf24_TransferByte((uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
  Nrf24_TransferByte(value);
  Nrf24_CSN_High();
  g_nrf_last_status = status;

  return status;
}

static uint8_t Nrf24_ReadBuffer(uint8_t reg, uint8_t *data, uint8_t length)
{
  uint8_t i;
  uint8_t status;

  Nrf24_CSN_Low();
  status = Nrf24_TransferByte((uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1FU)));
  for (i = 0U; i < length; i++)
  {
    data[i] = Nrf24_TransferByte(NRF_CMD_NOP);
  }
  Nrf24_CSN_High();
  g_nrf_last_status = status;

  return status;
}

static uint8_t Nrf24_WriteBuffer(uint8_t reg, const uint8_t *data, uint8_t length)
{
  uint8_t i;
  uint8_t status;

  Nrf24_CSN_Low();
  status = Nrf24_TransferByte((uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
  for (i = 0U; i < length; i++)
  {
    Nrf24_TransferByte(data[i]);
  }
  Nrf24_CSN_High();
  g_nrf_last_status = status;

  return status;
}

static void Nrf24_Command(uint8_t command)
{
  Nrf24_CSN_Low();
  g_nrf_last_status = Nrf24_TransferByte(command);
  Nrf24_CSN_High();
}

static void Nrf24_ClearIrqFlags(void)
{
  Nrf24_WriteRegister(NRF_REG_STATUS, NRF_STATUS_IRQ_MASK);
}

static void Nrf24_ActivateFeatures(void)
{
  Nrf24_CSN_Low();
  g_nrf_last_status = Nrf24_TransferByte(NRF_CMD_ACTIVATE);
  Nrf24_TransferByte(0x73U);
  Nrf24_CSN_High();
}

static void Nrf24_DisableAckPayload(void)
{
  Nrf24_WriteRegister(NRF_REG_FEATURE, 0x00U);
  Nrf24_WriteRegister(NRF_REG_DYNPD, 0x00U);
  if ((Nrf24_ReadRegister(NRF_REG_FEATURE) != 0x00U) ||
      (Nrf24_ReadRegister(NRF_REG_DYNPD) != 0x00U))
  {
    Nrf24_ActivateFeatures();
    Nrf24_WriteRegister(NRF_REG_FEATURE, 0x00U);
    Nrf24_WriteRegister(NRF_REG_DYNPD, 0x00U);
  }
}

#if (NRF_ENABLE_ACK_PAYLOAD != 0U)
static void Nrf24_EnableAckPayload(void)
{
  uint8_t feature;

  feature = (uint8_t)(NRF_FEATURE_EN_ACK_PAY | NRF_FEATURE_EN_DPL);
  Nrf24_WriteRegister(NRF_REG_FEATURE, feature);
  if (Nrf24_ReadRegister(NRF_REG_FEATURE) != feature)
  {
    Nrf24_ActivateFeatures();
    Nrf24_WriteRegister(NRF_REG_FEATURE, feature);
  }
  Nrf24_WriteRegister(NRF_REG_DYNPD, NRF_DYNPD_DPL_P0);
}
#endif

static uint8_t Nrf24_WriteTxPayloadWithAddress(const uint8_t *address, const uint8_t *payload, uint8_t length)
{
  uint8_t i;
  uint8_t status;

  if ((address == NULL) || (payload == NULL) || (length == 0U) || (length > 32U))
  {
    return 0U;
  }

  Nrf24_WriteBuffer(NRF_REG_TX_ADDR, address, NRF_ADDR_WIDTH);
  Nrf24_WriteBuffer(NRF_REG_RX_ADDR_P0, address, NRF_ADDR_WIDTH);

  Nrf24_CSN_Low();
  status = Nrf24_TransferByte(NRF_CMD_W_TX_PAYLOAD);
  for (i = 0U; i < length; i++)
  {
    Nrf24_TransferByte(payload[i]);
  }
  Nrf24_CSN_High();
  g_nrf_last_status = status;

  return 1U;
}

static void Nrf24_GpioInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
#ifdef __HAL_RCC_AFIO_CLK_ENABLE
  __HAL_RCC_AFIO_CLK_ENABLE();
#endif
#ifdef __HAL_AFIO_REMAP_SWJ_NOJTAG
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
#endif

  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NRF_SCK_GPIO_Port, NRF_SCK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port, NRF_MOSI_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF_CSN_Pin|NRF_SCK_Pin|NRF_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

uint8_t Nrf24_Init(void)
{
  static const uint8_t address[NRF_ADDR_WIDTH] = {0x11U, 0x22U, 0x33U, 0x44U, 0x55U};
  uint8_t readback[NRF_ADDR_WIDTH];
  uint8_t i;
  uint8_t ok;

  g_nrf_ready = 0U;
  Nrf24_GpioInit();
  Nrf24_CE_Low();
  Nrf24_CSN_High();
  HAL_Delay(5U);

  Nrf24_WriteRegister(NRF_REG_CONFIG, NRF_CONFIG_EN_CRC);
  Nrf24_WriteRegister(NRF_REG_EN_AA, 0x3FU);
  Nrf24_WriteRegister(NRF_REG_EN_RXADDR, 0x01U);
  Nrf24_WriteRegister(NRF_REG_SETUP_AW, 0x03U);
  Nrf24_WriteRegister(NRF_REG_SETUP_RETR, NRF_SETUP_RETR_VALUE);
  Nrf24_WriteRegister(NRF_REG_RF_CH, NRF_CHANNEL);
  Nrf24_WriteRegister(NRF_REG_RF_SETUP, 0x27U);
  Nrf24_WriteBuffer(NRF_REG_TX_ADDR, address, NRF_ADDR_WIDTH);
  Nrf24_WriteBuffer(NRF_REG_RX_ADDR_P0, address, NRF_ADDR_WIDTH);
  for (i = 0U; i < NRF_ADDR_WIDTH; i++)
  {
    g_nrf_tx_address[i] = address[i];
  }
  Nrf24_WriteRegister(NRF_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE);
#if (NRF_ENABLE_ACK_PAYLOAD != 0U)
  Nrf24_EnableAckPayload();
#else
  Nrf24_DisableAckPayload();
#endif
  Nrf24_WriteRegister(NRF_REG_STATUS, (uint8_t)(NRF_STATUS_RX_DR | NRF_STATUS_TX_OK | NRF_STATUS_MAX_RT));
  Nrf24_Command(NRF_CMD_FLUSH_TX);
  Nrf24_Command(NRF_CMD_FLUSH_RX);
  Nrf24_WriteRegister(NRF_REG_CONFIG,
                      (uint8_t)(NRF_CONFIG_PRIM_RX | NRF_CONFIG_PWR_UP | NRF_CONFIG_EN_CRC));
  HAL_Delay(2U);
  Nrf24_CE_High();

  ok = 1U;
  for (i = 0U; i < NRF_ADDR_WIDTH; i++)
  {
    readback[i] = 0U;
  }
  Nrf24_ReadBuffer(NRF_REG_RX_ADDR_P0, readback, NRF_ADDR_WIDTH);
  for (i = 0U; i < NRF_ADDR_WIDTH; i++)
  {
    if (readback[i] != address[i])
    {
      ok = 0U;
    }
  }

  if (Nrf24_ReadRegister(NRF_REG_RF_CH) != NRF_CHANNEL)
  {
    ok = 0U;
  }

  if (Nrf24_ReadRegister(NRF_REG_RF_SETUP) != 0x27U)
  {
    ok = 0U;
  }

  if (Nrf24_ReadRegister(NRF_REG_RX_PW_P0) != NRF24_PAYLOAD_SIZE)
  {
    ok = 0U;
  }

  g_nrf_ready = ok;
  return ok;
}

uint8_t Nrf24_IsReady(void)
{
  return g_nrf_ready;
}

uint8_t Nrf24_ReadPayload(uint8_t *payload, uint8_t length)
{
  uint8_t i;
  uint8_t status;
  uint8_t fifo_status;

  if ((g_nrf_ready == 0U) || (payload == NULL) || (length < NRF24_PAYLOAD_SIZE))
  {
    return 0U;
  }

  status = Nrf24_ReadRegister(NRF_REG_STATUS);
  fifo_status = Nrf24_ReadRegister(NRF_REG_FIFO_STATUS);
  if ((fifo_status & NRF_FIFO_RX_EMPTY) != 0U)
  {
    if ((status & NRF_STATUS_IRQ_MASK) != 0U)
    {
      Nrf24_ClearIrqFlags();
    }
    return 0U;
  }

  Nrf24_CSN_Low();
  g_nrf_last_status = Nrf24_TransferByte(NRF_CMD_R_RX_PAYLOAD);
  for (i = 0U; i < NRF24_PAYLOAD_SIZE; i++)
  {
    payload[i] = Nrf24_TransferByte(NRF_CMD_NOP);
  }
  Nrf24_CSN_High();

  Nrf24_ClearIrqFlags();
  if ((Nrf24_ReadRegister(NRF_REG_FIFO_STATUS) & NRF_FIFO_RX_EMPTY) == 0U)
  {
    Nrf24_ClearIrqFlags();
  }

  return 1U;
}

uint8_t Nrf24_WriteAckPayload(const uint8_t *payload, uint8_t length)
{
  uint8_t i;
  uint8_t fifo_status;

  if ((g_nrf_ready == 0U) || (payload == NULL) || (length == 0U) || (length > 32U))
  {
    return 0U;
  }

#if (NRF_ENABLE_ACK_PAYLOAD != 0U)
  fifo_status = Nrf24_ReadRegister(NRF_REG_FIFO_STATUS);
  if ((fifo_status & NRF_FIFO_TX_FULL) != 0U)
  {
    Nrf24_Command(NRF_CMD_FLUSH_TX);
  }
  Nrf24_CSN_Low();
  g_nrf_last_status = Nrf24_TransferByte(NRF_CMD_W_ACK_PAYLOAD);
  for (i = 0U; i < length; i++)
  {
    Nrf24_TransferByte(payload[i]);
  }
  Nrf24_CSN_High();
#else
  (void)i;
  (void)fifo_status;
  (void)payload;
  (void)length;
#endif

  return 1U;
}

uint8_t Nrf24_SendPayload(const uint8_t *address, const uint8_t *payload, uint8_t length)
{
  uint8_t status;
  uint8_t config;
  uint8_t send_flag;
  uint16_t poll_count;
  uint32_t start_tick;
  const uint8_t *tx_address;

  if ((g_nrf_ready == 0U) || (payload == NULL) || (length == 0U) || (length > 32U))
  {
    return 4U;
  }

  tx_address = (address != NULL) ? address : g_nrf_tx_address;
  config = Nrf24_ReadRegister(NRF_REG_CONFIG);

  Nrf24_CE_Low();
  Nrf24_WriteRegister(NRF_REG_STATUS, 0x70U);
  Nrf24_Command(NRF_CMD_FLUSH_TX);
  /* Keep the current one-way link settings while sending this optional packet. */
  Nrf24_WriteRegister(NRF_REG_CONFIG, (uint8_t)((config | NRF_CONFIG_PWR_UP) & (uint8_t)~NRF_CONFIG_PRIM_RX));
  (void)Nrf24_WriteTxPayloadWithAddress(tx_address, payload, length);

  start_tick = HAL_GetTick();
  Nrf24_CE_High();
  send_flag = 4U;
  poll_count = 0U;
  while (1)
  {
    status = Nrf24_ReadRegister(NRF_REG_STATUS);
    if ((status & NRF_STATUS_TX_OK) != 0U)
    {
      send_flag = 1U;
      break;
    }
    if ((status & NRF_STATUS_MAX_RT) != 0U)
    {
      send_flag = 2U;
      break;
    }
    if (((HAL_GetTick() - start_tick) >= 5U) || (++poll_count >= 5000U))
    {
      send_flag = 4U;
      break;
    }
  }

  Nrf24_CE_Low();
  Nrf24_WriteRegister(NRF_REG_STATUS, 0x70U);
  Nrf24_Command(NRF_CMD_FLUSH_TX);
  Nrf24_WriteBuffer(NRF_REG_TX_ADDR, g_nrf_tx_address, NRF_ADDR_WIDTH);
  Nrf24_WriteBuffer(NRF_REG_RX_ADDR_P0, g_nrf_tx_address, NRF_ADDR_WIDTH);
  Nrf24_WriteRegister(NRF_REG_CONFIG, config);
  if ((config & NRF_CONFIG_PRIM_RX) != 0U)
  {
    Nrf24_CE_High();
  }

  return send_flag;
}

uint8_t Nrf24_GetStatus(void)
{
  return g_nrf_last_status;
}

uint8_t Nrf24_ReadStatusRegister(void)
{
  return Nrf24_ReadRegister(NRF_REG_STATUS);
}

uint8_t Nrf24_ReadFifoStatusRegister(void)
{
  return Nrf24_ReadRegister(NRF_REG_FIFO_STATUS);
}
