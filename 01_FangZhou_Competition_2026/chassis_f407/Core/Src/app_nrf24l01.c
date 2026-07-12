#include "app_nrf24l01.h"

#define NRF_CE_PORT             GPIOA
#define NRF_CE_PIN              GPIO_PIN_8
#define NRF_CSN_PORT            GPIOA
#define NRF_CSN_PIN             GPIO_PIN_9
#define NRF_SCK_PORT            GPIOA
#define NRF_SCK_PIN             GPIO_PIN_10
#define NRF_MISO_PORT           GPIOB
#define NRF_MISO_PIN            GPIO_PIN_13
#define NRF_MOSI_PORT           GPIOB
#define NRF_MOSI_PIN            GPIO_PIN_14

#define NRF_R_REGISTER          0x00U
#define NRF_W_REGISTER          0x20U
#define NRF_R_RX_PAYLOAD        0x61U
#define NRF_FLUSH_TX            0xE1U
#define NRF_FLUSH_RX            0xE2U
#define NRF_ACTIVATE            0x50U
#define NRF_NOP                 0xFFU

#define NRF_REG_CONFIG          0x00U
#define NRF_REG_EN_AA           0x01U
#define NRF_REG_EN_RXADDR       0x02U
#define NRF_REG_SETUP_AW        0x03U
#define NRF_REG_SETUP_RETR      0x04U
#define NRF_REG_RF_CH           0x05U
#define NRF_REG_RF_SETUP        0x06U
#define NRF_REG_STATUS          0x07U
#define NRF_REG_RX_ADDR_P0      0x0AU
#define NRF_REG_TX_ADDR         0x10U
#define NRF_REG_RX_PW_P0        0x11U
#define NRF_REG_FIFO_STATUS     0x17U
#define NRF_REG_DYNPD           0x1CU
#define NRF_REG_FEATURE         0x1DU

#define NRF_STATUS_RX_DR        0x40U
#define NRF_STATUS_TX_DS        0x20U
#define NRF_STATUS_MAX_RT       0x10U
#define NRF_STATUS_CLEAR_ALL    0x70U
#define NRF_FIFO_RX_EMPTY       0x01U

#define NRF_RF_CHANNEL_VALUE    0x4CU
#define NRF_RF_SETUP_VALUE      0x27U
#define NRF_SETUP_RETR_VALUE    0x15U

static const uint8_t s_nrf_address[5] = {0x11U, 0x22U, 0x33U, 0x44U, 0x55U};
static uint8_t s_connected;

static void NrfWriteCe(uint8_t value)
{
  HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, (value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void NrfWriteCsn(uint8_t value)
{
  HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, (value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void NrfWriteSck(uint8_t value)
{
  HAL_GPIO_WritePin(NRF_SCK_PORT, NRF_SCK_PIN, (value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void NrfWriteMosi(uint8_t value)
{
  HAL_GPIO_WritePin(NRF_MOSI_PORT, NRF_MOSI_PIN, (value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t NrfReadMiso(void)
{
  return (HAL_GPIO_ReadPin(NRF_MISO_PORT, NRF_MISO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t NrfSwapByte(uint8_t byte)
{
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    NrfWriteMosi((byte & 0x80U) != 0U);
    byte <<= 1;
    NrfWriteSck(1U);
    if (NrfReadMiso() != 0U)
    {
      byte |= 0x01U;
    }
    NrfWriteSck(0U);
  }

  return byte;
}

static uint8_t NrfReadReg(uint8_t reg)
{
  uint8_t data;

  NrfWriteCsn(0U);
  (void)NrfSwapByte(NRF_R_REGISTER | (reg & 0x1FU));
  data = NrfSwapByte(NRF_NOP);
  NrfWriteCsn(1U);

  return data;
}

static void NrfReadRegs(uint8_t reg, uint8_t *data, uint8_t count)
{
  uint8_t i;

  NrfWriteCsn(0U);
  (void)NrfSwapByte(NRF_R_REGISTER | (reg & 0x1FU));
  for (i = 0U; i < count; i++)
  {
    data[i] = NrfSwapByte(NRF_NOP);
  }
  NrfWriteCsn(1U);
}

static void NrfWriteReg(uint8_t reg, uint8_t data)
{
  NrfWriteCsn(0U);
  (void)NrfSwapByte(NRF_W_REGISTER | (reg & 0x1FU));
  (void)NrfSwapByte(data);
  NrfWriteCsn(1U);
}

static void NrfWriteRegs(uint8_t reg, const uint8_t *data, uint8_t count)
{
  uint8_t i;

  NrfWriteCsn(0U);
  (void)NrfSwapByte(NRF_W_REGISTER | (reg & 0x1FU));
  for (i = 0U; i < count; i++)
  {
    (void)NrfSwapByte(data[i]);
  }
  NrfWriteCsn(1U);
}

static void NrfReadRxPayload(uint8_t *data, uint8_t count)
{
  uint8_t i;

  NrfWriteCsn(0U);
  (void)NrfSwapByte(NRF_R_RX_PAYLOAD);
  for (i = 0U; i < count; i++)
  {
    data[i] = NrfSwapByte(NRF_NOP);
  }
  NrfWriteCsn(1U);
}

static void NrfCommand(uint8_t command)
{
  NrfWriteCsn(0U);
  (void)NrfSwapByte(command);
  NrfWriteCsn(1U);
}

static void NrfActivateFeatures(void)
{
  NrfWriteCsn(0U);
  (void)NrfSwapByte(NRF_ACTIVATE);
  (void)NrfSwapByte(0x73U);
  NrfWriteCsn(1U);
}

static void NrfDisableAdvancedFeatures(void)
{
  NrfWriteReg(NRF_REG_FEATURE, 0x00U);
  NrfWriteReg(NRF_REG_DYNPD, 0x00U);
  if ((NrfReadReg(NRF_REG_FEATURE) != 0x00U) || (NrfReadReg(NRF_REG_DYNPD) != 0x00U))
  {
    NrfActivateFeatures();
    NrfWriteReg(NRF_REG_FEATURE, 0x00U);
    NrfWriteReg(NRF_REG_DYNPD, 0x00U);
  }
}

static void NrfRxMode(void)
{
  uint8_t config;

  NrfWriteCe(0U);
  config = NrfReadReg(NRF_REG_CONFIG);
  config |= 0x03U;
  NrfWriteReg(NRF_REG_CONFIG, config);
  HAL_Delay(2U);
  NrfWriteCe(1U);
}

static void NrfGpioInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  gpio.Pin = NRF_CE_PIN | NRF_CSN_PIN | NRF_SCK_PIN;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = NRF_MOSI_PIN;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = NRF_MISO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);

  NrfWriteCe(0U);
  NrfWriteCsn(1U);
  NrfWriteSck(0U);
  NrfWriteMosi(0U);
}

uint8_t AppNrf24l01_Init(void)
{
  uint8_t read_address[5];

  NrfGpioInit();

  NrfWriteReg(NRF_REG_CONFIG, 0x08U);
  NrfWriteReg(NRF_REG_EN_AA, 0x3FU);
  NrfWriteReg(NRF_REG_EN_RXADDR, 0x01U);
  NrfWriteReg(NRF_REG_SETUP_AW, 0x03U);
  NrfWriteReg(NRF_REG_SETUP_RETR, NRF_SETUP_RETR_VALUE);
  NrfWriteReg(NRF_REG_RF_CH, NRF_RF_CHANNEL_VALUE);
  NrfWriteReg(NRF_REG_RF_SETUP, NRF_RF_SETUP_VALUE);
  NrfWriteReg(NRF_REG_RX_PW_P0, APP_NRF24L01_PAYLOAD_WIDTH);
  NrfWriteRegs(NRF_REG_TX_ADDR, s_nrf_address, 5U);
  NrfWriteRegs(NRF_REG_RX_ADDR_P0, s_nrf_address, 5U);
  NrfDisableAdvancedFeatures();
  NrfCommand(NRF_FLUSH_TX);
  NrfCommand(NRF_FLUSH_RX);
  NrfWriteReg(NRF_REG_STATUS, NRF_STATUS_CLEAR_ALL);
  NrfRxMode();

  NrfReadRegs(NRF_REG_RX_ADDR_P0, read_address, 5U);
  s_connected = ((NrfReadReg(NRF_REG_RF_CH) == NRF_RF_CHANNEL_VALUE) &&
                 (NrfReadReg(NRF_REG_RX_PW_P0) == APP_NRF24L01_PAYLOAD_WIDTH) &&
                 (read_address[0] == s_nrf_address[0]) &&
                 (read_address[1] == s_nrf_address[1]) &&
                 (read_address[2] == s_nrf_address[2]) &&
                 (read_address[3] == s_nrf_address[3]) &&
                 (read_address[4] == s_nrf_address[4])) ? 1U : 0U;

  return s_connected;
}

AppNrf24l01RxResult_t AppNrf24l01_Receive(uint8_t packet[APP_NRF24L01_PAYLOAD_WIDTH])
{
  uint8_t status;
  uint8_t config;
  uint8_t fifo_status;

  if (packet == 0)
  {
    return APP_NRF24L01_RX_ERROR;
  }

  status = NrfReadReg(NRF_REG_STATUS);
  config = NrfReadReg(NRF_REG_CONFIG);
  if ((config & 0x02U) == 0U)
  {
    (void)AppNrf24l01_Init();
    return APP_NRF24L01_RX_ERROR;
  }

  if ((status & (NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT)) != 0U)
  {
    NrfWriteReg(NRF_REG_STATUS, (uint8_t)(status & (NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT)));
  }

  fifo_status = NrfReadReg(NRF_REG_FIFO_STATUS);
  if (((status & NRF_STATUS_RX_DR) == 0U) && ((fifo_status & NRF_FIFO_RX_EMPTY) != 0U))
  {
    return APP_NRF24L01_RX_EMPTY;
  }

  NrfReadRxPayload(packet, APP_NRF24L01_PAYLOAD_WIDTH);
  NrfWriteReg(NRF_REG_STATUS, NRF_STATUS_RX_DR);
  NrfCommand(NRF_FLUSH_RX);

  return APP_NRF24L01_RX_OK;
}

uint8_t AppNrf24l01_IsConnected(void)
{
  return s_connected;
}
