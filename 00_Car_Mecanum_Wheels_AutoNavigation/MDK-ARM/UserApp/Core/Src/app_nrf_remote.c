#include "app_nrf_remote.h"

#include "app_nrf24l01.h"

#define NRF_REMOTE_REINIT_MS      1000U
#define NRF_REMOTE_AXIS_CENTER    128
#define NRF_REMOTE_AXIS_MAX       100
static uint8_t g_nrf_remote_ready;
static uint32_t g_nrf_remote_next_init_tick;

static int16_t NrfRemote_DecodeAxis(uint8_t value)
{
  int16_t centered;
  int16_t axis;

  centered = (int16_t)value - NRF_REMOTE_AXIS_CENTER;
  if (centered >= 0)
  {
    axis = (int16_t)((centered * NRF_REMOTE_AXIS_MAX + 63) / 127);
  }
  else
  {
    axis = (int16_t)((centered * NRF_REMOTE_AXIS_MAX - 64) / 128);
  }

  if (axis > NRF_REMOTE_AXIS_MAX)
  {
    axis = NRF_REMOTE_AXIS_MAX;
  }
  else if (axis < -NRF_REMOTE_AXIS_MAX)
  {
    axis = -NRF_REMOTE_AXIS_MAX;
  }

  return axis;
}

static void NrfRemote_FillData(const uint8_t *packet, NrfRemote_Data_t *data)
{
  data->left_y = NrfRemote_DecodeAxis(packet[0]);
  data->left_x = NrfRemote_DecodeAxis(packet[1]);
  data->right_y = NrfRemote_DecodeAxis(packet[2]);
  data->right_x = NrfRemote_DecodeAxis(packet[3]);
}

void NrfRemote_Init(void)
{
  g_nrf_remote_ready = Nrf24_Init();
  g_nrf_remote_next_init_tick = HAL_GetTick() + NRF_REMOTE_REINIT_MS;
}

uint8_t NrfRemote_Read(NrfRemote_Data_t *data)
{
  uint8_t packet[NRF24_PAYLOAD_SIZE];
  uint8_t read_count;
  uint8_t has_data;
  uint32_t now_tick;

  if (data == NULL)
  {
    return 0U;
  }

  if (g_nrf_remote_ready == 0U)
  {
    now_tick = HAL_GetTick();
    if (now_tick >= g_nrf_remote_next_init_tick)
    {
      g_nrf_remote_ready = Nrf24_Init();
      g_nrf_remote_next_init_tick = now_tick + NRF_REMOTE_REINIT_MS;
    }
    return 0U;
  }

  has_data = 0U;
  for (read_count = 0U; read_count < 8U; read_count++)
  {
    if (Nrf24_ReadPayload(packet, sizeof(packet)) == 0U)
    {
      break;
    }

    NrfRemote_FillData(packet, data);
    has_data = 1U;
  }

  return has_data;
}

uint8_t NrfRemote_IsReady(void)
{
  return g_nrf_remote_ready;
}
