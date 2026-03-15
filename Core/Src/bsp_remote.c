#include "bsp_remote.h"

#include "stm32f1xx_hal.h"
#include <string.h>

#define BSP_REMOTE_TIMEOUT_MS 100U
#define BSP_REMOTE_MAX_RAW_ABS 700

static BSP_RemoteState_t s_remote;
static uint8_t s_remote_stream[BSP_REMOTE_FRAME_LEN];
static uint16_t s_remote_stream_idx = 0U;

static float BSP_Remote_Clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static uint8_t BSP_Remote_NormalizeSwitch(uint8_t raw)
{
  if (raw < 1U || raw > 3U)
  {
    return 3U;
  }

  return raw;
}

static uint8_t BSP_Remote_IsDecodedValid(const int16_t ch_raw[4], uint8_t sw_left, uint8_t sw_right)
{
  uint8_t i;

  if (sw_left < 1U || sw_left > 3U || sw_right < 1U || sw_right > 3U)
  {
    return 0U;
  }

  for (i = 0U; i < 4U; i++)
  {
    if (ch_raw[i] > BSP_REMOTE_MAX_RAW_ABS || ch_raw[i] < -BSP_REMOTE_MAX_RAW_ABS)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t BSP_Remote_DecodeFrameOne(const uint8_t *frame)
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch_raw[4];
  uint8_t sw_left;
  uint8_t sw_right;

  if (frame == 0)
  {
    return 0U;
  }

  ch0 = (int16_t)(((uint16_t)frame[0] | ((uint16_t)frame[1] << 8U)) & 0x07FFU);
  ch1 = (int16_t)((((uint16_t)frame[1] >> 3U) | ((uint16_t)frame[2] << 5U)) & 0x07FFU);
  ch2 = (int16_t)((((uint16_t)frame[2] >> 6U) | ((uint16_t)frame[3] << 2U) | ((uint16_t)frame[4] << 10U)) & 0x07FFU);
  ch3 = (int16_t)((((uint16_t)frame[4] >> 1U) | ((uint16_t)frame[5] << 7U)) & 0x07FFU);

  ch_raw[0] = ch0 - 1024;
  ch_raw[1] = ch1 - 1024;
  ch_raw[2] = ch2 - 1024;
  ch_raw[3] = ch3 - 1024;

  sw_left = BSP_Remote_NormalizeSwitch((frame[5] >> 4U) & 0x03U);
  sw_right = BSP_Remote_NormalizeSwitch((frame[5] >> 6U) & 0x03U);

  if (BSP_Remote_IsDecodedValid(ch_raw, sw_left, sw_right) == 0U)
  {
    return 0U;
  }

  s_remote.ch_raw[0] = ch_raw[0];
  s_remote.ch_raw[1] = ch_raw[1];
  s_remote.ch_raw[2] = ch_raw[2];
  s_remote.ch_raw[3] = ch_raw[3];

  s_remote.ch_norm[0] = BSP_Remote_Clamp((float)s_remote.ch_raw[0] / 660.0f, -1.0f, 1.0f);
  s_remote.ch_norm[1] = BSP_Remote_Clamp((float)s_remote.ch_raw[1] / 660.0f, -1.0f, 1.0f);
  s_remote.ch_norm[2] = BSP_Remote_Clamp((float)s_remote.ch_raw[2] / 660.0f, -1.0f, 1.0f);
  s_remote.ch_norm[3] = BSP_Remote_Clamp((float)s_remote.ch_raw[3] / 660.0f, -1.0f, 1.0f);

  s_remote.sw_left = sw_left;
  s_remote.sw_right = sw_right;

  s_remote.mouse_x = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8U));
  s_remote.mouse_y = (int16_t)((uint16_t)frame[8] | ((uint16_t)frame[9] << 8U));
  s_remote.mouse_z = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8U));
  s_remote.mouse_l = frame[12];
  s_remote.mouse_r = frame[13];
  s_remote.key_code = (uint16_t)frame[14] | ((uint16_t)frame[15] << 8U);
  s_remote.wheel = (int16_t)((uint16_t)frame[16] | ((uint16_t)frame[17] << 8U));

  s_remote.last_update_ms = HAL_GetTick();
  s_remote.online = 1U;

  return 1U;
}

void BSP_Remote_Init(void)
{
  memset(&s_remote, 0, sizeof(s_remote));
  s_remote.sw_left = 3U;
  s_remote.sw_right = 3U;
  s_remote.last_update_ms = HAL_GetTick() - (BSP_REMOTE_TIMEOUT_MS + 1U);
  s_remote.online = 0U;

  s_remote_stream_idx = 0U;
}

void BSP_Remote_DecodeFrame(const uint8_t *frame, uint16_t len)
{
  uint16_t offset;

  if (frame == 0 || len < BSP_REMOTE_FRAME_LEN)
  {
    return;
  }

  for (offset = 0U; (offset + BSP_REMOTE_FRAME_LEN) <= len; offset++)
  {
    if (BSP_Remote_DecodeFrameOne(&frame[offset]) != 0U)
    {
      return;
    }
  }
}

void BSP_Remote_FeedByte(uint8_t byte)
{
  s_remote_stream[s_remote_stream_idx++] = byte;

  if (s_remote_stream_idx < BSP_REMOTE_FRAME_LEN)
  {
    return;
  }

  if (BSP_Remote_DecodeFrameOne(s_remote_stream) != 0U)
  {
    s_remote_stream_idx = 0U;
    return;
  }

  memmove(s_remote_stream, &s_remote_stream[1], BSP_REMOTE_FRAME_LEN - 1U);
  s_remote_stream_idx = BSP_REMOTE_FRAME_LEN - 1U;
}

void BSP_Remote_GetState(BSP_RemoteState_t *out_state)
{
  uint32_t now;

  if (out_state == 0)
  {
    return;
  }

  now = HAL_GetTick();
  if ((now - s_remote.last_update_ms) > BSP_REMOTE_TIMEOUT_MS)
  {
    s_remote.online = 0U;
    s_remote.ch_raw[0] = 0;
    s_remote.ch_raw[1] = 0;
    s_remote.ch_raw[2] = 0;
    s_remote.ch_raw[3] = 0;
    s_remote.ch_norm[0] = 0.0f;
    s_remote.ch_norm[1] = 0.0f;
    s_remote.ch_norm[2] = 0.0f;
    s_remote.ch_norm[3] = 0.0f;
    s_remote.wheel = 0;
    s_remote.sw_left = 3U;
    s_remote.sw_right = 3U;
  }

  *out_state = s_remote;
}

uint8_t BSP_Remote_IsOnline(void)
{
  uint32_t now;

  now = HAL_GetTick();
  if ((now - s_remote.last_update_ms) > BSP_REMOTE_TIMEOUT_MS)
  {
    return 0U;
  }

  return s_remote.online;
}

void BSP_Remote_SetMockChannels(float ch0,
                                float ch1,
                                float ch2,
                                float ch3,
                                uint8_t sw_left,
                                uint8_t sw_right)
{
  s_remote.ch_norm[0] = BSP_Remote_Clamp(ch0, -1.0f, 1.0f);
  s_remote.ch_norm[1] = BSP_Remote_Clamp(ch1, -1.0f, 1.0f);
  s_remote.ch_norm[2] = BSP_Remote_Clamp(ch2, -1.0f, 1.0f);
  s_remote.ch_norm[3] = BSP_Remote_Clamp(ch3, -1.0f, 1.0f);

  s_remote.ch_raw[0] = (int16_t)(s_remote.ch_norm[0] * 660.0f);
  s_remote.ch_raw[1] = (int16_t)(s_remote.ch_norm[1] * 660.0f);
  s_remote.ch_raw[2] = (int16_t)(s_remote.ch_norm[2] * 660.0f);
  s_remote.ch_raw[3] = (int16_t)(s_remote.ch_norm[3] * 660.0f);

  s_remote.sw_left = BSP_Remote_NormalizeSwitch(sw_left);
  s_remote.sw_right = BSP_Remote_NormalizeSwitch(sw_right);
  s_remote.last_update_ms = HAL_GetTick();
  s_remote.online = 1U;
}
