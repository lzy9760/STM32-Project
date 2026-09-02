#include "vofa.h"

#include <string.h>

#define VOFA_CHANNEL_COUNT 4U
#define VOFA_FRAME_SIZE    (VOFA_CHANNEL_COUNT * 4U + 4U)

static UART_HandleTypeDef *s_huart;
static uint8_t s_tx_buffer[VOFA_FRAME_SIZE];

void Vofa_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
}

void Vofa_Send4(float channel0, float channel1,
                float channel2, float channel3)
{
  float values[VOFA_CHANNEL_COUNT];

  if ((s_huart == 0) || (s_huart->gState != HAL_UART_STATE_READY))
  {
    return;
  }

  values[0] = channel0;
  values[1] = channel1;
  values[2] = channel2;
  values[3] = channel3;
  memcpy(s_tx_buffer, values, sizeof(values));

  /* VOFA+ JustFloat frame tail: 00 00 80 7F. */
  s_tx_buffer[16] = 0x00U;
  s_tx_buffer[17] = 0x00U;
  s_tx_buffer[18] = 0x80U;
  s_tx_buffer[19] = 0x7FU;
  (void)HAL_UART_Transmit_IT(s_huart, s_tx_buffer, VOFA_FRAME_SIZE);
}

void Vofa_SendWheelSpeed(float left_target, float left_actual,
                         float right_target, float right_actual)
{
  Vofa_Send4(left_target, left_actual, right_target, right_actual);
}
