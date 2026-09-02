#include "ball_vision.h"

#include "ball_config.h"

#define VISION_RX_BUFFER_SIZE 128U
#define VISION_LINE_SIZE       24U

static volatile uint8_t s_rx_buffer[VISION_RX_BUFFER_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static char s_line[VISION_LINE_SIZE];
static uint8_t s_line_length;
static uint8_t s_discard_line;
static uint8_t s_velocity_ready;
static float s_last_position_mm;
static uint32_t s_last_position_ms;
static BallVisionData_t s_data;

static uint8_t ParsePosition(const char *text, float *position_mm)
{
  uint32_t integer = 0U;
  uint32_t fraction = 0U;
  uint32_t divisor = 1U;
  uint8_t index = 0U;
  uint8_t digits = 0U;
  uint8_t negative = 0U;
  uint8_t exponent_negative = 0U;
  uint8_t exponent_digits = 0U;
  uint8_t exponent = 0U;
  float value;

  while ((text[index] == ' ') || (text[index] == '\t'))
  {
    ++index;
  }
  if ((text[index] == '+') || (text[index] == '-'))
  {
    negative = (text[index] == '-') ? 1U : 0U;
    ++index;
  }
  while ((text[index] >= '0') && (text[index] <= '9'))
  {
    integer = integer * 10U + (uint32_t)(text[index] - '0');
    ++digits;
    ++index;
    if (integer > 10000U)
    {
      return 0U;
    }
  }
  if (text[index] == '.')
  {
    ++index;
    while ((text[index] >= '0') && (text[index] <= '9'))
    {
      if (divisor < 1000000U)
      {
        fraction = fraction * 10U + (uint32_t)(text[index] - '0');
        divisor *= 10U;
      }
      ++digits;
      ++index;
    }
  }
  if ((text[index] == 'e') || (text[index] == 'E'))
  {
    ++index;
    if ((text[index] == '+') || (text[index] == '-'))
    {
      exponent_negative = (text[index] == '-') ? 1U : 0U;
      ++index;
    }
    while ((text[index] >= '0') && (text[index] <= '9'))
    {
      exponent = (uint8_t)(exponent * 10U
                           + (uint8_t)(text[index] - '0'));
      ++exponent_digits;
      ++index;
      if (exponent > 6U)
      {
        return 0U;
      }
    }
    if (exponent_digits == 0U)
    {
      return 0U;
    }
  }
  while ((text[index] == ' ') || (text[index] == '\t'))
  {
    ++index;
  }
  if ((digits == 0U) || (text[index] != '\0'))
  {
    return 0U;
  }

  value = (float)integer + (float)fraction / (float)divisor;
  while (exponent > 0U)
  {
    value = (exponent_negative != 0U) ? value * 0.1f : value * 10.0f;
    --exponent;
  }
  if (negative != 0U)
  {
    value = -value;
  }
  if ((value < -(float)BALL_VISION_POSITION_LIMIT_X10 * 0.1f)
      || (value > (float)BALL_VISION_POSITION_LIMIT_X10 * 0.1f))
  {
    return 0U;
  }
  *position_mm = value;
  return 1U;
}

static void AcceptPosition(float position_mm, uint32_t now_ms)
{
  uint32_t dt_ms;
  float raw_velocity;

  s_data.position_mm = position_mm;
  s_data.last_frame_ms = now_ms;
  s_data.valid = 1U;
  ++s_data.good_frames;

  if (s_velocity_ready != 0U)
  {
    dt_ms = now_ms - s_last_position_ms;
    if ((dt_ms > 0U) && (dt_ms <= 200U))
    {
      raw_velocity = (position_mm - s_last_position_mm)
                     * 1000.0f / (float)dt_ms;
      s_data.velocity_mm_s += BALL_VISION_VELOCITY_ALPHA
                             * (raw_velocity - s_data.velocity_mm_s);
    }
    else
    {
      s_data.velocity_mm_s = 0.0f;
    }
  }
  else
  {
    s_data.velocity_mm_s = 0.0f;
    s_velocity_ready = 1U;
  }
  s_last_position_mm = position_mm;
  s_last_position_ms = now_ms;
}

static void ParseByte(uint8_t value, uint32_t now_ms)
{
  float position_mm;

  if ((value == '\r') || (value == '\n'))
  {
    if (s_discard_line != 0U)
    {
      s_discard_line = 0U;
      s_line_length = 0U;
      return;
    }
    if (s_line_length == 0U)
    {
      return;
    }
    s_line[s_line_length] = '\0';
    if (ParsePosition(s_line, &position_mm) != 0U)
    {
      AcceptPosition(position_mm, now_ms);
    }
    else
    {
      ++s_data.parse_errors;
    }
    s_line_length = 0U;
    return;
  }

  if (s_line_length < (VISION_LINE_SIZE - 1U))
  {
    s_line[s_line_length++] = (char)value;
  }
  else
  {
    s_line_length = 0U;
    s_discard_line = 1U;
    ++s_data.parse_errors;
  }
}

HAL_StatusTypeDef BallVision_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t peripheral_clock;

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_UART5_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &gpio);

  peripheral_clock = HAL_RCC_GetPCLK1Freq();
  UART5->CR1 = 0U;
  UART5->CR2 = 0U;
  UART5->CR3 = 0U;
  UART5->BRR = (peripheral_clock + (BALL_VISION_BAUD / 2U))
               / BALL_VISION_BAUD;
  UART5->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_line_length = 0U;
  s_discard_line = 0U;
  s_velocity_ready = 0U;
  s_data.position_mm = 0.0f;
  s_data.velocity_mm_s = 0.0f;
  s_data.last_frame_ms = 0U;
  s_data.good_frames = 0U;
  s_data.parse_errors = 0U;
  s_data.dropped_bytes = 0U;
  s_data.valid = 0U;

  HAL_NVIC_SetPriority(UART5_IRQn, 1U, 1U);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
  return HAL_OK;
}

void BallVision_Update(uint32_t now_ms)
{
  uint8_t value;

  while (s_rx_tail != s_rx_head)
  {
    value = s_rx_buffer[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) % VISION_RX_BUFFER_SIZE);
    ParseByte(value, now_ms);
  }
}

const BallVisionData_t *BallVision_GetData(void)
{
  return &s_data;
}

uint8_t BallVision_IsFresh(uint32_t now_ms)
{
  if (s_data.valid == 0U)
  {
    return 0U;
  }
  return ((uint32_t)(now_ms - s_data.last_frame_ms)
          <= BALL_VISION_TIMEOUT_MS) ? 1U : 0U;
}

void UART5_IRQHandler(void)
{
  uint32_t status = UART5->SR;
  uint8_t value;
  uint8_t next;

  if ((status & (USART_SR_RXNE | USART_SR_ORE
                 | USART_SR_NE | USART_SR_FE)) != 0U)
  {
    value = (uint8_t)UART5->DR;
    if ((status & USART_SR_RXNE) != 0U)
    {
      next = (uint8_t)((s_rx_head + 1U) % VISION_RX_BUFFER_SIZE);
      if (next != s_rx_tail)
      {
        s_rx_buffer[s_rx_head] = value;
        s_rx_head = next;
      }
      else
      {
        ++s_data.dropped_bytes;
      }
    }
  }
}
