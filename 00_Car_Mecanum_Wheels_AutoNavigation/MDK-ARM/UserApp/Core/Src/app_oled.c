#include "app_oled.h"

#include "app_nrf_remote.h"
#include "app_odometry.h"
#include "app_remote.h"

#include <stdio.h>

#define OLED_SCL_PIN              GPIO_PIN_8
#define OLED_SCL_GPIO_PORT        GPIOC
#define OLED_SDA_PIN              GPIO_PIN_9
#define OLED_SDA_GPIO_PORT        GPIOC

#define OLED_I2C_ADDR             0x78U
#define OLED_WIDTH                128U
#define OLED_PAGES                8U
#define OLED_UPDATE_PERIOD_MS     200U

static uint8_t g_oled_buf[OLED_PAGES][OLED_WIDTH];
static uint32_t g_oled_last_update_tick;
static uint8_t g_oled_ready;

static void Oled_Delay(void)
{
  volatile uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
  }
}

static void Oled_WriteScl(uint8_t value)
{
  HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_PIN,
                    (value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  Oled_Delay();
}

static void Oled_WriteSda(uint8_t value)
{
  HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_PIN,
                    (value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  Oled_Delay();
}

static void Oled_I2cStart(void)
{
  Oled_WriteSda(1U);
  Oled_WriteScl(1U);
  Oled_WriteSda(0U);
  Oled_WriteScl(0U);
}

static void Oled_I2cStop(void)
{
  Oled_WriteSda(0U);
  Oled_WriteScl(1U);
  Oled_WriteSda(1U);
}

static void Oled_I2cWriteByte(uint8_t data)
{
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    Oled_WriteSda((data & 0x80U) != 0U);
    Oled_WriteScl(1U);
    Oled_WriteScl(0U);
    data <<= 1U;
  }

  Oled_WriteSda(1U);
  Oled_WriteScl(1U);
  Oled_WriteScl(0U);
}

static void Oled_WriteCommand(uint8_t command)
{
  Oled_I2cStart();
  Oled_I2cWriteByte(OLED_I2C_ADDR);
  Oled_I2cWriteByte(0x00U);
  Oled_I2cWriteByte(command);
  Oled_I2cStop();
}

static void Oled_WriteData(const uint8_t *data, uint8_t length)
{
  uint8_t i;

  Oled_I2cStart();
  Oled_I2cWriteByte(OLED_I2C_ADDR);
  Oled_I2cWriteByte(0x40U);
  for (i = 0U; i < length; i++)
  {
    Oled_I2cWriteByte(data[i]);
  }
  Oled_I2cStop();
}

static void Oled_SetCursor(uint8_t page, uint8_t x)
{
  Oled_WriteCommand((uint8_t)(0xB0U | page));
  Oled_WriteCommand((uint8_t)(0x10U | ((x & 0xF0U) >> 4U)));
  Oled_WriteCommand((uint8_t)(0x00U | (x & 0x0FU)));
}

static void Oled_ClearBuffer(void)
{
  uint8_t page;
  uint8_t x;

  for (page = 0U; page < OLED_PAGES; page++)
  {
    for (x = 0U; x < OLED_WIDTH; x++)
    {
      g_oled_buf[page][x] = 0U;
    }
  }
}

static void Oled_Update(void)
{
  uint8_t page;

  for (page = 0U; page < OLED_PAGES; page++)
  {
    Oled_SetCursor(page, 0U);
    Oled_WriteData(g_oled_buf[page], OLED_WIDTH);
  }
}

static void Oled_GetGlyph(char ch, uint8_t glyph[5])
{
  uint8_t i;
  static const uint8_t blank[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
  static const uint8_t minus[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  static const uint8_t plus[5]  = {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U};
  static const uint8_t digits[10][5] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
  };
  static const uint8_t glyph_a[5] = {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU};
  static const uint8_t glyph_c[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
  static const uint8_t glyph_d[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
  static const uint8_t glyph_m[5] = {0x7EU, 0x02U, 0x0CU, 0x02U, 0x7EU};
  static const uint8_t glyph_o[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
  static const uint8_t glyph_r[5] = {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U};
  static const uint8_t glyph_x[5] = {0x63U, 0x14U, 0x08U, 0x14U, 0x63U};
  static const uint8_t glyph_y[5] = {0x07U, 0x08U, 0x70U, 0x08U, 0x07U};
  const uint8_t *src;

  if ((ch >= '0') && (ch <= '9'))
  {
    src = digits[ch - '0'];
  }
  else
  {
    switch (ch)
    {
      case ':': src = colon; break;
      case '-': src = minus; break;
      case '+': src = plus; break;
      case 'A': src = glyph_a; break;
      case 'C': src = glyph_c; break;
      case 'D': src = glyph_d; break;
      case 'M':
      case 'm': src = glyph_m; break;
      case 'O': src = glyph_o; break;
      case 'R': src = glyph_r; break;
      case 'X': src = glyph_x; break;
      case 'Y': src = glyph_y; break;
      default: src = blank; break;
    }
  }

  for (i = 0U; i < 5U; i++)
  {
    glyph[i] = src[i];
  }
}

static void Oled_DrawString(uint8_t x, uint8_t page, const char *text)
{
  uint8_t glyph[5];
  uint8_t i;

  while ((*text != '\0') && (page < OLED_PAGES) && (x < (OLED_WIDTH - 5U)))
  {
    Oled_GetGlyph(*text, glyph);
    for (i = 0U; i < 5U; i++)
    {
      g_oled_buf[page][x + i] = glyph[i];
    }
    if ((x + 5U) < OLED_WIDTH)
    {
      g_oled_buf[page][x + 5U] = 0U;
    }
    x = (uint8_t)(x + 6U);
    text++;
  }
}

static int32_t Oled_RoundMm(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value + 0.5f);
  }

  return (int32_t)(value - 0.5f);
}

static void Oled_RenderPose(void)
{
  Odometry_Pose_t pose;
  char line[18];

  pose = Odometry_GetPose();
  Oled_ClearBuffer();

  (void)snprintf(line, sizeof(line), "C:%u R:%u",
                 (unsigned int)NrfRemote_IsReady(),
                 (unsigned int)Remote_IsOnline());
  Oled_DrawString(0U, 0U, line);
  (void)snprintf(line, sizeof(line), "X:%7ld", (long)Oled_RoundMm(pose.x_mm));
  Oled_DrawString(0U, 2U, line);
  (void)snprintf(line, sizeof(line), "Y:%7ld", (long)Oled_RoundMm(pose.y_mm));
  Oled_DrawString(0U, 4U, line);
  (void)snprintf(line, sizeof(line), "A:%5ld", (long)Oled_RoundMm(pose.yaw_deg));
  Oled_DrawString(0U, 6U, line);
}

void AppOled_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_PIN, GPIO_PIN_SET);

  gpio.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &gpio);

  HAL_Delay(20U);
  Oled_WriteCommand(0xAEU);
  Oled_WriteCommand(0xD5U);
  Oled_WriteCommand(0x80U);
  Oled_WriteCommand(0xA8U);
  Oled_WriteCommand(0x3FU);
  Oled_WriteCommand(0xD3U);
  Oled_WriteCommand(0x00U);
  Oled_WriteCommand(0x40U);
  Oled_WriteCommand(0xA1U);
  Oled_WriteCommand(0xC8U);
  Oled_WriteCommand(0xDAU);
  Oled_WriteCommand(0x12U);
  Oled_WriteCommand(0x81U);
  Oled_WriteCommand(0xCFU);
  Oled_WriteCommand(0xD9U);
  Oled_WriteCommand(0xF1U);
  Oled_WriteCommand(0xDBU);
  Oled_WriteCommand(0x30U);
  Oled_WriteCommand(0xA4U);
  Oled_WriteCommand(0xA6U);
  Oled_WriteCommand(0x8DU);
  Oled_WriteCommand(0x14U);
  Oled_WriteCommand(0xAFU);

  Oled_RenderPose();
  Oled_Update();
  g_oled_last_update_tick = HAL_GetTick();
  g_oled_ready = 1U;
}

void AppOled_Task(void)
{
  uint32_t now;

  if (g_oled_ready == 0U)
  {
    return;
  }

  now = HAL_GetTick();
  if ((now - g_oled_last_update_tick) < OLED_UPDATE_PERIOD_MS)
  {
    return;
  }
  g_oled_last_update_tick = now;

  Oled_RenderPose();
  Oled_Update();
}
