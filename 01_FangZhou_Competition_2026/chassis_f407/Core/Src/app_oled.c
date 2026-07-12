#include "app_oled.h"

#define OLED_CLK_PIN              GPIO_PIN_3
#define OLED_CLK_GPIO_PORT        GPIOB
#define OLED_DIO_PIN              GPIO_PIN_5
#define OLED_DIO_GPIO_PORT        GPIOB

#define OLED_I2C_ADDR             0x78U
#define OLED_WIDTH                128U
#define OLED_PAGES                8U

static uint8_t s_oled_buf[OLED_PAGES][OLED_WIDTH];

static void Oled_Delay(void)
{
  volatile uint8_t i;

  for (i = 0U; i < 12U; i++)
  {
  }
}

static void Oled_WriteClk(uint8_t value)
{
  HAL_GPIO_WritePin(OLED_CLK_GPIO_PORT, OLED_CLK_PIN,
                    (value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  Oled_Delay();
}

static void Oled_WriteDio(uint8_t value)
{
  HAL_GPIO_WritePin(OLED_DIO_GPIO_PORT, OLED_DIO_PIN,
                    (value != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  Oled_Delay();
}

static void Oled_I2cStart(void)
{
  Oled_WriteDio(1U);
  Oled_WriteClk(1U);
  Oled_WriteDio(0U);
  Oled_WriteClk(0U);
}

static void Oled_I2cStop(void)
{
  Oled_WriteDio(0U);
  Oled_WriteClk(1U);
  Oled_WriteDio(1U);
}

static void Oled_I2cWriteByte(uint8_t data)
{
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    Oled_WriteDio((data & 0x80U) != 0U);
    Oled_WriteClk(1U);
    Oled_WriteClk(0U);
    data <<= 1U;
  }

  Oled_WriteDio(1U);
  Oled_WriteClk(1U);
  Oled_WriteClk(0U);
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

static void Oled_GetGlyph(char ch, uint8_t glyph[5])
{
  uint8_t i;
  const uint8_t *src;
  static const uint8_t blank[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
  static const uint8_t minus[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  static const uint8_t percent[5] = {0x63U, 0x13U, 0x08U, 0x64U, 0x63U};
  static const uint8_t digits[10][5] =
  {
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
  static const uint8_t letters[26][5] =
  {
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U},
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U},
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U},
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU},
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU},
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U},
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU},
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU},
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U},
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}
  };

  if ((ch >= 'a') && (ch <= 'z'))
  {
    ch = (char)(ch - ('a' - 'A'));
  }

  if ((ch >= '0') && (ch <= '9'))
  {
    src = digits[ch - '0'];
  }
  else if ((ch >= 'A') && (ch <= 'Z'))
  {
    src = letters[ch - 'A'];
  }
  else
  {
    switch (ch)
    {
      case ':': src = colon; break;
      case '-': src = minus; break;
      case '%': src = percent; break;
      default: src = blank; break;
    }
  }

  for (i = 0U; i < 5U; i++)
  {
    glyph[i] = src[i];
  }
}

void AppOled_Clear(void)
{
  uint8_t page;
  uint8_t x;

  for (page = 0U; page < OLED_PAGES; page++)
  {
    for (x = 0U; x < OLED_WIDTH; x++)
    {
      s_oled_buf[page][x] = 0U;
    }
  }
}

void AppOled_Print(uint8_t x, uint8_t page, const char *text)
{
  uint8_t glyph[5];
  uint8_t i;

  if (text == 0)
  {
    return;
  }

  while ((*text != '\0') && (page < OLED_PAGES) && (x < (OLED_WIDTH - 5U)))
  {
    Oled_GetGlyph(*text, glyph);
    for (i = 0U; i < 5U; i++)
    {
      s_oled_buf[page][x + i] = glyph[i];
    }
    if ((x + 5U) < OLED_WIDTH)
    {
      s_oled_buf[page][x + 5U] = 0U;
    }
    x = (uint8_t)(x + 6U);
    text++;
  }
}

void AppOled_PrintLine(uint8_t page, const char *text)
{
  AppOled_Print(0U, page, text);
}

void AppOled_Update(void)
{
  uint8_t page;

  for (page = 0U; page < OLED_PAGES; page++)
  {
    Oled_SetCursor(page, 0U);
    Oled_WriteData(s_oled_buf[page], OLED_WIDTH);
  }
}

void AppOled_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(OLED_CLK_GPIO_PORT, OLED_CLK_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OLED_DIO_GPIO_PORT, OLED_DIO_PIN, GPIO_PIN_SET);

  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = OLED_CLK_PIN;
  HAL_GPIO_Init(OLED_CLK_GPIO_PORT, &gpio);
  gpio.Pin = OLED_DIO_PIN;
  HAL_GPIO_Init(OLED_DIO_GPIO_PORT, &gpio);

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

  AppOled_Clear();
  AppOled_Update();
}
