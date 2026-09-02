#include "oled.h"

#include <string.h>

#define OLED_WIDTH                 128U
#define OLED_TEXT_COLUMNS           21U
#define OLED_I2C_TIMEOUT_MS         20U

static I2C_HandleTypeDef *s_hi2c;
static uint16_t s_address;
static uint8_t s_present;

static const uint8_t *GetGlyph(char c)
{
  static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};
  static const uint8_t dot[5] = {0U, 0x60U, 0x60U, 0U, 0U};
  static const uint8_t digit[10][5] =
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
  static const uint8_t letter[26][5] =
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

  if ((c >= 'a') && (c <= 'z'))
  {
    c = (char)(c - ('a' - 'A'));
  }
  if ((c >= '0') && (c <= '9'))
  {
    return digit[c - '0'];
  }
  if ((c >= 'A') && (c <= 'Z'))
  {
    return letter[c - 'A'];
  }
  if (c == '.')
  {
    return dot;
  }
  return blank;
}

static HAL_StatusTypeDef WriteCommands(const uint8_t *commands, uint8_t count)
{
  uint8_t packet[32];

  if ((s_present == 0U) || (count > (uint8_t)(sizeof(packet) - 1U)))
  {
    return HAL_ERROR;
  }
  packet[0] = 0x00U;
  memcpy(&packet[1], commands, count);
  return HAL_I2C_Master_Transmit(s_hi2c, s_address, packet,
                                 (uint16_t)count + 1U,
                                 OLED_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef WriteData(const uint8_t *data, uint8_t count)
{
  uint8_t packet[OLED_WIDTH + 1U];

  if ((s_present == 0U) || (count > OLED_WIDTH))
  {
    return HAL_ERROR;
  }
  packet[0] = 0x40U;
  memcpy(&packet[1], data, count);
  return HAL_I2C_Master_Transmit(s_hi2c, s_address, packet,
                                 (uint16_t)count + 1U,
                                 OLED_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef SetCursor(uint8_t row, uint8_t x)
{
  uint8_t command[3];

  command[0] = (uint8_t)(0xB0U + row);
  command[1] = (uint8_t)(x & 0x0FU);
  command[2] = (uint8_t)(0x10U | (x >> 4U));
  return WriteCommands(command, 3U);
}

HAL_StatusTypeDef Oled_Init(I2C_HandleTypeDef *hi2c)
{
  static const uint8_t init_commands[] =
  {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0x8DU, 0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
    0x81U, 0x7FU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
    0xAFU
  };
  static const uint8_t address[] = {(uint8_t)(0x3CU << 1U),
                                    (uint8_t)(0x3DU << 1U)};
  uint8_t zero[OLED_WIDTH] = {0U};
  uint8_t i;

  s_hi2c = hi2c;
  s_present = 0U;
  if (s_hi2c == NULL)
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < (uint8_t)sizeof(address); ++i)
  {
    if (HAL_I2C_IsDeviceReady(s_hi2c, address[i], 2U, 10U) == HAL_OK)
    {
      s_address = address[i];
      s_present = 1U;
      break;
    }
  }
  if (s_present == 0U)
  {
    return HAL_ERROR;
  }
  if (WriteCommands(init_commands, (uint8_t)sizeof(init_commands)) != HAL_OK)
  {
    s_present = 0U;
    return HAL_ERROR;
  }

  for (i = 0U; i < 8U; ++i)
  {
    if ((SetCursor(i, 0U) != HAL_OK)
        || (WriteData(zero, OLED_WIDTH) != HAL_OK))
    {
      s_present = 0U;
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

uint8_t Oled_IsPresent(void)
{
  return s_present;
}

void Oled_WriteText(uint8_t row, uint8_t column, const char *text)
{
  uint8_t data[OLED_WIDTH];
  uint8_t count = 0U;
  const uint8_t *glyph;
  uint8_t i;

  if ((s_present == 0U) || (row >= 8U) || (column >= OLED_TEXT_COLUMNS)
      || (text == NULL))
  {
    return;
  }

  while ((*text != '\0') && (count <= (OLED_WIDTH - 6U)))
  {
    glyph = GetGlyph(*text++);
    for (i = 0U; i < 5U; ++i)
    {
      data[count++] = glyph[i];
    }
    data[count++] = 0U;
  }

  if ((count != 0U) && (SetCursor(row, (uint8_t)(column * 6U)) == HAL_OK))
  {
    (void)WriteData(data, count);
  }
}
