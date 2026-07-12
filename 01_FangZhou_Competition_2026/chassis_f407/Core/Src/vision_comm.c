#include "vision_comm.h"

#include <stdio.h>
#include <string.h>

#define VISION_LINE_BUF_SIZE        192U

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx_byte;
static char s_rx_line[VISION_LINE_BUF_SIZE];
static uint16_t s_rx_index;
static uint8_t s_drop_line;
static VisionResult_t s_latest;
static VisionColorResult_t s_latest_color;
static VisionCommStats_t s_stats;

static uint8_t Vision_ResultColorCount(const VisionResult_t *result);
static uint8_t Vision_CollectColorsOnly(const char *raw, char *out, size_t out_size, size_t *index, uint8_t color_count);
static uint8_t Vision_FormatColorsOnly(const char *raw, char *out, size_t out_size);

static HAL_StatusTypeDef VisionComm_StartReceive(void)
{
  if (s_uart == 0)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
}

static uint8_t VisionComm_IsThisUart(UART_HandleTypeDef *huart)
{
  return ((huart != 0) && (s_uart != 0) && (huart->Instance == s_uart->Instance)) ? 1U : 0U;
}

static void Vision_CopyToken(char *dst, size_t dst_size, const char *src, size_t src_len)
{
  size_t n;

  if ((dst == 0) || (dst_size == 0U))
  {
    return;
  }

  n = src_len;
  if (n >= dst_size)
  {
    n = dst_size - 1U;
  }

  if ((src != 0) && (n > 0U))
  {
    (void)memcpy(dst, src, n);
  }
  dst[n] = '\0';
}

static uint32_t Vision_ParseU32(const char *src, size_t src_len)
{
  uint32_t value = 0U;
  size_t i;

  for (i = 0U; i < src_len; i++)
  {
    if ((src[i] < '0') || (src[i] > '9'))
    {
      break;
    }
    value = (value * 10U) + (uint32_t)(src[i] - '0');
  }

  return value;
}

static int32_t Vision_ParseI32(const char *src, size_t src_len)
{
  int32_t sign = 1;
  int32_t value = 0;
  size_t i = 0U;

  if ((src_len > 0U) && (src[0] == '-'))
  {
    sign = -1;
    i = 1U;
  }

  for (; i < src_len; i++)
  {
    if ((src[i] < '0') || (src[i] > '9'))
    {
      break;
    }
    value = (value * 10) + (int32_t)(src[i] - '0');
  }

  return sign * value;
}

static void Vision_CommitResult(const VisionResult_t *result)
{
  __disable_irq();
  s_latest = *result;
  s_stats.frame_count++;
  __enable_irq();
}

static void Vision_CommitColorResult(const VisionColorResult_t *result)
{
  __disable_irq();
  s_latest_color = *result;
  s_stats.frame_count++;
  __enable_irq();
}

static uint8_t Vision_ParseQrLine(const char *line)
{
  VisionResult_t result;
  const char *field_start;
  const char *p;
  uint8_t field = 0U;
  uint8_t field_count = 0U;

  if ((line == 0) || (strncmp(line, "QR|", 3U) != 0))
  {
    return 0U;
  }

  (void)memset(&result, 0, sizeof(result));
  field_start = line;
  p = line;

  while (1)
  {
    size_t len;

    if ((*p != '|') && (*p != '\0'))
    {
      p++;
      continue;
    }

    len = (size_t)(p - field_start);
    switch (field)
    {
      case 0U:
        break;
      case 1U:
        result.frame_no = Vision_ParseU32(field_start, len);
        break;
      case 2U:
        result.elapsed_ms = (uint16_t)Vision_ParseU32(field_start, len);
        break;
      case 3U:
        result.direct_hits = (uint8_t)Vision_ParseU32(field_start, len);
        break;
      case 4U:
        result.fenge_hits = (uint8_t)Vision_ParseU32(field_start, len);
        break;
      case 5U:
        result.total_codes = (uint8_t)Vision_ParseU32(field_start, len);
        break;
      case 6U:
        Vision_CopyToken(result.qr_list, sizeof(result.qr_list), field_start, len);
        break;
      case 7U:
        Vision_CopyToken(result.qr_tl, sizeof(result.qr_tl), field_start, len);
        break;
      case 8U:
        Vision_CopyToken(result.qr_tr, sizeof(result.qr_tr), field_start, len);
        break;
      case 9U:
        Vision_CopyToken(result.qr_bl, sizeof(result.qr_bl), field_start, len);
        break;
      case 10U:
        Vision_CopyToken(result.qr_br, sizeof(result.qr_br), field_start, len);
        break;
      default:
        break;
    }

    field_count++;
    if (*p == '\0')
    {
      break;
    }

    field++;
    p++;
    field_start = p;
  }

  if (field_count < 6U)
  {
    return 0U;
  }

  if (Vision_ResultColorCount(&result) < 1U)
  {
    return 1U;
  }

  result.valid = 1U;
  Vision_CommitResult(&result);
  return 1U;
}

static uint8_t Vision_ParseColorLine(const char *line)
{
  VisionColorResult_t result;
  const char *field_start;
  const char *p;
  uint8_t field = 0U;
  uint8_t field_count = 0U;

  if ((line == 0) || (strncmp(line, "CLR|", 4U) != 0))
  {
    return 0U;
  }

  (void)memset(&result, 0, sizeof(result));
  field_start = line;
  p = line;

  while (1)
  {
    size_t len;

    if ((*p != '|') && (*p != '\0'))
    {
      p++;
      continue;
    }

    len = (size_t)(p - field_start);
    switch (field)
    {
      case 0U:
        break;
      case 1U:
        result.frame_no = Vision_ParseU32(field_start, len);
        break;
      case 2U:
        result.color1_id = (int8_t)Vision_ParseI32(field_start, len);
        break;
      case 3U:
        result.color2_id = (int8_t)Vision_ParseI32(field_start, len);
        break;
      case 4U:
        result.approach = (int8_t)Vision_ParseI32(field_start, len);
        break;
      case 5U:
        result.dir_x = (int8_t)Vision_ParseI32(field_start, len);
        break;
      case 6U:
        result.dir_y = (int8_t)Vision_ParseI32(field_start, len);
        break;
      default:
        break;
    }

    field_count++;
    if (*p == '\0')
    {
      break;
    }

    field++;
    p++;
    field_start = p;
  }

  if (field_count < 7U)
  {
    return 0U;
  }

  result.valid = 1U;
  Vision_CommitColorResult(&result);
  return 1U;
}

static uint8_t Vision_ParseLine(const char *line)
{
  if (line == 0)
  {
    return 0U;
  }

  if (strncmp(line, "QR|", 3U) == 0)
  {
    return Vision_ParseQrLine(line);
  }

  if (strncmp(line, "CLR|", 4U) == 0)
  {
    return Vision_ParseColorLine(line);
  }

  return 0U;
}

static void Vision_AppendChar(char *out, size_t out_size, size_t *index, char ch)
{
  if ((out == 0) || (index == 0) || (*index + 1U >= out_size))
  {
    return;
  }

  out[*index] = ch;
  (*index)++;
  out[*index] = '\0';
}

static void Vision_AppendSpace(char *out, size_t out_size, size_t *index)
{
  if ((index == 0) || (*index == 0U))
  {
    return;
  }

  if (out[*index - 1U] != ' ')
  {
    Vision_AppendChar(out, out_size, index, ' ');
  }
}

static void Vision_AppendWord(char *out, size_t out_size, size_t *index, const char *word)
{
  if ((word == 0) || (*word == '\0'))
  {
    return;
  }

  Vision_AppendSpace(out, out_size, index);
  while (*word != '\0')
  {
    Vision_AppendChar(out, out_size, index, *word);
    word++;
  }
}

static uint8_t Vision_Match3(const unsigned char *p, unsigned char a, unsigned char b, unsigned char c)
{
  return ((p[0] == a) && (p[1] == b) && (p[2] == c)) ? 1U : 0U;
}

static uint8_t Vision_Match6(const unsigned char *p,
                             unsigned char a, unsigned char b, unsigned char c,
                             unsigned char d, unsigned char e, unsigned char f)
{
  return ((p[0] == a) && (p[1] == b) && (p[2] == c) &&
          (p[3] == d) && (p[4] == e) && (p[5] == f)) ? 1U : 0U;
}

static uint8_t Vision_TryAppendColor(const unsigned char *p, char *out, size_t out_size, size_t *index)
{
  const char *word = 0;
  uint8_t used = 3U;

  if (Vision_Match6(p, 0xE7U, 0x8EU, 0xABU, 0xE7U, 0xBAU, 0xA2U) != 0U)
  {
    word = "ROSE";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE8U, 0x82U, 0x89U, 0xE8U, 0x89U, 0xB2U) != 0U)
  {
    word = "SKIN";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE8U, 0x93U, 0x9DU, 0xE7U, 0xBBU, 0xBFU) != 0U)
  {
    word = "TEAL";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE9U, 0x9DU, 0x9BU, 0xE8U, 0x93U, 0x9DU) != 0U)
  {
    word = "INDIGO";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE5U, 0x8DU, 0xA1U, 0xE5U, 0x85U, 0xB6U) != 0U)
  {
    word = "KHAKI";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE9U, 0x85U, 0x92U, 0xE7U, 0xBAU, 0xA2U) != 0U)
  {
    word = "BURGUNDY";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE6U, 0xA9U, 0x84U, 0xE6U, 0xA6U, 0x84U) != 0U)
  {
    word = "OLIVE";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE5U, 0xA2U, 0xA8U, 0xE7U, 0xBBU, 0xBFU) != 0U)
  {
    word = "DARK_GREEN";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE8U, 0x8DU, 0x89U, 0xE7U, 0xBBU, 0xBFU) != 0U)
  {
    word = "LIME";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE8U, 0x97U, 0x8FU, 0xE8U, 0x93U, 0x9DU) != 0U)
  {
    word = "NAVY";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE5U, 0xA4U, 0xA9U, 0xE8U, 0x93U, 0x9DU) != 0U)
  {
    word = "SKY_BLUE";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE6U, 0xB9U, 0x96U, 0xE8U, 0x93U, 0x9DU) != 0U)
  {
    word = "AZURE";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE6U, 0xB5U, 0x85U, 0xE7U, 0x81U, 0xB0U) != 0U)
  {
    word = "LIGHT_GRAY";
    used = 6U;
  }
  else if (Vision_Match6(p, 0xE6U, 0xB7U, 0xB1U, 0xE7U, 0x81U, 0xB0U) != 0U)
  {
    word = "DARK_GRAY";
    used = 6U;
  }
  else if (Vision_Match3(p, 0xE7U, 0xBAU, 0xA2U) != 0U)
  {
    word = "RED";
  }
  else if (Vision_Match3(p, 0xE8U, 0x93U, 0x9DU) != 0U)
  {
    word = "BLUE";
  }
  else if (Vision_Match3(p, 0xE7U, 0xBBU, 0xBFU) != 0U)
  {
    word = "GREEN";
  }
  else if (Vision_Match3(p, 0xE9U, 0xBBU, 0x84U) != 0U)
  {
    word = "YELLOW";
  }
  else if (Vision_Match3(p, 0xE7U, 0x99U, 0xBDU) != 0U)
  {
    word = "WHITE";
  }
  else if (Vision_Match3(p, 0xE9U, 0xBBU, 0x91U) != 0U)
  {
    word = "BLACK";
  }
  else if (Vision_Match3(p, 0xE7U, 0xB4U, 0xABU) != 0U)
  {
    word = "PURPLE";
  }
  else if (Vision_Match3(p, 0xE6U, 0xA9U, 0x99U) != 0U)
  {
    word = "ORANGE";
  }
  else if (Vision_Match3(p, 0xE7U, 0x81U, 0xB0U) != 0U)
  {
    word = "GRAY";
  }
  else if (Vision_Match3(p, 0xE6U, 0xA3U, 0x95U) != 0U)
  {
    word = "BROWN";
  }
  else if (Vision_Match3(p, 0xE7U, 0xB2U, 0x89U) != 0U)
  {
    word = "PINK";
  }
  else if (Vision_Match3(p, 0xE9U, 0x9DU, 0x92U) != 0U)
  {
    word = "CYAN";
  }
  else if (Vision_Match3(p, 0xE9U, 0x87U, 0x91U) != 0U)
  {
    word = "GOLD";
  }
  else if (Vision_Match3(p, 0xE9U, 0x93U, 0xB6U) != 0U)
  {
    word = "SILVER";
  }
  else if (Vision_Match3(p, 0xE7U, 0xB1U, 0xB3U) != 0U)
  {
    word = "BEIGE";
  }
  else if (Vision_Match3(p, 0xE6U, 0xA0U, 0x97U) != 0U)
  {
    word = "MAROON";
  }

  if (word == 0)
  {
    return 0U;
  }

  Vision_AppendWord(out, out_size, index, word);
  return used;
}

static uint8_t Vision_EqualsIgnoreCase(const char *text, size_t len, const char *word)
{
  size_t i;

  for (i = 0U; i < len; i++)
  {
    char a = text[i];
    char b = word[i];

    if (b == '\0')
    {
      return 0U;
    }
    if ((a >= 'a') && (a <= 'z'))
    {
      a = (char)(a - ('a' - 'A'));
    }
    if ((b >= 'a') && (b <= 'z'))
    {
      b = (char)(b - ('a' - 'A'));
    }
    if (a != b)
    {
      return 0U;
    }
  }

  return (word[len] == '\0') ? 1U : 0U;
}

static const char *Vision_MatchAsciiColor(const char *text, size_t len)
{
  static const char *const names[] =
  {
    "RED",
    "YELLOW",
    "BLUE",
    "GREEN",
    "WHITE",
    "BLACK",
    "PURPLE",
    "ORANGE",
    "GRAY",
    "GREY",
    "BROWN",
    "PINK",
    "ROSE",
    "SKIN",
    "CYAN",
    "TEAL",
    "INDIGO",
    "GOLD",
    "SILVER",
    "BEIGE",
    "KHAKI",
    "MAROON",
    "BURGUNDY",
    "OLIVE",
    "LIME",
    "NAVY",
    "AZURE"
  };
  size_t i;

  for (i = 0U; i < (sizeof(names) / sizeof(names[0])); i++)
  {
    if (Vision_EqualsIgnoreCase(text, len, names[i]) != 0U)
    {
      return (Vision_EqualsIgnoreCase(text, len, "GREY") != 0U) ? "GRAY" : names[i];
    }
  }

  return 0;
}

static uint8_t Vision_CollectColorsOnly(const char *raw, char *out, size_t out_size, size_t *index, uint8_t color_count)
{
  size_t i = 0U;

  if ((raw == 0) || (index == 0))
  {
    return color_count;
  }

  while ((raw[i] != '\0') && (color_count < 2U))
  {
    unsigned char ch = (unsigned char)raw[i];

    if (((ch >= 'A') && (ch <= 'Z')) || ((ch >= 'a') && (ch <= 'z')))
    {
      const char *word_start = &raw[i];
      const char *color;
      size_t len = 0U;

      while (((raw[i] >= 'A') && (raw[i] <= 'Z')) ||
             ((raw[i] >= 'a') && (raw[i] <= 'z')) ||
             (raw[i] == '_'))
      {
        i++;
        len++;
      }

      color = Vision_MatchAsciiColor(word_start, len);
      if (color != 0)
      {
        Vision_AppendWord(out, out_size, index, color);
        color_count++;
      }
    }
    else if (((ch & 0xF0U) == 0xE0U) && (raw[i + 1U] != '\0') && (raw[i + 2U] != '\0'))
    {
      uint8_t used = Vision_TryAppendColor((const unsigned char *)&raw[i], out, out_size, index);
      if (used != 0U)
      {
        i += used;
        color_count++;
      }
      else
      {
        i += 3U;
      }
    }
    else if (((ch & 0xE0U) == 0xC0U) && (raw[i + 1U] != '\0'))
    {
      i += 2U;
    }
    else
    {
      i++;
    }
  }

  return color_count;
}

static uint8_t Vision_FormatColorsOnly(const char *raw, char *out, size_t out_size)
{
  size_t index = 0U;

  if ((out != 0) && (out_size > 0U))
  {
    out[0] = '\0';
  }

  return Vision_CollectColorsOnly(raw, out, out_size, &index, 0U);
}

static uint8_t Vision_ResultColorCount(const VisionResult_t *result)
{
  size_t index = 0U;
  uint8_t color_count = 0U;

  if (result == 0)
  {
    return 0U;
  }

  color_count = Vision_CollectColorsOnly(result->qr_list, 0, 0U, &index, color_count);
  color_count = Vision_CollectColorsOnly(result->qr_tl, 0, 0U, &index, color_count);
  color_count = Vision_CollectColorsOnly(result->qr_tr, 0, 0U, &index, color_count);
  color_count = Vision_CollectColorsOnly(result->qr_bl, 0, 0U, &index, color_count);
  color_count = Vision_CollectColorsOnly(result->qr_br, 0, 0U, &index, color_count);

  return color_count;
}

static void Vision_FormatHexPrefix(const char *raw, char *out, size_t out_size)
{
  static const char hex[] = "0123456789ABCDEF";
  size_t i;
  size_t index = 0U;

  if ((out == 0) || (out_size == 0U))
  {
    return;
  }

  out[0] = '\0';
  Vision_AppendWord(out, out_size, &index, "HEX");
  for (i = 0U; (raw[i] != '\0') && (i < 5U); i++)
  {
    unsigned char value = (unsigned char)raw[i];
    Vision_AppendSpace(out, out_size, &index);
    Vision_AppendChar(out, out_size, &index, hex[value >> 4]);
    Vision_AppendChar(out, out_size, &index, hex[value & 0x0FU]);
  }
}

static void Vision_FormatVisible(const char *raw, char *out, size_t out_size)
{
  size_t i = 0U;
  size_t index = 0U;

  if ((out == 0) || (out_size == 0U))
  {
    return;
  }

  out[0] = '\0';
  if (raw == 0)
  {
    return;
  }

  while ((raw[i] != '\0') && (index + 1U < out_size))
  {
    unsigned char ch = (unsigned char)raw[i];

    if ((ch >= 'a') && (ch <= 'z'))
    {
      Vision_AppendChar(out, out_size, &index, (char)(ch - ('a' - 'A')));
      i++;
    }
    else if (((ch >= 'A') && (ch <= 'Z')) || ((ch >= '0') && (ch <= '9')))
    {
      Vision_AppendChar(out, out_size, &index, (char)ch);
      i++;
    }
    else if ((ch == ',') || (ch == '|') || (ch == ';') || (ch == ' ') ||
             (ch == '-') || (ch == '_') || (ch == '+') || (ch == '/'))
    {
      Vision_AppendSpace(out, out_size, &index);
      i++;
    }
    else if (((ch & 0xF0U) == 0xE0U) && (raw[i + 1U] != '\0') && (raw[i + 2U] != '\0'))
    {
      uint8_t used = Vision_TryAppendColor((const unsigned char *)&raw[i], out, out_size, &index);
      if (used != 0U)
      {
        i += used;
      }
      else
      {
        i += 3U;
      }
    }
    else if (((ch & 0xE0U) == 0xC0U) && (raw[i + 1U] != '\0'))
    {
      i += 2U;
    }
    else
    {
      i++;
    }
  }

  while ((index > 0U) && (out[index - 1U] == ' '))
  {
    index--;
    out[index] = '\0';
  }

  if ((out[0] == '\0') && (raw[0] != '\0'))
  {
    Vision_FormatHexPrefix(raw, out, out_size);
  }
}

static const char *Vision_ColorName(int8_t id)
{
  static const char *const names[] =
  {
    "NONE",
    "RED",
    "YELLOW",
    "BLUE",
    "GREEN",
    "WHITE",
    "BLACK",
    "PURPLE",
    "ORANGE",
    "GRAY",
    "BROWN",
    "PINK",
    "ROSE",
    "SKIN",
    "CYAN",
    "TEAL",
    "INDIGO",
    "GOLD",
    "SILVER",
    "BEIGE",
    "KHAKI",
    "MAROON",
    "BURGUNDY",
    "OLIVE",
    "DARK_GREEN",
    "LIME",
    "NAVY",
    "SKY_BLUE",
    "AZURE",
    "LIGHT_GRAY",
    "DARK_GRAY"
  };

  if (id == VISION_COLOR_NO_CAMERA)
  {
    return "NO_CAMERA";
  }
  if ((id < 0) || (id > 30))
  {
    return "BAD_ID";
  }

  return names[(uint8_t)id];
}

static uint8_t Vision_IsNoCameraColorFrame(const VisionColorResult_t *result)
{
  if (result == 0)
  {
    return 0U;
  }

  return ((result->frame_no == 0U) &&
          (result->color1_id == VISION_COLOR_NO_CAMERA) &&
          (result->color2_id == VISION_COLOR_NO_CAMERA) &&
          (result->approach == -1)) ? 1U : 0U;
}

HAL_StatusTypeDef VisionComm_Init(UART_HandleTypeDef *huart)
{
  s_uart = huart;
  s_rx_index = 0U;
  s_drop_line = 0U;
  (void)memset(s_rx_line, 0, sizeof(s_rx_line));
  (void)memset(&s_latest, 0, sizeof(s_latest));
  (void)memset(&s_latest_color, 0, sizeof(s_latest_color));
  (void)memset(&s_stats, 0, sizeof(s_stats));

  return VisionComm_StartReceive();
}

void VisionComm_OnRxCplt(UART_HandleTypeDef *huart)
{
  if (VisionComm_IsThisUart(huart) == 0U)
  {
    return;
  }

  s_stats.byte_count++;
  s_stats.last_rx_tick = HAL_GetTick();

  if (s_rx_byte == '\r')
  {
    (void)VisionComm_StartReceive();
    return;
  }

  if (s_rx_byte == '\n')
  {
    if (s_drop_line == 0U)
    {
      s_rx_line[s_rx_index] = '\0';
      s_stats.line_count++;
      if ((s_rx_index > 0U) && (Vision_ParseLine(s_rx_line) == 0U))
      {
        s_stats.parse_error_count++;
      }
    }
    s_rx_index = 0U;
    s_drop_line = 0U;
    (void)VisionComm_StartReceive();
    return;
  }

  if (s_drop_line != 0U)
  {
    (void)VisionComm_StartReceive();
    return;
  }

  if (s_rx_index < (VISION_LINE_BUF_SIZE - 1U))
  {
    s_rx_line[s_rx_index] = (char)s_rx_byte;
    s_rx_index++;
  }
  else
  {
    s_rx_index = 0U;
    s_drop_line = 1U;
    s_stats.overflow_count++;
  }

  (void)VisionComm_StartReceive();
}

void VisionComm_OnError(UART_HandleTypeDef *huart)
{
  if (VisionComm_IsThisUart(huart) == 0U)
  {
    return;
  }

  s_stats.uart_error_count++;
  (void)VisionComm_StartReceive();
}

uint8_t VisionComm_GetLatest(VisionResult_t *result)
{
  uint8_t valid;

  if (result == 0)
  {
    return 0U;
  }

  __disable_irq();
  *result = s_latest;
  valid = s_latest.valid;
  __enable_irq();

  return valid;
}

uint8_t VisionComm_GetLatestColor(VisionColorResult_t *result)
{
  uint8_t valid;

  if (result == 0)
  {
    return 0U;
  }

  __disable_irq();
  *result = s_latest_color;
  valid = s_latest_color.valid;
  __enable_irq();

  return valid;
}

void VisionComm_GetStats(VisionCommStats_t *stats)
{
  if (stats == 0)
  {
    return;
  }

  __disable_irq();
  *stats = s_stats;
  __enable_irq();
}

void VisionComm_FormatQrText(const VisionResult_t *result, char *out, size_t out_size)
{
  if ((result == 0) || (result->qr_list[0] == '\0'))
  {
    Vision_CopyToken(out, out_size, "", 0U);
    return;
  }

  (void)Vision_FormatColorsOnly(result->qr_list, out, out_size);
}

uint8_t VisionComm_FormatQrColors(const VisionResult_t *result, char *out, size_t out_size)
{
  size_t index = 0U;
  uint8_t color_count = 0U;

  if ((out != 0) && (out_size > 0U))
  {
    out[0] = '\0';
  }
  if (result == 0)
  {
    return 0U;
  }

  color_count = Vision_CollectColorsOnly(result->qr_list, out, out_size, &index, color_count);
  color_count = Vision_CollectColorsOnly(result->qr_tl, out, out_size, &index, color_count);
  color_count = Vision_CollectColorsOnly(result->qr_tr, out, out_size, &index, color_count);
  color_count = Vision_CollectColorsOnly(result->qr_bl, out, out_size, &index, color_count);
  color_count = Vision_CollectColorsOnly(result->qr_br, out, out_size, &index, color_count);

  return color_count;
}

void VisionComm_FormatQuadrantText(const VisionResult_t *result, char *out, size_t out_size)
{
  char part[VISION_COMM_FIELD_SIZE];
  size_t index = 0U;

  if ((out == 0) || (out_size == 0U))
  {
    return;
  }

  out[0] = '\0';
  if (result == 0)
  {
    return;
  }

  if (result->qr_tl[0] != '\0')
  {
    Vision_FormatVisible(result->qr_tl, part, sizeof(part));
    Vision_AppendWord(out, out_size, &index, "TL");
    Vision_AppendWord(out, out_size, &index, part);
  }
  if (result->qr_tr[0] != '\0')
  {
    Vision_FormatVisible(result->qr_tr, part, sizeof(part));
    Vision_AppendWord(out, out_size, &index, "TR");
    Vision_AppendWord(out, out_size, &index, part);
  }
  if ((out[0] == '\0') && (result->qr_bl[0] != '\0'))
  {
    Vision_FormatVisible(result->qr_bl, part, sizeof(part));
    Vision_AppendWord(out, out_size, &index, "BL");
    Vision_AppendWord(out, out_size, &index, part);
  }
  if ((out[0] == '\0') && (result->qr_br[0] != '\0'))
  {
    Vision_FormatVisible(result->qr_br, part, sizeof(part));
    Vision_AppendWord(out, out_size, &index, "BR");
    Vision_AppendWord(out, out_size, &index, part);
  }
  if (out[0] == '\0')
  {
    Vision_CopyToken(out, out_size, "NO POS", 6U);
  }
}

void VisionComm_FormatColorText(const VisionColorResult_t *result, char *out, size_t out_size)
{
  if ((out == 0) || (out_size == 0U))
  {
    return;
  }

  out[0] = '\0';
  if ((result == 0) || (result->valid == 0U))
  {
    Vision_CopyToken(out, out_size, "CLR NO DATA", 11U);
    return;
  }

  if (Vision_IsNoCameraColorFrame(result) != 0U)
  {
    Vision_CopyToken(out, out_size, "NO CAMERA", 9U);
    return;
  }

  (void)snprintf(out, out_size, "C:%s %s",
                 Vision_ColorName(result->color1_id),
                 Vision_ColorName(result->color2_id));
}

void VisionComm_FormatColorStateText(const VisionColorResult_t *result, char *out, size_t out_size)
{
  const char *state = "UNKNOWN";

  if ((out == 0) || (out_size == 0U))
  {
    return;
  }

  out[0] = '\0';
  if ((result == 0) || (result->valid == 0U))
  {
    Vision_CopyToken(out, out_size, "STATE:NO DATA", 13U);
    return;
  }

  if (Vision_IsNoCameraColorFrame(result) != 0U)
  {
    state = "NO CAMERA";
  }
  else if (result->approach == -1)
  {
    state = "LOCKED";
  }
  else if (result->approach == 1)
  {
    state = "NEAR";
  }
  else if (result->approach == 0)
  {
    if ((result->color1_id != VISION_COLOR_NONE) && (result->color2_id != VISION_COLOR_NONE))
    {
      state = "COLOR OK";
    }
    else
    {
      state = "NO TARGET";
    }
  }

  (void)snprintf(out, out_size, "STATE:%s", state);
}

void VisionComm_FormatDirectionText(const VisionColorResult_t *result, char *out, size_t out_size)
{
  if ((out == 0) || (out_size == 0U))
  {
    return;
  }

  out[0] = '\0';
  if ((result == 0) || (result->valid == 0U))
  {
    Vision_CopyToken(out, out_size, "DIR:NO DATA", 11U);
    return;
  }

  (void)snprintf(out, out_size, "DIR X%+d Y%+d",
                 (int)result->dir_x,
                 (int)result->dir_y);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  VisionComm_OnRxCplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  VisionComm_OnError(huart);
}
