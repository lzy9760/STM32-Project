#include "line_sensor.h"

#include <string.h>

static ADC_HandleTypeDef *s_hadc;
static TIM_HandleTypeDef *s_trigger_timer;
static LineSensorFrame_t s_frame;
static uint8_t s_filter_ready;

volatile uint16_t g_line_sensor_dma[LINE_SENSOR_COUNT];
uint16_t g_line_white[LINE_SENSOR_COUNT] = LINE_WHITE_DEFAULT;
uint16_t g_line_black[LINE_SENSOR_COUNT] = LINE_BLACK_DEFAULT;

static const int16_t s_weight[LINE_SENSOR_COUNT] =
{
  -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

static uint16_t Normalize(uint16_t raw, uint16_t white, uint16_t black)
{
  int32_t span = (int32_t)black - (int32_t)white;
  int32_t value;

  if ((span < LINE_SENSOR_MIN_SPAN) && (span > -LINE_SENSOR_MIN_SPAN))
  {
    return 0U;
  }

  value = ((int32_t)raw - (int32_t)white) * 1000 / span;
  if (value < 0)
  {
    value = 0;
  }
  else if (value > 1000)
  {
    value = 1000;
  }
  return (uint16_t)value;
}

HAL_StatusTypeDef LineSensor_Init(ADC_HandleTypeDef *hadc,
                                  TIM_HandleTypeDef *trigger_timer)
{
  HAL_StatusTypeDef status;

  s_hadc = hadc;
  s_trigger_timer = trigger_timer;
  s_filter_ready = 0U;
  memset(&s_frame, 0, sizeof(s_frame));

  status = HAL_ADC_Start_DMA(s_hadc, (uint32_t *)g_line_sensor_dma,
                             LINE_SENSOR_COUNT);
  if (status != HAL_OK)
  {
    return status;
  }
  return HAL_TIM_Base_Start(s_trigger_timer);
}

void LineSensor_Update(void)
{
  int32_t weighted_sum = 0;
  uint32_t strength = 0U;
  uint16_t peak = 0U;
  uint8_t black_count = 0U;
  uint32_t i;

  for (i = 0U; i < LINE_SENSOR_COUNT; ++i)
  {
    uint16_t raw = g_line_sensor_dma[i];
    uint16_t raw_gray;
    int32_t filtered;

    s_frame.raw[i] = raw;
    raw_gray = Normalize(raw, g_line_white[i], g_line_black[i]);
    if (s_filter_ready == 0U)
    {
      s_frame.filtered[i] = raw;
    }
    else//低通滤波
    {
      filtered = (int32_t)s_frame.filtered[i];
      filtered += ((int32_t)raw - filtered) >> LINE_SENSOR_FILTER_SHIFT;
      s_frame.filtered[i] = (uint16_t)filtered;
    }

    s_frame.gray[i] = Normalize(s_frame.filtered[i],
                                g_line_white[i], g_line_black[i]);
    if (s_frame.gray[i] > peak)
    {
      peak = s_frame.gray[i];
    }
    if (raw_gray >= LINE_CROSS_SENSOR_LEVEL)
    {
      ++black_count;
    }
    weighted_sum += (int32_t)s_weight[i] * (int32_t)s_frame.gray[i];
    strength += s_frame.gray[i];
  }

  s_filter_ready = 1U;
  s_frame.strength = (strength > 65535U) ? 65535U : (uint16_t)strength;
  s_frame.peak = peak;
  s_frame.black_count = black_count;
  s_frame.cross_detected =
      (black_count >= LINE_CROSS_MIN_SENSORS) ? 1U : 0U;
  if (peak >= LINE_SENSOR_DETECT_LEVEL)
  {
    s_frame.error = (int16_t)(weighted_sum / (int32_t)strength);
    s_frame.detected = 1U;
  }
  else
  {
    s_frame.error = 0;
    s_frame.detected = 0U;
  }
}

const LineSensorFrame_t *LineSensor_GetFrame(void)
{
  return &s_frame;
}

void LineSensor_SetCalibration(const uint16_t white[LINE_SENSOR_COUNT],
                               const uint16_t black[LINE_SENSOR_COUNT])
{
  memcpy(g_line_white, white, sizeof(g_line_white));
  memcpy(g_line_black, black, sizeof(g_line_black));
}
