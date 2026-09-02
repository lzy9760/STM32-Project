#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "stm32f4xx_hal.h"
#include "app_config.h"

typedef struct
{
  uint16_t raw[LINE_SENSOR_COUNT];
  uint16_t filtered[LINE_SENSOR_COUNT];
  uint16_t gray[LINE_SENSOR_COUNT];
  int16_t error;
  uint16_t strength;
  uint16_t peak;
  uint8_t black_count;
  uint8_t cross_detected;
  uint8_t detected;
} LineSensorFrame_t;

HAL_StatusTypeDef LineSensor_Init(ADC_HandleTypeDef *hadc,
                                  TIM_HandleTypeDef *trigger_timer);
void LineSensor_Update(void);
const LineSensorFrame_t *LineSensor_GetFrame(void);
void LineSensor_SetCalibration(const uint16_t white[LINE_SENSOR_COUNT],
                               const uint16_t black[LINE_SENSOR_COUNT]);

extern volatile uint16_t g_line_sensor_dma[LINE_SENSOR_COUNT];
extern uint16_t g_line_white[LINE_SENSOR_COUNT];
extern uint16_t g_line_black[LINE_SENSOR_COUNT];

#endif
