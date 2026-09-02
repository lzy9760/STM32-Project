#ifndef APP_H
#define APP_H

#include "stm32f4xx_hal.h"

typedef enum
{
  APP_WAIT_START = 0,
  APP_BENCH_SPEED_PI,
  APP_LEAVE_START_LINE,
  APP_RUNNING,
  APP_APPROACH_STOP,
  APP_FINISHED,
  APP_ERROR
} AppState_t;

typedef struct
{
  AppState_t state;
  uint32_t elapsed_ms;
  uint32_t final_time_ms;
  int16_t line_error;
  uint16_t line_strength;
  uint16_t line_peak;
  uint8_t line_black_count;
  uint8_t max_black_count;
  uint8_t cross_detected;
  uint8_t line_detected;
  uint8_t finish_left;
  uint8_t finish_right;
  uint8_t oled_present;
  int16_t left_target;
  int16_t right_target;
  int16_t left_speed_raw;
  int16_t right_speed_raw;
  float left_speed;
  float right_speed;
  int16_t left_duty;
  int16_t right_duty;
} AppDebug_t;

HAL_StatusTypeDef App_Init(ADC_HandleTypeDef *hadc,
                           TIM_HandleTypeDef *adc_trigger_timer,
                           TIM_HandleTypeDef *pwm_timer,
                           TIM_HandleTypeDef *left_encoder,
                           TIM_HandleTypeDef *right_encoder,
                           I2C_HandleTypeDef *display_i2c,
                           UART_HandleTypeDef *debug_uart);
void App_Run(void);
const AppDebug_t *App_GetDebug(void);

extern volatile AppDebug_t g_app_debug;
extern volatile int16_t g_bench_left_target;
extern volatile int16_t g_bench_right_target;

#endif
