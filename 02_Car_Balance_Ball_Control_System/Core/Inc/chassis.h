#ifndef CHASSIS_H
#define CHASSIS_H

#include "stm32f4xx_hal.h"

typedef struct
{
  int16_t left_target;
  int16_t right_target;
  int16_t left_speed_raw;
  int16_t right_speed_raw;
  float left_speed;
  float right_speed;
  int16_t left_duty;
  int16_t right_duty;
  int32_t left_distance;
  int32_t right_distance;
} ChassisDebug_t;

HAL_StatusTypeDef Chassis_Init(TIM_HandleTypeDef *pwm_timer,
                               TIM_HandleTypeDef *left_encoder,
                               TIM_HandleTypeDef *right_encoder);
void Chassis_Reset(void);
void Chassis_SetSpeedTarget(int16_t left, int16_t right);
void Chassis_ControlStep(void);
void Chassis_Coast(void);
void Chassis_Brake(void);
const ChassisDebug_t *Chassis_GetDebug(void);
int32_t Chassis_GetAverageDistance(void);

/* Change these two values directly in Keil Watch while tuning. */
extern volatile float g_speed_pi_kp;
extern volatile float g_speed_pi_ki;

#endif
