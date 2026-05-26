#include "app_motor.h"
#include "main.h"

#define MOTOR_U1_DIR_SIGN   -1
#define MOTOR_U2_DIR_SIGN   1
#define MOTOR_U3_DIR_SIGN   -1
#define MOTOR_U4_DIR_SIGN   1
#define MOTOR_COUNT         4U

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
  int8_t direction_sign;
} Motor_Channel_t;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static const Motor_Channel_t g_motors[MOTOR_COUNT] =
{
  {&htim2, TIM_CHANNEL_1, GPIOD, GPIO_PIN_10, GPIOD, GPIO_PIN_11, MOTOR_U1_DIR_SIGN},
  {&htim2, TIM_CHANNEL_2, GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, MOTOR_U2_DIR_SIGN},
  {&htim2, TIM_CHANNEL_3, GPIOE, GPIO_PIN_12, GPIOE, GPIO_PIN_13, MOTOR_U3_DIR_SIGN},
  {&htim2, TIM_CHANNEL_4, GPIOE, GPIO_PIN_14, GPIOE, GPIO_PIN_15, MOTOR_U4_DIR_SIGN}
};

void Motor_StartAllPwm(void)
{
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void Motor_SetOutput(uint8_t motor_id, int32_t pwm)
{
  const Motor_Channel_t *motor;
  uint32_t pulse;

  if (motor_id >= MOTOR_COUNT)
  {
    return;
  }

  motor = &g_motors[motor_id];
  pwm *= motor->direction_sign;

  if (pwm > (int32_t)motor->htim->Init.Period)
  {
    pwm = (int32_t)motor->htim->Init.Period;//Clamp the PWM value to the maximum period
  }
  else if (pwm < -(int32_t)motor->htim->Init.Period)
  {
    pwm = -(int32_t)motor->htim->Init.Period;//Clamp the PWM value to the minimum period
  }

  pulse = (pwm >= 0) ? (uint32_t)pwm : (uint32_t)(-pwm);

  if (pwm > 0)
  {
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
  }
  else if (pwm < 0)
  {
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
  }

  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pulse);//Set the PWM compare value
}

void Motor_StopAll(void)
{
  uint8_t motor_id;

  for (motor_id = 0; motor_id < MOTOR_COUNT; motor_id++)
  {
    Motor_SetOutput(motor_id, 0);
  }
}
