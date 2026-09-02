#include "chassis.h"

#include "app_config.h"
#include "main.h"

typedef struct
{
  float integral;
} SpeedPi_t;

static TIM_HandleTypeDef *s_pwm_timer;
static TIM_HandleTypeDef *s_left_encoder;
static TIM_HandleTypeDef *s_right_encoder;
static uint32_t s_left_last;
static uint32_t s_right_last;
static SpeedPi_t s_left_pi;
static SpeedPi_t s_right_pi;
static ChassisDebug_t s_debug;
static uint8_t s_speed_filter_ready;

volatile float g_speed_pi_kp = SPEED_PI_KP;
volatile float g_speed_pi_ki = SPEED_PI_KI;

static int16_t ClampDuty(int32_t duty)
{
  if (duty > MOTOR_DUTY_MAX)
  {
    duty = MOTOR_DUTY_MAX;
  }
  else if (duty < -MOTOR_DUTY_MAX)
  {
    duty = -MOTOR_DUTY_MAX;
  }
  return (int16_t)duty;
}

static int16_t SpeedPiStep(SpeedPi_t *pi, int16_t target, float measured)
{
  float error = (float)target - measured;
  float output;

  pi->integral += error;
  if (pi->integral > SPEED_PI_INTEGRAL_LIMIT)
  {
    pi->integral = SPEED_PI_INTEGRAL_LIMIT;
  }
  else if (pi->integral < -SPEED_PI_INTEGRAL_LIMIT)
  {
    pi->integral = -SPEED_PI_INTEGRAL_LIMIT;
  }

  output = g_speed_pi_kp * error + g_speed_pi_ki * pi->integral;
  return ClampDuty((int32_t)output);
}

static int32_t ReadDelta(TIM_HandleTypeDef *timer, uint32_t *last)
{
  uint32_t current = __HAL_TIM_GET_COUNTER(timer);
  int32_t delta;

  if (__HAL_TIM_GET_AUTORELOAD(timer) > 0xFFFFU)
  {
    delta = (int32_t)(current - *last);
  }
  else
  {
    delta = (int16_t)((uint16_t)current - (uint16_t)(*last));
  }
  *last = current;
  return delta;
}

static void SetDirection(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                         GPIO_TypeDef *in2_port, uint16_t in2_pin,
                         int16_t duty)
{
  if (duty > 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
  }
}

static void ApplyMotor(int16_t left_duty, int16_t right_duty)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(s_pwm_timer) + 1U;
  uint32_t left_compare;
  uint32_t right_compare;

  left_duty = (int16_t)(left_duty * CHASSIS_LEFT_MOTOR_SIGN);
  right_duty = (int16_t)(right_duty * CHASSIS_RIGHT_MOTOR_SIGN);
  left_compare = (uint32_t)((((left_duty < 0) ? -left_duty : left_duty)
                            * (int32_t)period) / MOTOR_DUTY_MAX);
  right_compare = (uint32_t)((((right_duty < 0) ? -right_duty : right_duty)
                             * (int32_t)period) / MOTOR_DUTY_MAX);

  __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_2, 0U);
  SetDirection(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin,
               MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, left_duty);
  SetDirection(MOTOR_R_IN1_GPIO_Port, MOTOR_R_IN1_Pin,
               MOTOR_R_IN2_GPIO_Port, MOTOR_R_IN2_Pin, right_duty);
  __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_1, left_compare);
  __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_2, right_compare);
}

HAL_StatusTypeDef Chassis_Init(TIM_HandleTypeDef *pwm_timer,
                               TIM_HandleTypeDef *left_encoder,
                               TIM_HandleTypeDef *right_encoder)
{
  HAL_StatusTypeDef status;

  s_pwm_timer = pwm_timer;
  s_left_encoder = left_encoder;
  s_right_encoder = right_encoder;

  status = HAL_TIM_PWM_Start(s_pwm_timer, TIM_CHANNEL_1);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_TIM_PWM_Start(s_pwm_timer, TIM_CHANNEL_2);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_TIM_Encoder_Start(s_left_encoder, TIM_CHANNEL_ALL);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_TIM_Encoder_Start(s_right_encoder, TIM_CHANNEL_ALL);
  if (status != HAL_OK)
  {
    return status;
  }

  Chassis_Reset();
  Chassis_Coast();
  return HAL_OK;
}

void Chassis_Reset(void)
{
  s_left_pi.integral = 0.0f;
  s_right_pi.integral = 0.0f;
  s_debug.left_target = 0;
  s_debug.right_target = 0;
  s_debug.left_speed_raw = 0;
  s_debug.right_speed_raw = 0;
  s_debug.left_speed = 0.0f;
  s_debug.right_speed = 0.0f;
  s_debug.left_duty = 0;
  s_debug.right_duty = 0;
  s_debug.left_distance = 0;
  s_debug.right_distance = 0;
  s_left_last = __HAL_TIM_GET_COUNTER(s_left_encoder);
  s_right_last = __HAL_TIM_GET_COUNTER(s_right_encoder);
  s_speed_filter_ready = 0U;
}

void Chassis_SetSpeedTarget(int16_t left, int16_t right)
{
  s_debug.left_target = left;
  s_debug.right_target = right;
}

void Chassis_ControlStep(void)
{
  int32_t left_delta = ReadDelta(s_left_encoder, &s_left_last)
                       * CHASSIS_LEFT_ENCODER_SIGN;
  int32_t right_delta = ReadDelta(s_right_encoder, &s_right_last)
                        * CHASSIS_RIGHT_ENCODER_SIGN;

  s_debug.left_speed_raw = (int16_t)left_delta;
  s_debug.right_speed_raw = (int16_t)right_delta;
  if (s_speed_filter_ready == 0U)
  {
    s_debug.left_speed = (float)left_delta;
    s_debug.right_speed = (float)right_delta;
    s_speed_filter_ready = 1U;
  }
  else
  {
    s_debug.left_speed += SPEED_FILTER_ALPHA
                          * ((float)left_delta - s_debug.left_speed);
    s_debug.right_speed += SPEED_FILTER_ALPHA
                           * ((float)right_delta - s_debug.right_speed);
  }
  s_debug.left_distance += left_delta;
  s_debug.right_distance += right_delta;
  s_debug.left_duty = SpeedPiStep(&s_left_pi, s_debug.left_target,
                                  s_debug.left_speed);
  s_debug.right_duty = SpeedPiStep(&s_right_pi, s_debug.right_target,
                                   s_debug.right_speed);
  ApplyMotor(s_debug.left_duty, s_debug.right_duty);
}

void Chassis_Coast(void)
{
  s_debug.left_target = 0;
  s_debug.right_target = 0;
  s_debug.left_duty = 0;
  s_debug.right_duty = 0;
  s_left_pi.integral = 0.0f;
  s_right_pi.integral = 0.0f;
  ApplyMotor(0, 0);
}

void Chassis_Brake(void)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(s_pwm_timer) + 1U;

  s_debug.left_target = 0;
  s_debug.right_target = 0;
  s_debug.left_duty = MOTOR_DUTY_MAX;
  s_debug.right_duty = MOTOR_DUTY_MAX;
  s_left_pi.integral = 0.0f;
  s_right_pi.integral = 0.0f;
  HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_R_IN1_GPIO_Port, MOTOR_R_IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_R_IN2_GPIO_Port, MOTOR_R_IN2_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_1, period);
  __HAL_TIM_SET_COMPARE(s_pwm_timer, TIM_CHANNEL_2, period);
}

const ChassisDebug_t *Chassis_GetDebug(void)
{
  return &s_debug;
}

int32_t Chassis_GetAverageDistance(void)
{
  return (s_debug.left_distance + s_debug.right_distance) / 2;
}
