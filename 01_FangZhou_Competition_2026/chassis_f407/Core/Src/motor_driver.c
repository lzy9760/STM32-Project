#include "motor_driver.h"

#include "main.h"

#define MOTOR_DIRECTION_DEADTIME_CYCLES 1000U
#define MOTOR_COMMAND_DEADBAND_PERCENT  25
#define MOTOR_MIN_EFFECTIVE_DUTY_PERCENT 35
#define MOTOR_FULL_DUTY_THRESHOLD_PERCENT 95
#define MOTOR_REVERSE_COAST_MS          80U
#define MOTOR_DUTY_RAMP_INTERVAL_MS     2U
#define MOTOR_DUTY_RAMP_STEP_PERCENT    2
#define MOTOR_DUTY_HOLD_BAND_PERCENT    2

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
} Motor_Channel_t;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static const Motor_Channel_t s_motors[MOTOR_COUNT] =
{
  {&htim3, TIM_CHANNEL_1, GPIOF, GPIO_PIN_10, GPIOE, GPIO_PIN_6 },
  {&htim3, TIM_CHANNEL_2, GPIOG, GPIO_PIN_0,  GPIOG, GPIO_PIN_1 },
  {&htim4, TIM_CHANNEL_1, GPIOF, GPIO_PIN_2,  GPIOE, GPIO_PIN_5 },
  {&htim4, TIM_CHANNEL_2, GPIOG, GPIO_PIN_2,  GPIOG, GPIO_PIN_3 }
};

static int16_t s_motor_duty[MOTOR_COUNT];
static int8_t s_motor_direction[MOTOR_COUNT];
static uint32_t s_motor_reverse_ready_tick[MOTOR_COUNT];
static uint32_t s_motor_ramp_tick[MOTOR_COUNT];

static int16_t MotorDriver_ClampDuty(int16_t duty_percent)
{
  if (duty_percent > 100)
  {
    return 100;
  }
  if (duty_percent < -100)
  {
    return -100;
  }
  return duty_percent;
}

static int16_t MotorDriver_ApplyEffectiveDuty(int16_t duty_percent)
{
  int16_t abs_duty;

  if (duty_percent == 0)
  {
    return 0;
  }

  abs_duty = (duty_percent > 0) ? duty_percent : (int16_t)-duty_percent;
  if (abs_duty < MOTOR_COMMAND_DEADBAND_PERCENT)
  {
    return 0;
  }
  if (abs_duty < MOTOR_MIN_EFFECTIVE_DUTY_PERCENT)
  {
    return (duty_percent > 0) ? MOTOR_MIN_EFFECTIVE_DUTY_PERCENT : (int16_t)-MOTOR_MIN_EFFECTIVE_DUTY_PERCENT;
  }
  if (abs_duty >= MOTOR_FULL_DUTY_THRESHOLD_PERCENT)
  {
    return (duty_percent > 0) ? 100 : -100;
  }

  return duty_percent;
}

static int8_t MotorDriver_GetDirection(int16_t duty_percent)
{
  if (duty_percent > 0)
  {
    return 1;
  }
  if (duty_percent < 0)
  {
    return -1;
  }
  return 0;
}

static int16_t MotorDriver_AbsDuty(int16_t duty_percent)
{
  return (duty_percent < 0) ? (int16_t)-duty_percent : duty_percent;
}

static int16_t MotorDriver_SignedDuty(int8_t direction, int16_t duty_percent)
{
  return (direction > 0) ? duty_percent : (int16_t)-duty_percent;
}

static uint32_t MotorDriver_DutyToPulse(const Motor_Channel_t *motor, int16_t duty_percent)
{
  uint32_t timer_period;

  timer_period = __HAL_TIM_GET_AUTORELOAD(motor->htim) + 1U;
  return ((uint32_t)MotorDriver_AbsDuty(duty_percent) * timer_period) / 100U;
}

static int16_t MotorDriver_ApplyRamp(uint8_t motor_id, int16_t target_duty)
{
  uint32_t now;
  int16_t current_abs;
  int16_t target_abs;
  int16_t next_abs;
  int8_t target_direction;
  int8_t current_direction;

  target_direction = MotorDriver_GetDirection(target_duty);
  if (target_direction == 0)
  {
    return 0;
  }

  now = HAL_GetTick();
  current_direction = MotorDriver_GetDirection(s_motor_duty[motor_id]);
  current_abs = (current_direction == target_direction) ? MotorDriver_AbsDuty(s_motor_duty[motor_id]) : 0;
  target_abs = MotorDriver_AbsDuty(target_duty);

  if (current_abs == 0)
  {
    s_motor_ramp_tick[motor_id] = now;
    next_abs = (target_abs > MOTOR_MIN_EFFECTIVE_DUTY_PERCENT) ? MOTOR_MIN_EFFECTIVE_DUTY_PERCENT : target_abs;
    return MotorDriver_SignedDuty(target_direction, next_abs);
  }

  if ((target_abs == current_abs) ||
      ((target_abs < 100) &&
       (((target_abs > current_abs) ? (target_abs - current_abs) : (current_abs - target_abs)) <=
        MOTOR_DUTY_HOLD_BAND_PERCENT)))
  {
    s_motor_ramp_tick[motor_id] = now;
    return MotorDriver_SignedDuty(target_direction, current_abs);
  }

  if ((uint32_t)(now - s_motor_ramp_tick[motor_id]) < MOTOR_DUTY_RAMP_INTERVAL_MS)
  {
    return s_motor_duty[motor_id];
  }

  s_motor_ramp_tick[motor_id] = now;
  if (target_abs > current_abs)
  {
    next_abs = (int16_t)(current_abs + MOTOR_DUTY_RAMP_STEP_PERCENT);
    if (next_abs > target_abs)
    {
      next_abs = target_abs;
    }
  }
  else if (current_abs > MOTOR_DUTY_RAMP_STEP_PERCENT)
  {
    next_abs = (int16_t)(current_abs - MOTOR_DUTY_RAMP_STEP_PERCENT);
    if (next_abs < target_abs)
    {
      next_abs = target_abs;
    }
  }
  else
  {
    next_abs = target_abs;
  }

  return MotorDriver_SignedDuty(target_direction, next_abs);
}

static void MotorDriver_Deadtime(void)
{
  volatile uint32_t delay;

  for (delay = 0U; delay < MOTOR_DIRECTION_DEADTIME_CYCLES; delay++)
  {
    __NOP();
  }
}

static uint8_t MotorDriver_TickExpired(uint32_t tick)
{
  return (((int32_t)(HAL_GetTick() - tick)) >= 0) ? 1U : 0U;
}

static void MotorDriver_OutputOff(uint8_t motor_id, const Motor_Channel_t *motor)
{
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0U);
  HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
  s_motor_direction[motor_id] = 0;
  s_motor_duty[motor_id] = 0;
}

void MotorDriver_Init(void)
{
  uint8_t motor_id;

  for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
  {
    s_motor_duty[motor_id] = 0;
    s_motor_direction[motor_id] = 0;
    s_motor_reverse_ready_tick[motor_id] = 0U;
    s_motor_ramp_tick[motor_id] = 0U;
    HAL_GPIO_WritePin(s_motors[motor_id].in1_port, s_motors[motor_id].in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s_motors[motor_id].in2_port, s_motors[motor_id].in2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(s_motors[motor_id].htim, s_motors[motor_id].channel, 0U);

    if (HAL_TIM_PWM_Start(s_motors[motor_id].htim, s_motors[motor_id].channel) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void Motor_SetOutput(uint8_t motor_id, int16_t duty_percent)
{
  const Motor_Channel_t *motor;
  uint32_t pulse;
  int16_t duty;
  int8_t direction;

  if (motor_id >= MOTOR_COUNT)
  {
    return;
  }

  motor = &s_motors[motor_id];
  duty = MotorDriver_ApplyEffectiveDuty(MotorDriver_ClampDuty(duty_percent));
  direction = MotorDriver_GetDirection(duty);

  if (direction == 0)
  {
    MotorDriver_OutputOff(motor_id, motor);
    s_motor_reverse_ready_tick[motor_id] = 0U;
    return;
  }

  if ((s_motor_direction[motor_id] == 0) &&
      (MotorDriver_TickExpired(s_motor_reverse_ready_tick[motor_id]) == 0U))
  {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0U);
    s_motor_duty[motor_id] = 0;
    return;
  }

  if (direction == s_motor_direction[motor_id])
  {
    duty = MotorDriver_ApplyRamp(motor_id, duty);
    pulse = MotorDriver_DutyToPulse(motor, duty);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pulse);
    s_motor_duty[motor_id] = duty;
    return;
  }

  if (s_motor_direction[motor_id] != 0)
  {
    MotorDriver_OutputOff(motor_id, motor);
    s_motor_reverse_ready_tick[motor_id] = HAL_GetTick() + MOTOR_REVERSE_COAST_MS;
    return;
  }

  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0U);
  HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
  MotorDriver_Deadtime();

  if (direction > 0)
  {
    HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
  }

  duty = MotorDriver_ApplyRamp(motor_id, duty);
  pulse = MotorDriver_DutyToPulse(motor, duty);
  __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pulse);
  s_motor_direction[motor_id] = direction;
  s_motor_reverse_ready_tick[motor_id] = 0U;
  s_motor_duty[motor_id] = duty;
}

void MotorDriver_SetDuty(int16_t duty_percent)
{
  Motor_SetOutput(0U, duty_percent);
}

void MotorDriver_Stop(void)
{
  Motor_SetOutput(0U, 0);
}

int16_t MotorDriver_GetDuty(void)
{
  return Motor_GetDuty(0U);
}

void Motor_StopAll(void)
{
  uint8_t motor_id;

  for (motor_id = 0U; motor_id < MOTOR_COUNT; motor_id++)
  {
    Motor_SetOutput(motor_id, 0);
  }
}

int16_t Motor_GetDuty(uint8_t motor_id)
{
  if (motor_id >= MOTOR_COUNT)
  {
    return 0;
  }

  return s_motor_duty[motor_id];
}
