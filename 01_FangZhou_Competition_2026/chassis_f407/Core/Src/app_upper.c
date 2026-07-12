#include "app_upper.h"

#include "main.h"

#define STEPPER_PUL_GPIO_PORT       GPIOC
#define STEPPER_PUL_PIN             GPIO_PIN_8
#define STEPPER_PUL_TIM_CHANNEL     TIM_CHANNEL_3
#define STEPPER_DIR_GPIO_PORT       GPIOF
#define STEPPER_DIR_PIN             GPIO_PIN_14
#define STEPPER_EN_GPIO_PORT        GPIOF
#define STEPPER_EN_PIN              GPIO_PIN_15

#define SERVO_GPIO_PORT             GPIOC
#define SERVO_PIN                   GPIO_PIN_9

#define STEPPER_ENABLE_ACTIVE       GPIO_PIN_SET
#define STEPPER_UP_DIRECTION        GPIO_PIN_SET
#define STEPPER_DOWN_DIRECTION      GPIO_PIN_RESET
#define STEPPER_TIMER_TICK_HZ       100000U
#define STEPPER_MIN_SPEED_HZ        2U
#define STEPPER_MAX_SPEED_HZ        20000U
#define STEPPER_DEFAULT_SPEED_HZ    1000U

#define SERVO_PWM_FRAME_US          20000U
#define SERVO_TIMER_TICK_HZ         1000000U
#define SERVO_DUTY_25_PULSE_US      ((SERVO_PWM_FRAME_US * 25U) / 1000U)
#define SERVO_DUTY_75_PULSE_US      ((SERVO_PWM_FRAME_US * 75U) / 1000U)
#define SERVO_PWM_ACTIVE_STATE      GPIO_PIN_RESET
#define SERVO_PWM_IDLE_STATE        GPIO_PIN_SET

#define UPPER_BUTTON_DEBOUNCE_COUNT 3U

static TIM_HandleTypeDef s_htim8;
static TIM_HandleTypeDef s_htim7;
static uint8_t s_stepper_running;
static GPIO_PinState s_stepper_direction;
static volatile uint16_t s_servo_pulse_us = SERVO_DUTY_25_PULSE_US;
static volatile uint8_t s_servo_pwm_active;
static uint8_t s_button_sample_flags;
static uint8_t s_button_stable_flags;
static uint8_t s_button_stable_count;

static void Upper_TIM8_Init(void);
static void Upper_TIM7_Init(void);
static void Stepper_GPIO_Init(void);
static void Servo_GPIO_Init(void);
static uint32_t Stepper_GetTIM8ClockHz(void);
static HAL_StatusTypeDef Stepper_SetSpeed(uint32_t step_hz);
static HAL_StatusTypeDef Stepper_Run(GPIO_PinState direction);
static void Stepper_Hold(void);
static void Stepper_Enable(uint8_t enable);
static void Servo_SetPulseUs(uint16_t pulse_us);
static void Servo_SetTimerPeriodUs(uint16_t period_us);
static void Servo_PWM_Start(void);
static uint8_t Upper_NormalizeFlags(uint8_t remote_flags);
static uint8_t Upper_DebounceFlags(uint8_t remote_flags);

void AppUpper_Init(void)
{
  Upper_TIM8_Init();
  Upper_TIM7_Init();
  Stepper_GPIO_Init();
  Servo_GPIO_Init();
  Servo_SetPulseUs(SERVO_DUTY_25_PULSE_US);
  Servo_PWM_Start();

  if (Stepper_SetSpeed(STEPPER_DEFAULT_SPEED_HZ) != HAL_OK)
  {
    Error_Handler();
  }

  Stepper_Enable(1U);
  Stepper_Hold();
}

void AppUpper_Task(uint8_t remote_flags)
{
  uint8_t flags;
  uint8_t lift_flags;
  uint8_t servo_flags;

  flags = Upper_DebounceFlags(remote_flags);

  lift_flags = flags & (APP_UPPER_FLAG_LIFT_UP | APP_UPPER_FLAG_LIFT_DOWN);
  if (lift_flags == APP_UPPER_FLAG_LIFT_UP)
  {
    if (Stepper_Run(STEPPER_UP_DIRECTION) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else if (lift_flags == APP_UPPER_FLAG_LIFT_DOWN)
  {
    if (Stepper_Run(STEPPER_DOWN_DIRECTION) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
  {
    Stepper_Hold();
  }

  servo_flags = flags & (APP_UPPER_FLAG_SERVO_75 | APP_UPPER_FLAG_SERVO_25);
  if (servo_flags == APP_UPPER_FLAG_SERVO_75)
  {
    Servo_SetPulseUs(SERVO_DUTY_75_PULSE_US);
  }
  else if (servo_flags == APP_UPPER_FLAG_SERVO_25)
  {
    Servo_SetPulseUs(SERVO_DUTY_25_PULSE_US);
  }
}

void AppUpper_TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&s_htim7);
}

static void Upper_TIM8_Init(void)
{
  TIM_MasterConfigTypeDef master_config = {0};
  TIM_OC_InitTypeDef oc_config = {0};
  TIM_BreakDeadTimeConfigTypeDef break_config = {0};

  __HAL_RCC_TIM8_CLK_ENABLE();

  s_htim8.Instance = TIM8;
  s_htim8.Init.Prescaler = 0;
  s_htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_htim8.Init.Period = 65535;
  s_htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  s_htim8.Init.RepetitionCounter = 0;
  s_htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&s_htim8) != HAL_OK)
  {
    Error_Handler();
  }

  master_config.MasterOutputTrigger = TIM_TRGO_RESET;
  master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&s_htim8, &master_config) != HAL_OK)
  {
    Error_Handler();
  }

  oc_config.OCMode = TIM_OCMODE_PWM1;
  oc_config.Pulse = 0;
  oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc_config.OCFastMode = TIM_OCFAST_DISABLE;
  oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&s_htim8, &oc_config, STEPPER_PUL_TIM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  break_config.OffStateRunMode = TIM_OSSR_DISABLE;
  break_config.OffStateIDLEMode = TIM_OSSI_DISABLE;
  break_config.LockLevel = TIM_LOCKLEVEL_OFF;
  break_config.DeadTime = 0;
  break_config.BreakState = TIM_BREAK_DISABLE;
  break_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  break_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&s_htim8, &break_config) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_GPIOC_CLK_ENABLE();
  {
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = STEPPER_PUL_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(STEPPER_PUL_GPIO_PORT, &gpio);
  }
}

static void Upper_TIM7_Init(void)
{
  RCC_ClkInitTypeDef clock_config = {0};
  uint32_t flash_latency = 0U;
  uint32_t timer_clock_hz;
  uint32_t prescaler;

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  timer_clock_hz = HAL_RCC_GetPCLK1Freq();
  if (clock_config.APB1CLKDivider != RCC_HCLK_DIV1)
  {
    timer_clock_hz *= 2U;
  }

  if (timer_clock_hz < SERVO_TIMER_TICK_HZ)
  {
    Error_Handler();
  }

  prescaler = (timer_clock_hz / SERVO_TIMER_TICK_HZ) - 1U;
  if (prescaler > 0xFFFFU)
  {
    Error_Handler();
  }

  __HAL_RCC_TIM7_CLK_ENABLE();

  s_htim7.Instance = TIM7;
  s_htim7.Init.Prescaler = prescaler;
  s_htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_htim7.Init.Period = SERVO_DUTY_25_PULSE_US - 1U;
  s_htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&s_htim7) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

static void Stepper_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();

  HAL_GPIO_WritePin(STEPPER_DIR_GPIO_PORT, STEPPER_DIR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STEPPER_EN_GPIO_PORT, STEPPER_EN_PIN,
                    (STEPPER_ENABLE_ACTIVE == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  gpio.Pin = STEPPER_DIR_PIN | STEPPER_EN_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &gpio);
}

static void Servo_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(SERVO_GPIO_PORT, SERVO_PIN, SERVO_PWM_IDLE_STATE);

  gpio.Pin = SERVO_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERVO_GPIO_PORT, &gpio);
}

static uint32_t Stepper_GetTIM8ClockHz(void)
{
  RCC_ClkInitTypeDef clock_config = {0};
  uint32_t flash_latency = 0U;
  uint32_t pclk2_hz = HAL_RCC_GetPCLK2Freq();

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  if (clock_config.APB2CLKDivider != RCC_HCLK_DIV1)
  {
    pclk2_hz *= 2U;
  }

  return pclk2_hz;
}

static HAL_StatusTypeDef Stepper_SetSpeed(uint32_t step_hz)
{
  uint32_t timer_clock_hz;
  uint32_t prescaler;
  uint32_t period_ticks;

  if ((step_hz < STEPPER_MIN_SPEED_HZ) || (step_hz > STEPPER_MAX_SPEED_HZ) || (s_stepper_running != 0U))
  {
    return HAL_ERROR;
  }

  timer_clock_hz = Stepper_GetTIM8ClockHz();
  if (timer_clock_hz < STEPPER_TIMER_TICK_HZ)
  {
    return HAL_ERROR;
  }

  prescaler = (timer_clock_hz / STEPPER_TIMER_TICK_HZ) - 1U;
  period_ticks = STEPPER_TIMER_TICK_HZ / step_hz;

  if ((prescaler > 0xFFFFU) || (period_ticks < 2U) || (period_ticks > 0x10000U))
  {
    return HAL_ERROR;
  }

  __HAL_TIM_DISABLE(&s_htim8);
  __HAL_TIM_SET_PRESCALER(&s_htim8, prescaler);
  __HAL_TIM_SET_AUTORELOAD(&s_htim8, period_ticks - 1U);
  __HAL_TIM_SET_COMPARE(&s_htim8, STEPPER_PUL_TIM_CHANNEL, period_ticks / 2U);
  __HAL_TIM_SET_COUNTER(&s_htim8, 0U);
  if (HAL_TIM_GenerateEvent(&s_htim8, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  __HAL_TIM_CLEAR_FLAG(&s_htim8, TIM_FLAG_UPDATE | TIM_FLAG_CC1);

  return HAL_OK;
}

static HAL_StatusTypeDef Stepper_Run(GPIO_PinState direction)
{
  if ((s_stepper_running != 0U) && (direction == s_stepper_direction))
  {
    return HAL_OK;
  }

  if (s_stepper_running != 0U)
  {
    HAL_TIM_PWM_Stop(&s_htim8, STEPPER_PUL_TIM_CHANNEL);
    s_stepper_running = 0U;
  }

  HAL_GPIO_WritePin(STEPPER_DIR_GPIO_PORT, STEPPER_DIR_PIN, direction);
  Stepper_Enable(1U);

  __HAL_TIM_SET_COUNTER(&s_htim8, 0U);
  __HAL_TIM_CLEAR_FLAG(&s_htim8, TIM_FLAG_UPDATE | TIM_FLAG_CC1);
  if (HAL_TIM_PWM_Start(&s_htim8, STEPPER_PUL_TIM_CHANNEL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  s_stepper_direction = direction;
  s_stepper_running = 1U;

  return HAL_OK;
}

static void Stepper_Hold(void)
{
  if (s_stepper_running != 0U)
  {
    HAL_TIM_PWM_Stop(&s_htim8, STEPPER_PUL_TIM_CHANNEL);
    __HAL_TIM_SET_COUNTER(&s_htim8, 0U);
    s_stepper_running = 0U;
  }

  Stepper_Enable(1U);
}

static void Stepper_Enable(uint8_t enable)
{
  GPIO_PinState pin_state;

  if (enable != 0U)
  {
    pin_state = STEPPER_ENABLE_ACTIVE;
  }
  else
  {
    pin_state = (STEPPER_ENABLE_ACTIVE == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(STEPPER_EN_GPIO_PORT, STEPPER_EN_PIN, pin_state);
}

static void Servo_SetPulseUs(uint16_t pulse_us)
{
  if (pulse_us == 0U)
  {
    pulse_us = 1U;
  }
  else if (pulse_us >= SERVO_PWM_FRAME_US)
  {
    pulse_us = SERVO_PWM_FRAME_US - 1U;
  }

  s_servo_pulse_us = pulse_us;
}

static void Servo_SetTimerPeriodUs(uint16_t period_us)
{
  if (period_us == 0U)
  {
    period_us = 1U;
  }

  __HAL_TIM_SET_AUTORELOAD(&s_htim7, period_us - 1U);
  __HAL_TIM_SET_COUNTER(&s_htim7, 0U);
}

static void Servo_PWM_Start(void)
{
  s_servo_pwm_active = 1U;
  HAL_GPIO_WritePin(SERVO_GPIO_PORT, SERVO_PIN, SERVO_PWM_ACTIVE_STATE);
  Servo_SetTimerPeriodUs(s_servo_pulse_us);

  if (HAL_TIM_Base_Start_IT(&s_htim7) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint8_t Upper_NormalizeFlags(uint8_t remote_flags)
{
  uint8_t flags = remote_flags & APP_UPPER_FLAG_MASK;

  if ((flags & (APP_UPPER_FLAG_LIFT_UP | APP_UPPER_FLAG_LIFT_DOWN)) ==
      (APP_UPPER_FLAG_LIFT_UP | APP_UPPER_FLAG_LIFT_DOWN))
  {
    flags &= (uint8_t)~(APP_UPPER_FLAG_LIFT_UP | APP_UPPER_FLAG_LIFT_DOWN);
  }

  if ((flags & (APP_UPPER_FLAG_SERVO_75 | APP_UPPER_FLAG_SERVO_25)) ==
      (APP_UPPER_FLAG_SERVO_75 | APP_UPPER_FLAG_SERVO_25))
  {
    flags &= (uint8_t)~(APP_UPPER_FLAG_SERVO_75 | APP_UPPER_FLAG_SERVO_25);
  }

  return flags;
}

static uint8_t Upper_DebounceFlags(uint8_t remote_flags)
{
  uint8_t flags = Upper_NormalizeFlags(remote_flags);

  if (flags == 0U)
  {
    s_button_sample_flags = 0U;
    s_button_stable_flags = 0U;
    s_button_stable_count = 0U;
    return 0U;
  }

  if (flags == s_button_sample_flags)
  {
    if (s_button_stable_count < UPPER_BUTTON_DEBOUNCE_COUNT)
    {
      s_button_stable_count++;
    }
  }
  else
  {
    s_button_sample_flags = flags;
    s_button_stable_count = 1U;
  }

  if (s_button_stable_count >= UPPER_BUTTON_DEBOUNCE_COUNT)
  {
    s_button_stable_flags = s_button_sample_flags;
  }

  return s_button_stable_flags;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7)
  {
    if (s_servo_pwm_active != 0U)
    {
      HAL_GPIO_WritePin(SERVO_GPIO_PORT, SERVO_PIN, SERVO_PWM_IDLE_STATE);
      s_servo_pwm_active = 0U;
      Servo_SetTimerPeriodUs((uint16_t)(SERVO_PWM_FRAME_US - s_servo_pulse_us));
    }
    else
    {
      HAL_GPIO_WritePin(SERVO_GPIO_PORT, SERVO_PIN, SERVO_PWM_ACTIVE_STATE);
      s_servo_pwm_active = 1U;
      Servo_SetTimerPeriodUs(s_servo_pulse_us);
    }
  }
}
