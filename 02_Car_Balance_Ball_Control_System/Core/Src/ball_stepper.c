#include "ball_stepper.h"

#include "ball_config.h"

#define STEPPER_TIMER_HZ 100000U

static volatile uint32_t s_half_period_ticks;
static volatile int32_t s_command_hz;
static volatile uint8_t s_enabled;
static volatile uint8_t s_running;
static GPIO_PinState s_active_direction;
static GPIO_PinState s_pending_direction;
static uint8_t s_direction_valid;
static uint8_t s_reverse_confirm_count;

static void StepPinAsLowOutput(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}

static void StepPinAsTimerOutput(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &gpio);
}

static void StopPulseOutput(void)
{
  TIM3->DIER &= ~TIM_DIER_CC1IE;
  TIM3->CCER &= ~TIM_CCER_CC1E;
  TIM3->CR1 &= ~TIM_CR1_CEN;
  TIM3->SR = 0U;
  StepPinAsLowOutput();
  s_running = 0U;
  s_command_hz = 0;
}

HAL_StatusTypeDef BallStepper_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &gpio);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  StepPinAsLowOutput();

  TIM3->CR1 = 0U;
  TIM3->CR2 = 0U;
  TIM3->SMCR = 0U;
  TIM3->PSC = 839U;
  TIM3->ARR = 0xFFFFU;
  TIM3->CCMR1 = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1;
  TIM3->CCER = 0U;
  TIM3->DIER = 0U;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = 0U;

  HAL_NVIC_SetPriority(TIM3_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  s_half_period_ticks = 0U;
  s_command_hz = 0;
  s_enabled = 0U;
  s_running = 0U;
  s_direction_valid = 0U;
  s_reverse_confirm_count = 0U;
  BallStepper_SetEnabled(0U);
  return HAL_OK;
}

void BallStepper_Stop(void)
{
  StopPulseOutput();
  s_reverse_confirm_count = 0U;
}

void BallStepper_SetEnabled(uint8_t enabled)
{
  GPIO_PinState level;

  if (enabled == 0U)
  {
    BallStepper_Stop();
    level = (BALL_STEPPER_ENABLE_ACTIVE_LEVEL == GPIO_PIN_SET)
            ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, level);
    s_enabled = 0U;
    s_direction_valid = 0U;
    return;
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,
                    BALL_STEPPER_ENABLE_ACTIVE_LEVEL);
  s_enabled = 1U;
}

int32_t BallStepper_SetCommand(float command_hz)
{
  GPIO_PinState direction;
  float magnitude;
  uint32_t frequency_hz;
  uint32_t half_period;

  if (s_enabled == 0U)
  {
    BallStepper_Stop();
    return 0;
  }

  magnitude = (command_hz >= 0.0f) ? command_hz : -command_hz;
  if (magnitude <= BALL_STEPPER_DEADBAND_HZ)
  {
    BallStepper_Stop();
    return 0;
  }
  if (magnitude < BALL_STEPPER_MIN_HZ)
  {
    magnitude = BALL_STEPPER_MIN_HZ;
  }
  if (magnitude > BALL_STEPPER_MAX_HZ)
  {
    magnitude = BALL_STEPPER_MAX_HZ;
  }

  frequency_hz = (uint32_t)(magnitude + 0.5f);
  half_period = STEPPER_TIMER_HZ / (2U * frequency_hz);
  if (half_period < 2U)
  {
    half_period = 2U;
  }
  direction = (command_hz >= 0.0f)
              ? BALL_STEPPER_POSITIVE_DIR_LEVEL
              : ((BALL_STEPPER_POSITIVE_DIR_LEVEL == GPIO_PIN_SET)
                 ? GPIO_PIN_RESET : GPIO_PIN_SET);

  if (s_direction_valid == 0U)
  {
    s_active_direction = direction;
    s_direction_valid = 1U;
    s_reverse_confirm_count = 0U;
  }
  else if (direction != s_active_direction)
  {
    if ((s_reverse_confirm_count == 0U)
        || (direction != s_pending_direction))
    {
      s_pending_direction = direction;
      s_reverse_confirm_count = 1U;
    }
    else if (s_reverse_confirm_count < BALL_STEPPER_REVERSE_CONFIRM_CYCLES)
    {
      s_reverse_confirm_count++;
    }

    StopPulseOutput();
    if (s_reverse_confirm_count < BALL_STEPPER_REVERSE_CONFIRM_CYCLES)
    {
      return 0;
    }

    s_active_direction = direction;
    s_reverse_confirm_count = 0U;
  }
  else
  {
    s_reverse_confirm_count = 0U;
  }

  if ((s_running == 0U)
      || (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) != s_active_direction))
  {
    StopPulseOutput();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, s_active_direction);
    StepPinAsTimerOutput();
    s_half_period_ticks = half_period;
    TIM3->CNT = 0U;
    TIM3->CCR1 = half_period;
    TIM3->SR = 0U;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->DIER |= TIM_DIER_CC1IE;
    TIM3->CR1 |= TIM_CR1_CEN;
    s_running = 1U;
  }
  else
  {
    s_half_period_ticks = half_period;
  }

  s_command_hz = (command_hz >= 0.0f)
                 ? (int32_t)frequency_hz : -(int32_t)frequency_hz;
  return s_command_hz;
}

uint8_t BallStepper_IsEnabled(void)
{
  return s_enabled;
}

int32_t BallStepper_GetCommand(void)
{
  return s_command_hz;
}

void TIM3_IRQHandler(void)
{
  if ((TIM3->SR & TIM_SR_CC1IF) != 0U)
  {
    TIM3->SR &= ~TIM_SR_CC1IF;
    TIM3->CCR1 += s_half_period_ticks;
  }
}
