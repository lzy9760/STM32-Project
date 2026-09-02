#include "ball_angle.h"

#include "ball_config.h"

static float s_filtered_adc;
static uint8_t s_filter_ready;

HAL_StatusTypeDef BallAngle_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);

  ADC2->CR1 = 0U;
  ADC2->CR2 = 0U;
  ADC2->SQR1 = 0U;
  ADC2->SQR2 = 0U;
  ADC2->SQR3 = ADC_CHANNEL_8;
  ADC2->SMPR2 = (ADC2->SMPR2 & ~(7U << 24U)) | (7U << 24U);
  ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;
  ADC2->CR2 |= ADC_CR2_ADON;

  s_filtered_adc = 0.0f;
  s_filter_ready = 0U;
  return HAL_OK;
}

uint8_t BallAngle_Read(uint16_t *raw_adc, float *filtered_adc)
{
  uint32_t total = 0U;
  uint32_t timeout;
  uint16_t sample = 0U;
  uint8_t i;

  if ((raw_adc == 0) || (filtered_adc == 0))
  {
    return 0U;
  }

  for (i = 0U; i < 8U; ++i)
  {
    ADC2->CR2 |= ADC_CR2_SWSTART;
    timeout = 100000U;
    while (((ADC2->SR & ADC_SR_EOC) == 0U) && (timeout > 0U))
    {
      --timeout;
    }
    if (timeout == 0U)
    {
      return 0U;
    }
    sample = (uint16_t)ADC2->DR;
    total += sample;
  }

  sample = (uint16_t)(total / 8U);
  if (s_filter_ready == 0U)
  {
    s_filtered_adc = (float)sample;
    s_filter_ready = 1U;
  }
  else
  {
    s_filtered_adc += BALL_ANGLE_FILTER_ALPHA
                      * ((float)sample - s_filtered_adc);
  }

  *raw_adc = sample;
  *filtered_adc = s_filtered_adc;
  return 1U;
}
