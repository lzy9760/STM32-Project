#include "main.h"
#include "Delay.h"
#include "AD.h"
#include "NRF24L01.h"

static void MX_GPIO_Init(void);
void SystemClock_Config(void);

#define JOY_LV_CHANNEL ADC_CHANNEL_0
#define JOY_LH_CHANNEL ADC_CHANNEL_1
#define JOY_RV_CHANNEL ADC_CHANNEL_2
#define JOY_RH_CHANNEL ADC_CHANNEL_3
#define REMOTE_TX_PERIOD_MS 10U

static uint8_t Joystick_ToByte(uint16_t adc_value);

int main(void)
{
	uint32_t last_tx_tick;

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

	AD_Init();
	NRF24L01_Init();

	last_tx_tick = HAL_GetTick() - REMOTE_TX_PERIOD_MS;

	while (1)
	{
		uint32_t now = HAL_GetTick();

		if ((now - last_tx_tick) >= REMOTE_TX_PERIOD_MS)
		{
			last_tx_tick = now;

			/* Read ADC as close to send time as possible for freshest values */
			NRF24L01_TxPacket[0] = Joystick_ToByte(AD_GetValue(JOY_LV_CHANNEL));
			NRF24L01_TxPacket[1] = Joystick_ToByte(AD_GetValue(JOY_LH_CHANNEL));
			NRF24L01_TxPacket[2] = Joystick_ToByte(AD_GetValue(JOY_RV_CHANNEL));
			NRF24L01_TxPacket[3] = Joystick_ToByte(AD_GetValue(JOY_RH_CHANNEL));

			(void)NRF24L01_Send();
		}
	}
}

static uint8_t Joystick_ToByte(uint16_t adc_value)
{
	if (adc_value > 4095U)
	{
		adc_value = 4095U;
	}

	return (uint8_t)((adc_value * 255U + 2047U) / 4095U);
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
	                              RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
	(void)file;
	(void)line;
}
#endif
