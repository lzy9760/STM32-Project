#include "main.h"
#include "Delay.h"
#include "AD.h"
#include "Key.h"
#include "NRF24L01.h"

static void MX_GPIO_Init(void);
void SystemClock_Config(void);

#define JOY_LV_CHANNEL ADC_CHANNEL_0
#define JOY_LH_CHANNEL ADC_CHANNEL_1
#define JOY_RV_CHANNEL ADC_CHANNEL_2
#define JOY_RH_CHANNEL ADC_CHANNEL_3
#define JOY_CHANNEL_COUNT 4U
#define REMOTE_TX_PERIOD_MS 5U
#define REMOTE_PACKET_BUTTON_INDEX 4U
#define JOY_ADC_MAX 4095U
#define JOY_CENTER_DEFAULT 2048U
#define JOY_CENTER_MIN 1400U
#define JOY_CENTER_MAX 2700U
#define JOY_DEADZONE_ADC 220U
#define JOY_ADC_AVERAGE_SAMPLES 4U
#define JOY_CENTER_CALIBRATION_SAMPLES 64U
#define REMOTE_NEUTRAL_BURST_COUNT 2U

static const uint32_t s_joy_channels[JOY_CHANNEL_COUNT] = {
	JOY_LV_CHANNEL,
	JOY_LH_CHANNEL,
	JOY_RV_CHANNEL,
	JOY_RH_CHANNEL
};
static uint16_t s_joy_center[JOY_CHANNEL_COUNT] = {
	JOY_CENTER_DEFAULT,
	JOY_CENTER_DEFAULT,
	JOY_CENTER_DEFAULT,
	JOY_CENTER_DEFAULT
};

static void Joystick_CalibrateCenter(void);
static uint16_t Joystick_ReadAverage(uint8_t axis_index);
static uint8_t Joystick_ToByte(uint8_t axis_index, uint16_t adc_value);
static uint8_t Remote_IsNeutralPacket(void);

int main(void)
{
	uint32_t last_tx_tick;
	uint8_t was_active;

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

	AD_Init();
	Key_Init();
	Joystick_CalibrateCenter();
	NRF24L01_Init();

	last_tx_tick = HAL_GetTick() - REMOTE_TX_PERIOD_MS;
	was_active = 0U;

	while (1)
	{
		uint32_t now = HAL_GetTick();

		if ((now - last_tx_tick) >= REMOTE_TX_PERIOD_MS)
		{
			uint8_t i;
			uint8_t is_neutral;
			uint8_t send_count;
			uint8_t neutral_sent_ok;

			last_tx_tick = now;

			NRF24L01_TxPacket[0] = Joystick_ToByte(0U, Joystick_ReadAverage(0U));
			NRF24L01_TxPacket[1] = Joystick_ToByte(1U, Joystick_ReadAverage(1U));
			NRF24L01_TxPacket[2] = Joystick_ToByte(2U, Joystick_ReadAverage(2U));
			NRF24L01_TxPacket[3] = Joystick_ToByte(3U, Joystick_ReadAverage(3U));
			NRF24L01_TxPacket[REMOTE_PACKET_BUTTON_INDEX] = Key_GetLiftButtons();

			is_neutral = Remote_IsNeutralPacket();
			send_count = ((was_active != 0U) && (is_neutral != 0U)) ? REMOTE_NEUTRAL_BURST_COUNT : 1U;
			neutral_sent_ok = 0U;
			for (i = 0U; i < send_count; i++)
			{
				if (NRF24L01_Send() == 1U)
				{
					neutral_sent_ok = 1U;
				}
			}

			if (is_neutral == 0U)
			{
				was_active = 1U;
			}
			else if (neutral_sent_ok != 0U)
			{
				was_active = 0U;
			}
		}
	}
}

static void Joystick_CalibrateCenter(void)
{
	uint8_t i;
	uint16_t sample;
	uint32_t sum[JOY_CHANNEL_COUNT] = {0U, 0U, 0U, 0U};
	uint32_t center;

	for (sample = 0U; sample < JOY_CENTER_CALIBRATION_SAMPLES; sample++)
	{
		for (i = 0U; i < JOY_CHANNEL_COUNT; i++)
		{
			sum[i] += AD_GetValue(s_joy_channels[i]);
		}
		HAL_Delay(2U);
	}

	for (i = 0U; i < JOY_CHANNEL_COUNT; i++)
	{
		center = (sum[i] + (JOY_CENTER_CALIBRATION_SAMPLES / 2U)) / JOY_CENTER_CALIBRATION_SAMPLES;
		if ((center < JOY_CENTER_MIN) || (center > JOY_CENTER_MAX))
		{
			center = JOY_CENTER_DEFAULT;
		}
		s_joy_center[i] = (uint16_t)center;
	}
}

static uint16_t Joystick_ReadAverage(uint8_t axis_index)
{
	uint8_t sample;
	uint32_t sum;

	if (axis_index >= JOY_CHANNEL_COUNT)
	{
		axis_index = 0U;
	}

	sum = 0U;
	for (sample = 0U; sample < JOY_ADC_AVERAGE_SAMPLES; sample++)
	{
		sum += AD_GetValue(s_joy_channels[axis_index]);
	}

	return (uint16_t)((sum + (JOY_ADC_AVERAGE_SAMPLES / 2U)) / JOY_ADC_AVERAGE_SAMPLES);
}

static uint8_t Joystick_ToByte(uint8_t axis_index, uint16_t adc_value)
{
	int32_t delta;
	uint32_t span;
	uint32_t magnitude;
	uint32_t scaled;
	uint16_t center;

	if (adc_value > JOY_ADC_MAX)
	{
		adc_value = JOY_ADC_MAX;
	}
	if (axis_index >= JOY_CHANNEL_COUNT)
	{
		axis_index = 0U;
	}

	center = s_joy_center[axis_index];
	delta = (int32_t)adc_value - (int32_t)center;
	magnitude = (delta >= 0) ? (uint32_t)delta : (uint32_t)(-delta);
	if (magnitude <= JOY_DEADZONE_ADC)
	{
		return 128U;
	}

	span = (delta >= 0) ? (JOY_ADC_MAX - center) : center;
	if (span <= JOY_DEADZONE_ADC)
	{
		return 128U;
	}

	magnitude -= JOY_DEADZONE_ADC;
	span -= JOY_DEADZONE_ADC;
	scaled = (magnitude * 1000U + (span / 2U)) / span;
	if (scaled > 1000U)
	{
		scaled = 1000U;
	}

	if (delta >= 0)
	{
		return (uint8_t)(128U + ((scaled * 127U + 500U) / 1000U));
	}
	return (uint8_t)(128U - ((scaled * 128U + 500U) / 1000U));
}

static uint8_t Remote_IsNeutralPacket(void)
{
	if ((NRF24L01_TxPacket[0] == 128U) &&
	    (NRF24L01_TxPacket[1] == 128U) &&
	    (NRF24L01_TxPacket[2] == 128U) &&
	    (NRF24L01_TxPacket[3] == 128U) &&
	    (NRF24L01_TxPacket[REMOTE_PACKET_BUTTON_INDEX] == 0U))
	{
		return 1U;
	}
	return 0U;
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
