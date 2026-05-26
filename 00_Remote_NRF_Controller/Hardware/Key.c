#include "main.h"
#include "Key.h"

static uint8_t Key_Num;

void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
	                      GPIO_PIN_7 | GPIO_PIN_11 | GPIO_PIN_12;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
	                      GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_10;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

uint8_t Key_GetNum(void)
{
	uint8_t temp;

	if (Key_Num != 0U)
	{
		temp = Key_Num;
		Key_Num = 0U;
		return temp;
	}

	return 0U;
}

uint8_t Key_GetState(void)
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET) { return 1U; }
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) { return 2U; }
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) { return 3U; }
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) { return 4U; }
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) { return 5U; }
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) { return 6U; }
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET) { return 7U; }
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET) { return 8U; }
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) { return 9U; }
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) { return 10U; }
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) { return 11U; }
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) { return 12U; }

	return 0U;
}

void Key_Tick(void)
{
	static uint8_t count;
	static uint8_t curr_state;
	static uint8_t prev_state;

	count++;
	if (count >= 20U)
	{
		count = 0U;
		prev_state = curr_state;
		curr_state = Key_GetState();

		if ((curr_state == 0U) && (prev_state != 0U))
		{
			Key_Num = prev_state;
		}
	}
}
