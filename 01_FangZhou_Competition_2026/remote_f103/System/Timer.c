#include "main.h"
#include "Timer.h"
#include "Key.h"

static TIM_HandleTypeDef htim1;

void Timer_Init(void)
{
	__HAL_RCC_TIM1_CLK_ENABLE();

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = (uint32_t)(HAL_RCC_GetPCLK2Freq() / 1000000U) - 1U;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000U - 1U;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0U;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
	HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2U, 1U);
	HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

	if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
}

void TIM1_UP_IRQHandler(void)
{
	if ((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) &&
	    (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) != RESET))
	{
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
		Key_Tick();
	}
}
