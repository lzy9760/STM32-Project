#include "main.h"
#include "Delay.h"

void Delay_us(uint32_t us)
{
	uint32_t ticks = (SystemCoreClock / 1000000U) * us;
	uint32_t start = DWT->CYCCNT;

	if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0U;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
		start = DWT->CYCCNT;
	}

	while ((DWT->CYCCNT - start) < ticks)
	{
	}
}

void Delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

void Delay_s(uint32_t s)
{
	while (s--)
	{
		HAL_Delay(1000U);
	}
}
