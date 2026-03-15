#include "bsp_clock.h"
#include "stm32f1xx_hal.h"

void BSP_Clock_Init(void)
{
  /* HAL_GetTick is used directly as the system time base. */
}

uint32_t BSP_Clock_Millis(void)
{//获取当前系统时间（毫秒）
  return HAL_GetTick();
}

float BSP_Clock_ElapsedSeconds(uint32_t *last_ms)
{//计算自上次调用以来的时间间隔（秒）
  uint32_t now;
  uint32_t elapsed_ms;

  if (last_ms == 0)
  {
    return 0.0f;
  }

  now = HAL_GetTick();
  elapsed_ms = now - *last_ms;//计算时间间隔（毫秒）
  *last_ms = now;

  return (float)elapsed_ms * 0.001f;
}
