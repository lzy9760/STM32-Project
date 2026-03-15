#include "task_monitor.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_debug.h"
#include "app_mode.h"
#include "bsp_board.h"
#include "task_cfg.h"

void Task_Monitor_Entry(void)
{
  TickType_t last_wake;
  uint32_t monitor_tick = 0U;//监控周期时间，单位为毫秒

  last_wake = xTaskGetTickCount();//获取当前任务的tick计数，作为任务的初始唤醒时间

  for (;;)
  {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_MONITOR_PERIOD_MS));//任务延时，等待下一个周期
    monitor_tick += TASK_MONITOR_PERIOD_MS;//累加监控周期时间

    APP_Mode_SetForceSafe(BSP_Board_ReadUserSwitch());//根据用户开关状态设置强制安全模式
    APP_Debug_UpdateIndicator(monitor_tick);//更新调试指示器，显示监控周期时间
  }
}
