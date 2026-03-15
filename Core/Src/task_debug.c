#include "task_debug.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_debug.h"
#include "task_cfg.h"

void Task_Debug_Entry(void)
{
  TickType_t last_wake;

  last_wake = xTaskGetTickCount();//获取当前任务的tick计数，作为任务的初始唤醒时间

  for (;;)
  {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_DEBUG_PERIOD_MS));//任务延时，等待下一个周期
    APP_Debug_Heartbeat();//发送调试心跳包
  }
}
