#include "task_comm.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_debug.h"
#include "bsp_vision_link.h"
#include "protocol_cfg.h"
#include "task_cfg.h"

#include <string.h>

void Task_Comm_Entry(void)
{
  TickType_t last_wake;
  BSP_VisionTelemetry_t telemetry;
  uint8_t tx_bytes[PROTOCOL_VISION_TX_BUFFER_SIZE];

  memset(&telemetry, 0, sizeof(telemetry));//将遥测数据结构体telemetry的所有字节初始化为0
  last_wake = xTaskGetTickCount();//获取当前任务的tick计数，作为任务的初始唤醒时间

  for (;;)
  {
    uint16_t tx_len;

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_COMM_PERIOD_MS));//任务延时，等待下一个周期

    APP_Debug_FillVisionTelemetry(&telemetry);//填充遥测数据
    BSP_Vision_SendTelemetry(&telemetry);//发送遥测数据

    tx_len = BSP_Vision_PopTx(tx_bytes, (uint16_t)sizeof(tx_bytes));
    if (tx_len > 0U)
    {
      APP_VisionTxBytesHook(tx_bytes, tx_len);//将打包后的vision帧发送到串口
    }
  }
}
