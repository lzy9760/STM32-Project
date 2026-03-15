#include "app_debug.h"

#include "app_mode.h"
#include "bsp_board.h"
#include "main.h"

void APP_Debug_FillVisionTelemetry(BSP_VisionTelemetry_t *telemetry)
{//填充视觉遥测数据
  AppRuntime_t *runtime;

  if (telemetry == 0)
  {
    return;
  }
//获取当前应用运行时状态
  runtime = APP_Mode_Runtime();
  if (runtime == 0)
  {
    return;
  }

  telemetry->mode = (uint8_t)runtime->gimbal_out.mode;//设置视觉遥测数据模式
  telemetry->yaw_deg = runtime->gimbal_feedback.yaw_deg;//设置视觉遥测数据yaw轴角度
  telemetry->pitch_deg = runtime->gimbal_feedback.pitch_deg;//设置视觉遥测数据pitch轴角度
  telemetry->roll_deg = runtime->attitude.roll_deg;//设置视觉遥测数据roll轴角度
  telemetry->yaw_rate_dps = runtime->gimbal_feedback.yaw_rate_dps;//设置视觉遥测数据yaw轴角速度
  telemetry->pitch_rate_dps = runtime->gimbal_feedback.pitch_rate_dps;//设置视觉遥测数据pitch轴角速度
}

void APP_Debug_UpdateIndicator(uint32_t monitor_tick_ms)
{//更新指示器状态
  AppRuntime_t *runtime;
  uint8_t led_level;

  runtime = APP_Mode_Runtime();
  if (runtime == 0)
  {
    return;
  }

  if (runtime->can_online == 0U)
  {
    led_level = (uint8_t)(((monitor_tick_ms / 80U) & 0x01U) ? 1U : 0U);//如果CAN线离线，指示灯闪烁
  }
  else if (runtime->gimbal_limited != 0U)
  {
    led_level = (uint8_t)(((monitor_tick_ms / 100U) & 0x01U) ? 1U : 0U);//如果云台受限，指示灯闪烁
  }
  else if (runtime->imu_online == 0U)
  {
    led_level = (uint8_t)(((monitor_tick_ms / 300U) & 0x01U) ? 1U : 0U);//如果IMU离线，指示灯闪烁
  }
  else
  {
    led_level = 1U;
  }

  BSP_Board_SetIndicatorLevel(led_level);
}

void APP_Debug_Heartbeat(void)
{
  /* Reserved for future debug uplink / log flushing. */
}

__weak void APP_VisionTxBytesHook(const uint8_t *data, uint16_t len)
{
  (void)data;
  (void)len;
}
