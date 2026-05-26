#include "app.h"
#include "app_encoder.h"
#include "app_freertos.h"
#include "app_motor.h"
#include "app_mpu6050.h"
#include "app_navigation.h"
#include "app_oled.h"
#include "app_remote.h"
#include "pid.h"

#define APP_AUTO_NAV_ENABLE       1U
#define APP_AUTO_NAV_TARGET_X_MM  3000.0f
#define APP_AUTO_NAV_TARGET_Y_MM  500.0f
#define APP_AUTO_NAV_SPEED_MM_S   300.0f

void App_Init(void)
{
  Motor_StartAllPwm();
  Motor_StopAll();
  Encoder_Init();
  Pid_InitAll();
  Remote_Init();
  MPU6050_Init();
  Navigation_Init();
  AppOled_Init();
#if (APP_AUTO_NAV_ENABLE != 0U)
  Navigation_StartTo(APP_AUTO_NAV_TARGET_X_MM,
                     APP_AUTO_NAV_TARGET_Y_MM,
                     APP_AUTO_NAV_SPEED_MM_S);
#endif
}

void App_Task(void)
{
  if (Navigation_IsActive() != 0U)
  {
    if (Remote_HasManualCommand() != 0U)
    {
      Navigation_Stop();
      Remote_ControlTask();
    }
    else
    {
      Navigation_Task();
    }
  }
  else
  {
    Remote_ControlTask();
  }
  Pid_UpdateAll();
  AppOled_Task();
}

uint8_t App_StartScheduler(void)
{
  return App_FreeRtosStart();
}
