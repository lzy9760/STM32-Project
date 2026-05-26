#include "app_freertos.h"

#include "app_navigation.h"
#include "app_oled.h"
#include "app_remote.h"
#include "pid.h"

#include "FreeRTOS.h"
#include "task.h"

#define REMOTE_TASK_PERIOD_MS       5U
#define NAVIGATION_TASK_PERIOD_MS   5U
#define OLED_TASK_PERIOD_MS         50U
#define REMOTE_TASK_STACK           384U
#define NAVIGATION_TASK_STACK       512U
#define PID_TASK_STACK              192U
#define OLED_TASK_STACK             384U
#define REMOTE_TASK_PRIORITY        4U
#define NAVIGATION_TASK_PRIORITY    4U
#define PID_TASK_PRIORITY           3U
#define OLED_TASK_PRIORITY          1U

static void Remote_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (Navigation_IsActive() == 0U)
    {
      Remote_ControlTask();
    }
    else if (Remote_HasManualCommand() != 0U)
    {
      Navigation_Stop();
      Remote_ControlTask();
    }
    vTaskDelay(pdMS_TO_TICKS(REMOTE_TASK_PERIOD_MS));
  }
}

static void Navigation_TaskEntry(void *argument)
{
  (void)argument;

  for (;;)
  {
    Navigation_Task();
    vTaskDelay(pdMS_TO_TICKS(NAVIGATION_TASK_PERIOD_MS));
  }
}

static void Pid_Task(void *argument)
{
  TickType_t last_tick;

  (void)argument;
  last_tick = xTaskGetTickCount();

  for (;;)
  {
    Pid_UpdateAll();
    vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(PID_RUN_PERIOD_MS));
  }
}

static void Oled_Task(void *argument)
{
  (void)argument;

  for (;;)
  {
    AppOled_Task();
    vTaskDelay(pdMS_TO_TICKS(OLED_TASK_PERIOD_MS));
  }
}

uint8_t App_FreeRtosStart(void)
{
  BaseType_t remote_ok;
  BaseType_t navigation_ok;
  BaseType_t pid_ok;
  BaseType_t oled_ok;

  remote_ok = xTaskCreate(Remote_Task,
                          "remote",
                          REMOTE_TASK_STACK,
                          NULL,
                          REMOTE_TASK_PRIORITY,
                          NULL);
  navigation_ok = xTaskCreate(Navigation_TaskEntry,
                              "nav",
                              NAVIGATION_TASK_STACK,
                              NULL,
                              NAVIGATION_TASK_PRIORITY,
                              NULL);
  pid_ok = xTaskCreate(Pid_Task,
                       "pid",
                       PID_TASK_STACK,
                       NULL,
                       PID_TASK_PRIORITY,
                       NULL);
  oled_ok = xTaskCreate(Oled_Task,
                        "oled",
                        OLED_TASK_STACK,
                        NULL,
                        OLED_TASK_PRIORITY,
                        NULL);

  if ((remote_ok != pdPASS) ||
      (navigation_ok != pdPASS) ||
      (pid_ok != pdPASS) ||
      (oled_ok != pdPASS))
  {
    return 0U;
  }

  vTaskStartScheduler();
  return 0U;
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t task_handle, char *task_name)
{
  (void)task_handle;
  (void)task_name;

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
