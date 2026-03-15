/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_mode.h"
#include "task_comm.h"
#include "task_control_1khz.h"
#include "task_debug.h"
#include "task_monitor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

osThreadId_t ctrlTaskHandle;
const osThreadAttr_t ctrlTask_attributes = {
  .name = "ctrlTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t)osPriorityRealtime,
};
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal,
};
osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
  .name = "monitorTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t)osPriorityBelowNormal,
};
osThreadId_t debugTaskHandle;
const osThreadAttr_t debugTask_attributes = {
  .name = "debugTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCtrlTask(void *argument);
void StartCommTask(void *argument);
void StartMonitorTask(void *argument);
void StartDebugTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  APP_Mode_Bringup();//初始化机器人模式，包括设置默认参数、初始化传感器等
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  ctrlTaskHandle = osThreadNew(StartCtrlTask, NULL, &ctrlTask_attributes);
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);
  monitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &monitorTask_attributes);
  debugTaskHandle = osThreadNew(StartDebugTask, NULL, &debugTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartCtrlTask */
/**
  * @brief  1kHz control loop task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCtrlTask */
void StartCtrlTask(void *argument)//1kHz控制循环任务
{
  /* USER CODE BEGIN StartCtrlTask */
  (void)argument;
  Task_Control1kHz_Entry();
  /* USER CODE END StartCtrlTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
  * @brief  Communication task for vision uplink/downlink.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)//100Hz通信任务
{
  /* USER CODE BEGIN StartCommTask */
  (void)argument;
  Task_Comm_Entry();
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
  * @brief  Safety/status monitor task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)//100Hz监控任务
{
  /* USER CODE BEGIN StartMonitorTask */
  (void)argument;
  Task_Monitor_Entry();
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartDebugTask */
/**
  * @brief  Debug service task.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument)//10Hz调试任务
{
  /* USER CODE BEGIN StartDebugTask */
  (void)argument;
  Task_Debug_Entry();
  /* USER CODE END StartDebugTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
