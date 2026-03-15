/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f1xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
#include "main.h"
/* USER CODE BEGIN Includes */
#include "bsp_board.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
  GPIO_InitTypeDef gpio = {0};

  if (hcan == 0 || hcan->Instance != CAN1)
  {
    return;
  }

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* CAN1 remap to PB8/PB9 to match board wiring. */
  __HAL_AFIO_REMAP_CAN1_2();

  gpio.Pin = BSP_V26_CAN1_RX_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BSP_V26_CAN1_RX_PORT, &gpio);

  gpio.Pin = BSP_V26_CAN1_TX_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_V26_CAN1_TX_PORT, &gpio);

  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
  if (hcan == 0 || hcan->Instance != CAN1)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);

  HAL_GPIO_DeInit(BSP_V26_CAN1_RX_PORT, BSP_V26_CAN1_RX_PIN);
  HAL_GPIO_DeInit(BSP_V26_CAN1_TX_PORT, BSP_V26_CAN1_TX_PIN);

  __HAL_RCC_CAN1_CLK_DISABLE();
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
