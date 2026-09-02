/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_L_IN1_Pin GPIO_PIN_8
#define MOTOR_L_IN1_GPIO_Port GPIOE
#define MOTOR_L_IN2_Pin GPIO_PIN_10
#define MOTOR_L_IN2_GPIO_Port GPIOE
#define MOTOR_R_IN1_Pin GPIO_PIN_12
#define MOTOR_R_IN1_GPIO_Port GPIOE
#define MOTOR_R_IN2_Pin GPIO_PIN_14
#define MOTOR_R_IN2_GPIO_Port GPIOE
#define FINISH_L_Pin GPIO_PIN_0
#define FINISH_L_GPIO_Port GPIOG
#define FINISH_R_Pin GPIO_PIN_1
#define FINISH_R_GPIO_Port GPIOG
#define START_KEY_Pin GPIO_PIN_2
#define START_KEY_GPIO_Port GPIOG
#define MODE_KEY_Pin GPIO_PIN_3
#define MODE_KEY_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
