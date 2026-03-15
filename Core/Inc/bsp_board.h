#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include "main.h"
#include <stdint.h>

#define BSP_BOARD_MANUAL_VERSION "V26.03.05"
#define BSP_BOARD_MANUAL_MCU     "STM32F405RGT6"

/* Manual section 2: connector/pin mapping */
#define BSP_V26_CAN1_RX_PIN      GPIO_PIN_8
#define BSP_V26_CAN1_RX_PORT     GPIOB
#define BSP_V26_CAN1_TX_PIN      GPIO_PIN_9
#define BSP_V26_CAN1_TX_PORT     GPIOB

#define BSP_V26_CAN2_RX_PIN      GPIO_PIN_5
#define BSP_V26_CAN2_RX_PORT     GPIOB
#define BSP_V26_CAN2_TX_PIN      GPIO_PIN_6
#define BSP_V26_CAN2_TX_PORT     GPIOB

#define BSP_V26_USB_DM_PIN       GPIO_PIN_11
#define BSP_V26_USB_DM_PORT      GPIOA
#define BSP_V26_USB_DP_PIN       GPIO_PIN_12
#define BSP_V26_USB_DP_PORT      GPIOA

#define BSP_V26_USART6_RX_PIN    GPIO_PIN_7
#define BSP_V26_USART6_RX_PORT   GPIOC
#define BSP_V26_USART6_TX_PIN    GPIO_PIN_6
#define BSP_V26_USART6_TX_PORT   GPIOC

#define BSP_V26_USART1_TX_PIN    GPIO_PIN_9
#define BSP_V26_USART1_TX_PORT   GPIOA
#define BSP_V26_USART1_RX_PIN    GPIO_PIN_10
#define BSP_V26_USART1_RX_PORT   GPIOA

#define BSP_V26_IO1_SW_PIN       GPIO_PIN_3
#define BSP_V26_IO1_SW_PORT      GPIOC
#define BSP_V26_IO2_PIN          GPIO_PIN_2
#define BSP_V26_IO2_PORT         GPIOC
#define BSP_V26_IO3_PIN          GPIO_PIN_1
#define BSP_V26_IO3_PORT         GPIOC
#define BSP_V26_DR16_PIN         GPIO_PIN_2
#define BSP_V26_DR16_PORT        GPIOD

/* Manual section 3.1: BMI088 mapping */
#define BSP_V26_BMI088_TEMP_PIN      GPIO_PIN_10
#define BSP_V26_BMI088_TEMP_PORT     GPIOB
#define BSP_V26_BMI088_SPI_SCK_PIN   GPIO_PIN_5
#define BSP_V26_BMI088_SPI_SCK_PORT  GPIOA
#define BSP_V26_BMI088_SPI_MISO_PIN  GPIO_PIN_6
#define BSP_V26_BMI088_SPI_MISO_PORT GPIOA
#define BSP_V26_BMI088_SPI_MOSI_PIN  GPIO_PIN_7
#define BSP_V26_BMI088_SPI_MOSI_PORT GPIOA
#define BSP_V26_BMI088_CSB_GYRO_PIN  GPIO_PIN_1
#define BSP_V26_BMI088_CSB_GYRO_PORT GPIOB
#define BSP_V26_BMI088_CSB_ACCL_PIN  GPIO_PIN_4
#define BSP_V26_BMI088_CSB_ACCL_PORT GPIOC
#define BSP_V26_BMI088_INT_GYRO_PIN  GPIO_PIN_5
#define BSP_V26_BMI088_INT_GYRO_PORT GPIOC
#define BSP_V26_BMI088_INT_ACCL_PIN  GPIO_PIN_0
#define BSP_V26_BMI088_INT_ACCL_PORT GPIOB

/* Manual section 5.3: RGB indicator pin */
#define BSP_V26_RGB_CTRL_PIN     GPIO_PIN_8
#define BSP_V26_RGB_CTRL_PORT    GPIOC

void BSP_Board_InitFromManualV26(void);
void BSP_Board_SetImuCsAccel(uint8_t level);
void BSP_Board_SetImuCsGyro(uint8_t level);
uint8_t BSP_Board_IsImuAccelIntActive(void);
uint8_t BSP_Board_IsImuGyroIntActive(void);
uint8_t BSP_Board_ReadUserSwitch(void);
void BSP_Board_SetIndicatorLevel(uint8_t level);

#endif /* BSP_BOARD_H */
