#include "bsp_board.h"

static GPIO_PinState BSP_Board_LevelToPinState(uint8_t level)
{
  return (level == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

void BSP_Board_InitFromManualV26(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* BMI088 chip-select and indicator output */
  gpio.Pin = BSP_V26_BMI088_CSB_GYRO_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BSP_V26_BMI088_CSB_GYRO_PORT, &gpio);

  gpio.Pin = BSP_V26_BMI088_CSB_ACCL_PIN;
  HAL_GPIO_Init(BSP_V26_BMI088_CSB_ACCL_PORT, &gpio);

  gpio.Pin = BSP_V26_RGB_CTRL_PIN;
  HAL_GPIO_Init(BSP_V26_RGB_CTRL_PORT, &gpio);

  BSP_Board_SetImuCsAccel(1U);
  BSP_Board_SetImuCsGyro(1U);
  BSP_Board_SetIndicatorLevel(0U);

  /* BMI088 interrupt inputs and user switch */
  gpio.Pin = BSP_V26_BMI088_INT_GYRO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BSP_V26_BMI088_INT_GYRO_PORT, &gpio);

  gpio.Pin = BSP_V26_BMI088_INT_ACCL_PIN;
  HAL_GPIO_Init(BSP_V26_BMI088_INT_ACCL_PORT, &gpio);

  gpio.Pin = BSP_V26_IO1_SW_PIN;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BSP_V26_IO1_SW_PORT, &gpio);

  gpio.Pin = BSP_V26_DR16_PIN;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BSP_V26_DR16_PORT, &gpio);
}

void BSP_Board_SetImuCsAccel(uint8_t level)
{
  HAL_GPIO_WritePin(BSP_V26_BMI088_CSB_ACCL_PORT,
                    BSP_V26_BMI088_CSB_ACCL_PIN,
                    BSP_Board_LevelToPinState(level));
}

void BSP_Board_SetImuCsGyro(uint8_t level)
{
  HAL_GPIO_WritePin(BSP_V26_BMI088_CSB_GYRO_PORT,
                    BSP_V26_BMI088_CSB_GYRO_PIN,
                    BSP_Board_LevelToPinState(level));
}

uint8_t BSP_Board_IsImuAccelIntActive(void)
{
  return (HAL_GPIO_ReadPin(BSP_V26_BMI088_INT_ACCL_PORT,
                           BSP_V26_BMI088_INT_ACCL_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

uint8_t BSP_Board_IsImuGyroIntActive(void)
{
  return (HAL_GPIO_ReadPin(BSP_V26_BMI088_INT_GYRO_PORT,
                           BSP_V26_BMI088_INT_GYRO_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

uint8_t BSP_Board_ReadUserSwitch(void)
{
  /* IO_1_SW is an onboard regular push-button: active low. */
  return (HAL_GPIO_ReadPin(BSP_V26_IO1_SW_PORT, BSP_V26_IO1_SW_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
}

void BSP_Board_SetIndicatorLevel(uint8_t level)
{
  HAL_GPIO_WritePin(BSP_V26_RGB_CTRL_PORT,
                    BSP_V26_RGB_CTRL_PIN,
                    BSP_Board_LevelToPinState(level));
}
