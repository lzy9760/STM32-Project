#include "drv8323.h"

extern SPI_HandleTypeDef hspi3;   /* 声明：主文件里定义的 SPI 手柄 */
drv_status_t drv_nb = {0,0xFFFF,0xFFFF,0};


static uint16_t SPI_Nb_Send(uint16_t data)
{
  uint16_t rx = 0;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);


  while (HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&data, (uint8_t *)&rx, 1, 1000) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

   HAL_Delay(1);
  
  return rx;
}

static void SPI_Nb_Write_Reg(uint16_t reg, uint16_t data)
{
  //reg左移11位表示门牌号要在bit14~11位上,&0x7FF表示筛掉高5位废话，取低11位，最高位1，表示读，最高位0，表示写
  uint16_t frame = (reg << 11) | (data & 0x7FF);

  SPI_Nb_Send(frame);
}

static uint16_t SPI_Nb_Read_Reg(uint16_t reg)
{
  uint16_t rx;
  rx = SPI_Nb_Send((0x8000 | (reg << 11)));
  return (rx & 0x7FF);
}

void drv8323_Init(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);//叫醒drv8323
    HAL_Delay(2);

    SPI_Nb_Write_Reg(0x06, 0x2C0);
    /*0 << 9   = 0x000    VREF_DIV=0（不分频，CSA中心 = VREF/2 = 1.65V）
      3 << 6   = 0x0C0    CSA_GAIN=40 在 7:6 位
      3 << 0   = 0x003    SEN_LVL=1.0V 在 1:0 位
*/

    //回读校验

    drv_nb.reg06 = SPI_Nb_Read_Reg(0x06);
    drv_nb.writeback_ojbk = (drv_nb.reg06 == 0x2C0);

    //清除历史记录
    SPI_Nb_Write_Reg(0x02, 0x001);
    SPI_Nb_Write_Reg(0x02, 0x000);

    
    //读取故障状态
    drv_nb.fault1 = SPI_Nb_Read_Reg(0x00);
    drv_nb.fault2 = SPI_Nb_Read_Reg(0x01);
    drv_nb.nfault = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);

    drv_nb.reg02 = SPI_Nb_Read_Reg(0x02);

    drv_nb.reg03 = SPI_Nb_Read_Reg(0x03);
    drv_nb.reg04 = SPI_Nb_Read_Reg(0x04);
    drv_nb.reg05 = SPI_Nb_Read_Reg(0x05);
    drv_nb.enable = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

}
/* Diagnostic reads use a separate bounded path; boot SPI behavior is unchanged. */
#define DRV_FAULT_SNAPSHOT_TIMEOUT_MS 3U

volatile drv_fault_snapshot_t drv_fault_snapshot = {
    DRV_FAULT_SNAPSHOT_IDLE, 0U, 0U, 0U, 0U, 0xFFFFU, 0xFFFFU, 0xFFFFU
};

static void drv8323_FaultSnapshotFinish(uint32_t state, uint32_t spi_status)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (drv_fault_snapshot.state == DRV_FAULT_SNAPSHOT_PENDING ||
        drv_fault_snapshot.state == DRV_FAULT_SNAPSHOT_READING)
    {
        GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16U;
        drv_fault_snapshot.elapsed_ms = HAL_GetTick() - drv_fault_snapshot.time_ms;
        drv_fault_snapshot.spi_status = spi_status;
        drv_fault_snapshot.state = state;
    }
    __set_PRIMASK(mask);
}

/* Called with IRQs masked, only after the six PWM inputs have been forced low. */
void drv8323_FaultSnapshotRequest(uint32_t startup_stage)
{
    if (drv_fault_snapshot.state != DRV_FAULT_SNAPSHOT_IDLE) return;
    drv_fault_snapshot.time_ms = HAL_GetTick();
    drv_fault_snapshot.startup_stage = startup_stage;
    drv_fault_snapshot.nfault = (GPIOB->IDR & GPIO_PIN_7) ? 1U : 0U;
    drv_fault_snapshot.enable = (GPIOB->ODR & GPIO_PIN_5) ? 1U : 0U;
    drv_fault_snapshot.tim1_cnt = TIM1->CNT;
    drv_fault_snapshot.tim1_ccer = TIM1->CCER;
    drv_fault_snapshot.tim1_ccr[0] = TIM1->CCR1;
    drv_fault_snapshot.tim1_ccr[1] = TIM1->CCR2;
    drv_fault_snapshot.tim1_ccr[2] = TIM1->CCR3;
    drv_fault_snapshot.tim1_ccr[3] = TIM1->CCR4;
    drv_fault_snapshot.state = DRV_FAULT_SNAPSHOT_PENDING;
    if (!drv_fault_snapshot.enable)
        drv8323_FaultSnapshotFinish(DRV_FAULT_SNAPSHOT_SPI_ERROR, HAL_ERROR);
}

/* Also called by SysTick, before the ordinary safety-fault early return. */
uint8_t drv8323_FaultSnapshotPending(void)
{
    if (drv_fault_snapshot.state != DRV_FAULT_SNAPSHOT_PENDING &&
        drv_fault_snapshot.state != DRV_FAULT_SNAPSHOT_READING) return 0U;
    if ((uint32_t)(HAL_GetTick() - drv_fault_snapshot.time_ms) >=
        DRV_FAULT_SNAPSHOT_TIMEOUT_MS)
    {
        drv8323_FaultSnapshotFinish(DRV_FAULT_SNAPSHOT_TIMEOUT, HAL_TIMEOUT);
        return 0U;
    }
    return 1U;
}

void drv8323_FaultSnapshotAbort(void)
{
    drv8323_FaultSnapshotFinish(DRV_FAULT_SNAPSHOT_ABORTED, HAL_ERROR);
}

/* Foreground only: no HAL_Delay, no retries, no CLR_FLT or ENABLE-high writes. */
void drv8323_FaultSnapshotTask(void)
{
    uint32_t i, gap, mask;
    uint16_t tx, rx;
    HAL_StatusTypeDef status;
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U) return;
    mask = __get_PRIMASK();
    __disable_irq();
    if (!drv8323_FaultSnapshotPending() ||
        drv_fault_snapshot.state != DRV_FAULT_SNAPSHOT_PENDING)
    {
        __set_PRIMASK(mask);
        return;
    }
    drv_fault_snapshot.state = DRV_FAULT_SNAPSHOT_READING;
    __set_PRIMASK(mask);

    if (hspi3.Instance != SPI3 || hspi3.State != HAL_SPI_STATE_READY ||
        hspi3.Lock != HAL_UNLOCKED || !__HAL_RCC_SPI3_IS_CLK_ENABLED())
    {
        drv8323_FaultSnapshotFinish(DRV_FAULT_SNAPSHOT_SPI_ERROR, HAL_BUSY);
        return;
    }
    for (i = 0U; i < 2U; ++i)
    {
        /* At least 1 us CS-high gap, exceeding the DRV8323 400 ns minimum. */
        for (gap = 0U; gap < SystemCoreClock / 1000000U + 1U; ++gap) __NOP();
        if (!drv8323_FaultSnapshotPending()) return;
        tx = (uint16_t)(0x8000U | (i << 11U));
        rx = 0U;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        status = HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&tx,
                                        (uint8_t *)&rx, 1U, 1U);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        /* SysTick may have expired the capture while SPI was running. */
        mask = __get_PRIMASK();
        __disable_irq();
        if (!drv8323_FaultSnapshotPending())
        {
            __set_PRIMASK(mask);
            return;
        }
        if (status != HAL_OK)
        {
            drv8323_FaultSnapshotFinish(DRV_FAULT_SNAPSHOT_SPI_ERROR, status);
            __set_PRIMASK(mask);
            return;
        }
        if (i == 0U) drv_fault_snapshot.fault1 = rx & 0x07FFU;
        else drv_fault_snapshot.fault2 = rx & 0x07FFU;
        drv_fault_snapshot.valid_mask |= 1U << i;
        __set_PRIMASK(mask);
    }
    drv8323_FaultSnapshotFinish(DRV_FAULT_SNAPSHOT_VALID, HAL_OK);
}
