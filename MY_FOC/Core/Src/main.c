/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "drv8323.h"
#include "foc.h"
#include "math.h"
#include "encoder.h"
#include "debug.h"



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
  volatile uint32_t over_cnt = 0;

  volatile uint32_t adc_offset[3];//三路零点，初值0
  volatile uint32_t adc_last_read[3];//校准计数器，初值0
  volatile uint32_t adc_ok_count = 0;//ADC采集成功次数

  volatile float foc_ia;
  volatile float foc_ib;
  volatile float foc_ic;

  volatile uint32_t foc_bumb = 0;



  volatile uint32_t enc_continuous_error = 0;
  

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void adc_nb_calibrate(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 输出关断与采样新鲜度保护。运行期使用下列电流阈值，
 * 上电校准使用 foc.h 中的独立阈值；两者均为软件停机阈值。 */
#define MOTOR_SAFETY_CURRENT_A 15.0f
#define MOTOR_SAFETY_STALE_MS 5U
#define MOTOR_SAFETY_RUN_MS 280000U
volatile uint32_t motor_safety_fault = 0U;
volatile uint32_t motor_fault_enc_glitch_count = 0U;
volatile uint32_t motor_fault_enc_zero_reject_count = 0U;
volatile uint32_t motor_fault_enc_valid_count = 0U;
volatile uint32_t motor_stop_request = 0U;
volatile uint32_t motor_safety_active = 0U;
volatile uint32_t motor_nfault_ignored = 0U;
static volatile uint32_t motor_safety_monitor = 0U;
static volatile uint32_t motor_adc_ms, motor_loop_ms, motor_start_ms;
/* Read-only cycle diagnostics for the ADC-synchronous running current loop.
 * Execution maximum covers foc_curr_loop_nb(), not the entire ADC IRQ. */
volatile uint32_t foc_current_loop_count = 0U;
volatile uint32_t foc_current_loop_period_cycles = 0U;
volatile uint32_t foc_current_loop_max_cycles = 0U;
static uint32_t foc_current_loop_last_cycles;
static volatile uint32_t motor_startup_stage = 0U; /* nFAULT snapshot only */
extern volatile uint32_t enc_spi_fail, enc_parity_fail;

void Motor_SafetyStop(uint32_t reason)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* 保留通道使能，利用现有 OIS=0 和 OSSI 强制六路输入低，再关驱动。 */
    if (__HAL_RCC_TIM1_IS_CLK_ENABLED())
        TIM1->BDTR = (TIM1->BDTR | TIM_BDTR_OSSI) & ~TIM_BDTR_MOE;
    __DSB();
    if (motor_safety_fault == 0U)
    {
        /* 停机后主循环仍读编码器，首次故障时先锁存计数供对比。 */
        motor_fault_enc_glitch_count = enc_glitch_count;
        motor_fault_enc_zero_reject_count = enc_zero_reject_count;
        motor_fault_enc_valid_count = enc_valid_count;
        motor_safety_fault = reason;
        if (reason == 2U) drv8323_FaultSnapshotRequest(motor_startup_stage);
    }
    /* Only nFAULT and repeated bumb stops defer ENABLE; PWM is already off. */
    if ((reason != 2U && reason != 10U) || !drv8323_FaultSnapshotPending())
    {
        drv8323_FaultSnapshotAbort();
        if (__HAL_RCC_GPIOB_IS_CLK_ENABLED())
            GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16U;
    }
    motor_safety_active = 0U;
    foc_bumb = 1U;
    foc_target_id = foc_target_iq = 0.0f;
    __set_PRIMASK(mask);
}

void Motor_SafetyTick(void)
{
    uint32_t now;
    (void)drv8323_FaultSnapshotPending(); /* 3 ms deadline even after a stop */
    if (!motor_safety_monitor || motor_safety_fault) return;
    if (motor_stop_request) { Motor_SafetyStop(6U); return; }
    if (!(GPIOB->IDR & GPIO_PIN_7)) { Motor_SafetyStop(2U); return; }
    if (foc_bumb) { Motor_SafetyStop(10U); return; }
    if (!motor_safety_active) return;
    now = HAL_GetTick();
    if (now - motor_adc_ms >= MOTOR_SAFETY_STALE_MS ||
        now - motor_loop_ms >= MOTOR_SAFETY_STALE_MS)
        Motor_SafetyStop(5U);
    else if (now - motor_start_ms >= MOTOR_SAFETY_RUN_MS)
        Motor_SafetyStop(7U);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin != GPIO_PIN_7) return;
    if (motor_safety_monitor) Motor_SafetyStop(2U);
    else ++motor_nfault_ignored;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Debug_Init();

  drv8323_Init();
  encoder_init();

  /* 原有初始化已完整返回，保留原始 ojBk/寄存器读数；不再发送配置命令。 */
  if (!drv_nb.writeback_ojbk || drv_nb.fault1 || drv_nb.fault2 ||
      !drv_nb.enable || !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
  {
      Motor_SafetyStop(1U);
  }
  else
  {
      uint32_t mask = __get_PRIMASK();
      __disable_irq();
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
      motor_safety_monitor = 1U;
      if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) Motor_SafetyStop(2U);
      __set_PRIMASK(mask);
  }


  if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) Motor_SafetyStop(11U);//ADC触发
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) Motor_SafetyStop(11U);//先启动注入组，供上电零点校准使用

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2100);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2100);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2100);



  /* 先在六路 PWM 未开启时完成零偏校准。 */
  {
      uint32_t trigger = ADC1->CR2 & ADC_CR2_JEXTEN;
      ADC1->CR2 &= ~ADC_CR2_JEXTEN;
      adc_nb_calibrate();
      ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_JEXTEN) | trigger;
  }
  if (adc_ok_count != 1000U ||
      adc_offset[0] < 1000U || adc_offset[0] > 3000U ||
      adc_offset[1] < 1000U || adc_offset[1] > 3000U ||
      adc_offset[2] < 1000U || adc_offset[2] > 3000U)
      Motor_SafetyStop(3U);

  if (!foc_bumb)
  {
      uint32_t errors = enc_spi_fail + enc_parity_fail + enc_sb_count;
      encoder_666_update();
      if (errors != enc_spi_fail + enc_parity_fail + enc_sb_count ||
          !enc_valid ||
          !(enc_deg >= 0.0f && enc_deg < 360.0f)) Motor_SafetyStop(8U);
  }
  if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) Motor_SafetyStop(11U);//之后仅由TIM1_CC4触发，转换完成中断中读取三相电流

  /* 保护与六路启动互斥，避免中断停机后又被后续 HAL_Start 重新开启。 */
  {
      uint32_t mask = __get_PRIMASK();
      uint32_t status = HAL_OK;
      __disable_irq();
      if (!foc_bumb && !motor_stop_request &&
          HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) &&
          (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) == RESET))
      {
          motor_startup_stage = 1U;
          status |= HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
          status |= HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
          status |= HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
          status |= HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
          status |= HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
          status |= HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
          motor_startup_stage = 2U;
          if (status != HAL_OK) Motor_SafetyStop(11U);
          else if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) ||
                   (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET))
              Motor_SafetyStop(2U);
          else
          {
              motor_start_ms = motor_adc_ms = motor_loop_ms = HAL_GetTick();
              motor_safety_active = 1U;
          }
      }
      else if (!foc_bumb) Motor_SafetyStop(motor_stop_request ? 6U : 2U);
      __set_PRIMASK(mask);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
    motor_startup_stage = 3U;
    drv8323_FaultSnapshotTask(); /* Before debug/encoder work and repeated stops */
    motor_loop_ms = HAL_GetTick();
    Debug_Task();
    {
        uint32_t errors = enc_spi_fail + enc_parity_fail + enc_sb_count;
        encoder_666_update();

        if(!enc_valid || errors != enc_spi_fail + enc_parity_fail + enc_sb_count)
        {
          enc_continuous_error++;
        }
        else
        {
          enc_continuous_error = 0;
        }

        if(motor_safety_active && enc_continuous_error >= 50)
        {
          Motor_SafetyStop(8U);
        }
        /* Fresh-angle speed tracking stays in foreground, outside the ADC ISR. */
        if (foc_aligned && !foc_bumb && !motorVars_M1.flagEnableAlignment)
        {
          foc_update_speed();
        }


        
    }
    
    if (ENC_isCalibrationReady() && !foc_bumb)
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        TIM1->BDTR = (TIM1->BDTR | TIM_BDTR_OSSI) & ~TIM_BDTR_MOE;
        __DSB();
        motor_safety_active = 0U;
        __set_PRIMASK(mask);
        ENC_saveCalibration();
        mask = __get_PRIMASK();
        __disable_irq();
        motor_adc_ms = motor_loop_ms = HAL_GetTick();
        if (!foc_bumb && !motor_stop_request && foc_aligned &&
            HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) &&
            (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) == RESET))
        {
            TIM1->BDTR |= TIM_BDTR_MOE;
            motor_safety_active = 1U;
        }
        else if (!foc_bumb) Motor_SafetyStop(motor_stop_request ? 6U : 2U);
        __set_PRIMASK(mask);
    }
    Motor_SafetyTick();


    if(TIM1->SR & TIM_SR_UIF)
    {

      TIM1->SR = ~TIM_SR_UIF;

      if (foc_bumb)                      
    {
        Motor_SafetyStop(10U);
        /* 输出已关断；下面仅保留旧 CCR 值，不再依赖 50% 占空比保护。 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2100);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2100);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2100);   /* 50% 对称零电流 */
        continue;
    }



      if(foc_aligned && !motorVars_M1.flagEnableAlignment)
      {
        
        static uint32_t spd_tick = 0U;
        uint32_t spd_now = HAL_GetTick();
        if ((uint32_t)(spd_now - spd_tick) >= 2U)
        {
          spd_tick = spd_now; 
          foc_pos_loop_nb();
          foc_speed_loop_nb();
        }
      }
      else
      {
        runMotor1Control();


      }

      //安全检测
      if(fabs(foc_ia) > 10.0f || fabs(foc_ib) > 10.0f || fabs(foc_ic) > 10.0f)
    {
      over_cnt++;
      if(over_cnt >= 8)
      {
        foc_target_iq = 0.0f;
        foc_target_id = 0.0f;
        foc_bumb = 1;
        Motor_SafetyStop(4U);
      }
      
    }
    else
    {
      over_cnt = 0;
    }
    }

    }



  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 4099;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 148;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4099);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc->Instance != ADC1)
  {
    return;
  }

  adc_last_read[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
  adc_last_read[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
  adc_last_read[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

  foc_ia = ((int32_t)adc_last_read[0] - (int32_t)adc_offset[0]) * 0.0403f;
  foc_ib = ((int32_t)adc_last_read[1] - (int32_t)adc_offset[1]) * 0.0403f;
  foc_ic = ((int32_t)adc_last_read[2] - (int32_t)adc_offset[2]) * 0.0403f;
  motor_adc_ms = HAL_GetTick();
  if (motor_safety_active)
  {
      uint8_t aligning = (foc_aligned == 0U);
      float current_limit = aligning ? FOC_ALIGN_CURRENT_LIMIT_A : MOTOR_SAFETY_CURRENT_A;
      uint8_t current_bad = (!(fabsf(foc_ia) <= current_limit) ||
                             !(fabsf(foc_ib) <= current_limit) ||
                             !(fabsf(foc_ic) <= current_limit));
      if (current_bad ||
          adc_last_read[0] < 32U || adc_last_read[0] > 4063U ||
          adc_last_read[1] < 32U || adc_last_read[1] > 4063U ||
          adc_last_read[2] < 32U || adc_last_read[2] > 4063U)
      {
          if (aligning) stopMotorControl(FOC_ALIGN_ERROR_OVERCURRENT);
          Motor_SafetyStop(4U);
          return; /* 关断优先，不能在本次回调后半段再写入校准 PWM。 */
      }
      if (aligning) motor1CtrlISR(foc_ia, foc_ib, foc_ic);
      else if (!foc_bumb && !motorVars_M1.flagEnableAlignment &&
               motorVars_M1.motorState == MOTOR_CL_RUNNING)
      {
          uint32_t started = DWT->CYCCNT;
          uint32_t elapsed;
          if (foc_current_loop_count != 0U)
              foc_current_loop_period_cycles = started - foc_current_loop_last_cycles;
          foc_current_loop_last_cycles = started;
          
          //电流环放在adc回调执行，避免主循环其他函数影响电流环时序
          foc_curr_loop_nb();


          elapsed = DWT->CYCCNT - started;
          if (elapsed > foc_current_loop_max_cycles) foc_current_loop_max_cycles = elapsed;
          ++foc_current_loop_count;
      }
  }
}

void adc_nb_calibrate(void)
{
     uint32_t sum[3] = {0,0,0};
     uint32_t n = 1000;


     for(uint32_t i = 0; i < n; i++)
     {

      ADC1->CR2 |= ADC_CR2_JSWSTART;

             if(HAL_ADCEx_InjectedPollForConversion(&hadc1, 10) == HAL_OK)
             {
                 sum[0] += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
                 sum[1] += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
                 sum[2] += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

                 adc_ok_count++;
             }

     }

if(adc_ok_count > 0)
{
    adc_offset[0] = sum[0] / adc_ok_count;
    adc_offset[1] = sum[1] / adc_ok_count;
    adc_offset[2] = sum[2] / adc_ok_count;
}

adc_last_read[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
adc_last_read[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
adc_last_read[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  Motor_SafetyStop(9U);
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
