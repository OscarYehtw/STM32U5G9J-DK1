/**
  ******************************************************************************
  * @file    mxplatform.c
  * @brief   MX Platform Initialization.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mxplatform.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
DMA_HandleTypeDef    handle_GPDMA1_Channel1;
DMA_HandleTypeDef    handle_GPDMA1_Channel0;
DMA_HandleTypeDef    hdma_adc1;
DMA_HandleTypeDef    hdma_adc4;
CRC_HandleTypeDef    hcrc;
DCACHE_HandleTypeDef hdcache1;
DCACHE_HandleTypeDef hdcache2;
XSPI_HandleTypeDef   hxspi1;
OSPI_HandleTypeDef   hospi1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* External function --------------------------------------------------------*/
void MX_FREERTOS_Init(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if defined(__ICCARM__)
/* New definition from EWARM V9, compatible with EWARM8 */
int iar_fputc(int ch);
#define PUTCHAR_PROTOTYPE int iar_fputc(int ch)
#elif defined ( __GNUC__) && !defined(__clang__)
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __ICCARM__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  if (osKernelGetState() != osKernelRunning) {
    /* Before starting the scheduler, use blocking mode */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
  }

  /* In the scheduler, use non-blocking mode */
  sendchar(ch);

  return ch;
}
/* USER CODE END 0 */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_PLL2InitTypeDef RCC_PLL2InitTypeDef = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_DSI|RCC_PERIPHCLK_OSPI;
  PeriphClkInit.DsiClockSelection = RCC_DSICLKSOURCE_PLL3;
  PeriphClkInit.LtdcClockSelection = RCC_LTDCCLKSOURCE_PLL3;
  PeriphClkInit.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;

  /*         PLL2_VCO Input = HSE_VALUE/PLL2M = 4 Mhz
   *         PLL2_VCO Output = PLL2_VCO Input * PLL2N = 180 Mhz
   *         PLLOSPICLK = PLL2_VCO Output/PLL2R = 180 Mhz
   * ---->   OSPI presacler = 1 so OSPI frequency = 90 Mhz
   */
  RCC_PLL2InitTypeDef.PLL2ClockOut = RCC_PLL2_DIVQ;
  RCC_PLL2InitTypeDef.PLL2Source = RCC_PLLSOURCE_HSE;
  RCC_PLL2InitTypeDef.PLL2M = 4;
  RCC_PLL2InitTypeDef.PLL2N = 45;
  RCC_PLL2InitTypeDef.PLL2P = 1;
  RCC_PLL2InitTypeDef.PLL2Q = 1;
  RCC_PLL2InitTypeDef.PLL2R = 1;
  RCC_PLL2InitTypeDef.PLL2RGE = RCC_PLLVCIRANGE_0;
  RCC_PLL2InitTypeDef.PLL2FRACN = 0;
  PeriphClkInit.PLL2= RCC_PLL2InitTypeDef;

  PeriphClkInit.PLL3.PLL3Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLL3.PLL3M = 4;
  PeriphClkInit.PLL3.PLL3N = 125;
  PeriphClkInit.PLL3.PLL3P = 8;
  PeriphClkInit.PLL3.PLL3Q = 2;
  PeriphClkInit.PLL3.PLL3R = 24;
  PeriphClkInit.PLL3.PLL3RGE = RCC_PLLVCIRANGE_0;
  PeriphClkInit.PLL3.PLL3FRACN = 0;
  PeriphClkInit.PLL3.PLL3ClockOut = RCC_PLL3_DIVP|RCC_PLL3_DIVR;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* GPDMA1 interrupt Init */
  HAL_NVIC_SetPriority(GPDMA1_Channel10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel10_IRQn);

  hdma_adc4.Instance = GPDMA1_Channel10;
  hdma_adc4.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_adc4.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  hdma_adc4.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  hdma_adc4.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
  hdma_adc4.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&hdma_adc4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&hdma_adc4, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */
#if 0 // Disable the HSPI initialization code generated by CubeMX
  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 4;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 27;
  hospi1.Init.ChipSelectHighTime = 2;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.DQSPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 0;
  if (HAL_OSPI_DLYB_SetConfig(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */
#endif
  BSP_OSPI_NOR_Info_t sOSPI_NOR_Info;
  BSP_OSPI_NOR_Init_t sOSPI_NOR_Init;

  int32_t status;

  memset(&sOSPI_NOR_Info, 0, sizeof(sOSPI_NOR_Info));
  memset(&sOSPI_NOR_Init, 0, sizeof(sOSPI_NOR_Init));

  //sOSPI_NOR_Init.InterfaceMode = BSP_OSPI_NOR_OPI_MODE;
  //sOSPI_NOR_Init.TransferRate  = BSP_OSPI_NOR_DTR_TRANSFER;
  sOSPI_NOR_Init.InterfaceMode = BSP_OSPI_NOR_SPI_MODE;
  sOSPI_NOR_Init.TransferRate  = BSP_OSPI_NOR_STR_TRANSFER;

  status = BSP_OSPI_NOR_Init(0, &sOSPI_NOR_Init);

  if (status != BSP_ERROR_NONE)
  {
    printf("\r\nOSPI NOR Initialization : Failed");
    printf("\r\nOSPI NOR Test Aborted");
    printf("\r\n");
  }

  BSP_OSPI_NOR_GetInfo(0, &sOSPI_NOR_Info);

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief HSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HSPI1_Init(void)
{

  /* USER CODE BEGIN HSPI1_Init 0 */
#if 0 // Disable the HSPI initialization code generated by CubeMX
  /* USER CODE END HSPI1_Init 0 */

  /* USER CODE BEGIN HSPI1_Init 1 */

  /* USER CODE END HSPI1_Init 1 */
  /* HSPI1 parameter configuration*/
  hxspi1.Instance = HSPI1;
  hxspi1.Init.FifoThresholdByte = 2;
  hxspi1.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi1.Init.MemoryType = HAL_XSPI_MEMTYPE_APMEM_16BITS;
  hxspi1.Init.MemorySize = HAL_XSPI_SIZE_512MB;
  hxspi1.Init.ChipSelectHighTimeCycle = 1;
  hxspi1.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi1.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi1.Init.WrapSize = HAL_XSPI_WRAP_32_BYTES;
  hxspi1.Init.ClockPrescaler = 0;
  hxspi1.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi1.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
  hxspi1.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_64KB;
  hxspi1.Init.MaxTran = 0;
  hxspi1.Init.Refresh = 0;
  if (HAL_XSPI_Init(&hxspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HSPI1_Init 2 */
#endif
  BSP_HSPI_RAM_Cfg_t sHSPI_Init;

  sHSPI_Init.LatencyType      = BSP_HSPI_RAM_FIXED_LATENCY;
  sHSPI_Init.BurstType        = BSP_HSPI_RAM_LINEAR_BURST;
  sHSPI_Init.BurstLength      = BSP_HSPI_RAM_BURST_16_BYTES;
  sHSPI_Init.ReadLatencyCode  = BSP_HSPI_RAM_READ_LATENCY_6;
  sHSPI_Init.WriteLatencyCode = BSP_HSPI_RAM_WRITE_LATENCY_6;
  sHSPI_Init.IOMode           = BSP_HSPI_RAM_IO_X16_MODE;

  if (BSP_HSPI_RAM_Init(0, &sHSPI_Init) != BSP_ERROR_NONE)
  {
    printf("\r\nHSPI RAM Initialization : Failed");
    printf("\r\nHSPI RAM Test Aborted");
    printf("\r\n");
    Error_Handler();
  }

  if(BSP_HSPI_RAM_EnableMemoryMappedMode(0) != BSP_ERROR_NONE)
  {
    printf("\r\nHSPI RAM Mem-Mapped Cfg : Failed");
    printf("\r\nHSPI RAM Test Aborted");
    printf("\r\n");
    Error_Handler();
  }
  /* USER CODE END HSPI1_Init 2 */

}

/**
  * @brief DCACHE1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCACHE1_Init(void)
{

  /* USER CODE BEGIN DCACHE1_Init 0 */

  /* USER CODE END DCACHE1_Init 0 */

  /* USER CODE BEGIN DCACHE1_Init 1 */

  /* USER CODE END DCACHE1_Init 1 */
  hdcache1.Instance = DCACHE1;
  hdcache1.Init.ReadBurstType = DCACHE_READ_BURST_INCR;
  if (HAL_DCACHE_Init(&hdcache1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCACHE1_Init 2 */

  /* USER CODE END DCACHE1_Init 2 */

}

/**
  * @brief DCACHE2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCACHE2_Init(void)
{

  /* USER CODE BEGIN DCACHE2_Init 0 */

  /* USER CODE END DCACHE2_Init 0 */

  /* USER CODE BEGIN DCACHE2_Init 1 */

  /* USER CODE END DCACHE2_Init 1 */
  hdcache2.Instance = DCACHE2;
  hdcache2.Init.ReadBurstType = DCACHE_READ_BURST_INCR;
  if (HAL_DCACHE_Init(&hdcache2) != HAL_OK)
  {
    Error_Handler();
  }
   __HAL_RCC_SYSCFG_CLK_ENABLE();
   HAL_SYSCFG_DisableSRAMCached();
  /* USER CODE BEGIN DCACHE2_Init 2 */

  /* USER CODE END DCACHE2_Init 2 */

}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
  * @brief MX Platform Initialization Function
  * @retval None
  */
void MX_PLATFORM_Init(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* Initialize all configured peripherals */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_UART_Init();
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_CRC_Init();
  MX_SPI2_Init();
  MX_GD25LQ182E_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_DMA2D_Init();
  MX_ICACHE_Init();
  MX_GPU2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_LTDC_Init();
  MX_LCD_Init();
#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 0)
  //MX_OCTOSPI1_Init();
  //MX_HSPI1_Init();
#endif
  MX_DCACHE1_Init();
  MX_DCACHE2_Init();
  MX_I2C_Init();
  MX_ADC1_Init();
  MX_ADC4_Init();
  MX_JPEG_Init();
  MX_AMS_Init();
  MX_SENSIRION_Init();
  EXTI_IRQHandler_Config();

  //MX_TouchGFX_Init();

  /* Call PreOsInit function */
  //MX_TouchGFX_PreOSInit();

  /* Init scheduler */
  osKernelInitialize();
  /* Call init function for freertos objects (in app_freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

}
