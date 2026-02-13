/**
  ******************************************************************************
  * @file    adc_config.c
  * @brief   ADC Configuration.
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
/* Definitions of environment analog values */
  /* Value of analog reference voltage (Vref+), connected to analog voltage   */
  /* supply Vdda (unit: mV).                                                  */
  //#define VDDA_APPLI                       (3300UL)
  #define VDDA_APPLI                       (1800UL)

  /* Init variable out of expected ADC conversion data range */
  #define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(ADC1, LL_ADC_RESOLUTION_12B) + 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
ADC_HandleTypeDef    hadc1;
ADC_HandleTypeDef    hadc4;

/* Variable to report status of ADC group regular unitary conversion          */
/*  0: ADC group regular unitary conversion is not completed                  */
/*  1: ADC group regular unitary conversion is completed                      */
/*  2: ADC group regular unitary conversion has not been started yet          */
/*     (initial state)                                                        */
__IO uint8_t aAdc1ConvStatus = 2; /* Variable set into ADC interruption callback */

/* Variables for ADC conversion data */
__IO uint16_t aADC1ConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */

/* Variables for ADC conversion data computation to physical values */
uint16_t aADC1ConvertedData_mVolt = 0;  /* Value of voltage calculated from ADC conversion data (unit: mV) */

uint16_t adc_raw[2];
uint16_t adc_mV[2];
uint16_t adc_val[2];
volatile uint8_t adc_rank = 0;

uint16_t adc4_raw[8];
uint16_t adc4_mV[8];
volatile uint8_t adc4_rank = 0;

/* Variable containing ADC conversions data */
uint32_t   aADC4ConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

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

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{
  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   * PA7 --> ADC1_IN12, ANA_PMIC_VREF_1V2
   * PC5 --> ADC1_IN14, DISP_MCU_NTC_READOUT
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_391CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */
  /* Perform ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC4_SCAN_ENABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoPowerOff = ADC_LOW_POWER_NONE;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
  hadc4.Init.NbrOfConversion = 8;
#else
  hadc4.Init.NbrOfConversion = 2;
#endif
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = ENABLE;
  hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc4.Init.SamplingTimeCommon1 = ADC4_SAMPLETIME_79CYCLES_5;
  hadc4.Init.SamplingTimeCommon2 = ADC4_SAMPLETIME_79CYCLES_5;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC4_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC4_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC4_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC4_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC4_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC4_REGULAR_RANK_6;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC4_REGULAR_RANK_7;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC4_REGULAR_RANK_8;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
#else
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC4_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC4_SAMPLINGTIME_COMMON_1;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC4_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  /* USER CODE BEGIN ADC4_Init 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  MX_ADC4Queue_Config();
  __HAL_LINKDMA(&hadc4, DMA_Handle, hdma_adc4);
  if (HAL_DMAEx_List_LinkQ(&hdma_adc4, &ADC4Queue) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc4,
                        (uint32_t *)aADC4ConvertedData,
                        (ADC_CONVERTED_DATA_BUFFER_SIZE)
                       ) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief ADC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hadc->Instance==ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */

    /* Initializes the peripherals clock */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
    PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA7     ------> ADC1_IN12
    PC5     ------> ADC1_IN14
    */
    GPIO_InitStruct.Pin = PMIC_VREF_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PMIC_VREF_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DISP_NTC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DISP_NTC_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */

  }
  if(hadc->Instance==ADC4)
  {
    /* USER CODE BEGIN ADC4_MspInit 0 */

    /* USER CODE END ADC4_MspInit 0 */

    /* Initializes the peripherals clock */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
    PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC4_CLK_ENABLE();

#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**ADC4 GPIO Configuration
    PC2     ------> ADC4_IN3
    PC3     ------> ADC4_IN4
    PD11    ------> ADC4_IN15
    PD12    ------> ADC4_IN16
    PD13    ------> ADC4_IN17
    PF15    ------> ADC4_IN6
    PG0     ------> ADC4_IN7
    PG1     ------> ADC4_IN8
    */
    GPIO_InitStruct.Pin = WHEEL_VOUT2_Pin|WHEEL_VOUT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(WHEEL_VOUT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ANA_USB1_VBUS_Pin|ANA_VBAT_MEAS_Pin|ANA_BP_MUX_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ANA_ADC4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ANA_DCIN1_MEAS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ANA_DCIN1_MEAS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ANA_THERM_NTC_Pin|ANA_HW_ID_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ANA_HW_ID_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    //HAL_NVIC_SetPriority(ADC4_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(ADC4_IRQn);
#else
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**ADC4 GPIO Configuration
    PA4     ------> ADC4_IN9
    PA5     ------> ADC4_IN10
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ANA_DCIN1_MEAS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ANA_DCIN1_MEAS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ANA_THERM_NTC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ANA_THERM_NTC_GPIO_Port, &GPIO_InitStruct);
#endif

    /* USER CODE BEGIN ADC4_MspInit 1 */

    /* USER CODE END ADC4_MspInit 1 */

  }

}

/**
  * @brief ADC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA7     ------> ADC1_IN12
    PC5     ------> ADC1_IN14
    */
    HAL_GPIO_DeInit(PMIC_VREF_GPIO_Port, PMIC_VREF_Pin);
    HAL_GPIO_DeInit(DISP_NTC_GPIO_Port, DISP_NTC_Pin);

    /* USER CODE BEGIN ADC4_MspDeInit 1 */

    /* USER CODE END ADC4_MspDeInit 1 */
  }
  if(hadc->Instance==ADC4)
  {
    /* USER CODE BEGIN ADC4_MspDeInit 0 */

    /* USER CODE END ADC4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC4_CLK_DISABLE();

#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
    /**ADC4 GPIO Configuration
    PC2     ------> ADC4_IN3
    PC3     ------> ADC4_IN4
    PD11    ------> ADC4_IN15
    PD12    ------> ADC4_IN16
    PD13    ------> ADC4_IN17
    PF15    ------> ADC4_IN6
    PG0     ------> ADC4_IN7
    PG1     ------> ADC4_IN8
    */
    HAL_GPIO_DeInit(WHEEL_VOUT_GPIO_Port, WHEEL_VOUT2_Pin|WHEEL_VOUT1_Pin);
    HAL_GPIO_DeInit(ANA_ADC4_GPIO_Port, ANA_USB1_VBUS_Pin|ANA_VBAT_MEAS_Pin|ANA_BP_MUX_OUT_Pin);
    HAL_GPIO_DeInit(ANA_DCIN1_MEAS_GPIO_Port, ANA_DCIN1_MEAS_Pin);
    HAL_GPIO_DeInit(ANA_HW_ID_GPIO_Port, ANA_THERM_NTC_Pin|ANA_HW_ID_Pin);
#else
    /**ADC4 GPIO Configuration
    PA4     ------> ADC4_IN9
    PA5     ------> ADC4_IN10
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5);
#endif

    /* USER CODE BEGIN ADC4_MspDeInit 1 */

    /* USER CODE END ADC4_MspDeInit 1 */
  }
}

/* USER CODE END 0 */

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  Conversion transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1) {
      #if 0
      /* Retrieve ADC conversion data */
      aADC1ConvertedData = HAL_ADC_GetValue(hadc);

      /* Computation of ADC conversions raw data to physical values           */
      /* using helper macro.                                                  */
      aADC1ConvertedData_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(ADC1, VDDA_APPLI, aADC1ConvertedData, LL_ADC_RESOLUTION_12B);

      /* Update status variable of ADC unitary conversion                     */
      aAdc1ConvStatus = 1;
      osMessageQueuePut(adc12QueueHandle, &aADC1ConvertedData_mVolt, 0, 0);
      #endif

      #if 0
      /* Read Rank1 */
      adc_raw[0] = HAL_ADC_GetValue(hadc);

      /* Read Rank2 */
      adc_raw[1] = HAL_ADC_GetValue(hadc);

      adc_mV[0] = __LL_ADC_CALC_DATA_TO_VOLTAGE(
                      ADC1, VDDA_APPLI, adc_raw[0], LL_ADC_RESOLUTION_12B);

      adc_mV[1] = __LL_ADC_CALC_DATA_TO_VOLTAGE(
                      ADC1, VDDA_APPLI, adc_raw[1], LL_ADC_RESOLUTION_12B);

      osMessageQueuePut(adc12QueueHandle, adc_mV, 0, 0);
      #endif
      uint16_t raw = HAL_ADC_GetValue(hadc);

      adc_raw[adc_rank] = raw;
      adc_mV[adc_rank] =
          __LL_ADC_CALC_DATA_TO_VOLTAGE(
              ADC1, VDDA_APPLI, raw, LL_ADC_RESOLUTION_12B);

      adc_rank++;

      /* 2 channels done */
      if (adc_rank >= 2)
      {
          adc_rank = 0;
          /* example: send both channels */
      }
      osMessageQueuePut(adc12QueueHandle, adc_mV, 0, 0);

  }

  if (hadc->Instance == ADC4) {
      uint16_t raw4 = HAL_ADC_GetValue(hadc);

      adc4_raw[adc4_rank] = raw4;
      adc4_mV[adc4_rank] = __LL_ADC_CALC_DATA_TO_VOLTAGE(ADC4, VDDA_APPLI, raw4, LL_ADC_RESOLUTION_12B);
      //adc4_rank++;

      //if (adc4_rank >= 3)
      //{
      //    adc4_rank = 0;
      //}
      osMessageQueuePut(adc4QueueHandle, adc4_mV, 0, 0);
      //printf("ADC4[%d]: %d mv\n\r", adc4_rank, adc4_mV[adc4_rank]);
  }

}

/**
  * @brief  ADC error interruption callback
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* Note: Disable ADC interruption that caused this error before entering in
           infinite loop below. */

  /* In case of error due to overrun: Disable ADC group regular overrun interruption */
  LL_ADC_DisableIT_OVR(ADC1);

  /* Error reporting */
  Error_Handler();
}
