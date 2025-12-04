/**
  ******************************************************************************
  * @file    spi_config.c
  * @brief   SPI Configuration.
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
XSPI_HandleTypeDef  hospi[OSPI_NOR_INSTANCES_NUMBER] = {0};
GD25_OSPI_NOR_Ctx_t Ospi_Ctx[OSPI_NOR_INSTANCES_NUMBER] = {{
    OSPI_ACCESS_NONE,
    GD25LQ128E_SPI_MODE,
    GD25LQ128E_STR_TRANSFER
  }
};

XSPI_HandleTypeDef  xspi1;
SPI_HandleTypeDef   hspi2;

/* Buffer used for transmission */
uint8_t spiTxBuf[] = "**** SPI Message ****";

/* Buffer used for reception */
uint8_t spiRxBuf[BUFFERSIZE];

/* transfer state */
__IO uint8_t wTransferState = TRANSFER_WAIT;

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
  * @brief  Initializes the OSPI MSP.
  * @param  hospi OSPI handle
  * @retval None
  */
static void OSPI_NOR_MspInit(XSPI_HandleTypeDef *hospi)
{
#if 0
  GPIO_InitTypeDef GPIO_InitStruct;

  /* hospi unused argument(s) compilation warning */
  UNUSED(hospi);

  /* Enable the OctoSPI memory interface clock */
  OSPI_CLK_ENABLE();

  __HAL_RCC_PWR_CLK_ENABLE();
  /* Enable VDDIO2 supply */
  //HAL_PWREx_EnableVddIO2();

  /* Reset the OctoSPI memory interface */
  OSPI_FORCE_RESET();
  OSPI_RELEASE_RESET();

  /* Enable GPIO clocks */
  OSPI_CLK_GPIO_CLK_ENABLE();
  OSPI_CS_GPIO_CLK_ENABLE();
  OSPI_D0_GPIO_CLK_ENABLE();
  OSPI_D1_GPIO_CLK_ENABLE();
  OSPI_D2_GPIO_CLK_ENABLE();
  OSPI_D3_GPIO_CLK_ENABLE();

  /* Enable HSLV GPIOs */
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_CLK_GPIO_PORT, OSPI_CLK_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_CS_GPIO_PORT, OSPI_CS_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_D0_GPIO_PORT, OSPI_D0_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_D1_GPIO_PORT, OSPI_D1_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_D2_GPIO_PORT, OSPI_D2_PIN);
  HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_D3_GPIO_PORT, OSPI_D3_PIN);

  /* OctoSPI CS GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_CS_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = OSPI_CS_PIN_AF;
  HAL_GPIO_Init(OSPI_CS_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI CLK GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_CLK_PIN;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = OSPI_CLK_PIN_AF;
  HAL_GPIO_Init(OSPI_CLK_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D0 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_D0_PIN;
  GPIO_InitStruct.Alternate = OSPI_D0_PIN_AF;
  HAL_GPIO_Init(OSPI_D0_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D1 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_D1_PIN;
  GPIO_InitStruct.Alternate = OSPI_D1_PIN_AF;
  HAL_GPIO_Init(OSPI_D1_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D2 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_D2_PIN;
  GPIO_InitStruct.Alternate = OSPI_D2_PIN_AF;
  HAL_GPIO_Init(OSPI_D2_GPIO_PORT, &GPIO_InitStruct);

  /* OctoSPI D3 GPIO pin configuration  */
  GPIO_InitStruct.Pin       = OSPI_D3_PIN;
  GPIO_InitStruct.Alternate = OSPI_D3_PIN_AF;
  HAL_GPIO_Init(OSPI_D3_GPIO_PORT, &GPIO_InitStruct);

  /* Configure the NVIC for OSPI */
  /* NVIC configuration for OSPI interrupt */
  HAL_NVIC_SetPriority(OCTOSPI1_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(OCTOSPI1_IRQn);
#endif
}

/**
  * @brief XSPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hxspi: XSPI handle pointer
  * @retval None
  */
void HAL_XSPI_MspInit(XSPI_HandleTypeDef *hxspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* hospi unused argument(s) compilation warning */
  UNUSED(hospi);

  if(hxspi->Instance==HSPI1)
  {
    /* USER CODE BEGIN HSPI1_MspInit 0 */

    /* USER CODE END HSPI1_MspInit 0 */
  }
  if(hxspi->Instance==OCTOSPI1)
  {
    /* Enable the OctoSPI memory interface clock */
    OSPI_CLK_ENABLE();

    __HAL_RCC_PWR_CLK_ENABLE();
    /* Enable VDDIO2 supply */
    // HAL_PWREx_EnableVddIO2();

    /* Reset the OctoSPI memory interface */
    OSPI_FORCE_RESET();
    OSPI_RELEASE_RESET();

    /* Enable GPIO clocks */
    OSPI_CLK_GPIO_CLK_ENABLE();
    OSPI_CS_GPIO_CLK_ENABLE();

    /* Enable HSLV GPIOs */
#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
    HAL_GPIO_EnableHighSPeedLowVoltage(DATA_FLASH_CS_GPIO_Port, DATA_FLASH_SPI_CS_Pin);
#else
    HAL_GPIO_EnableHighSPeedLowVoltage(OSPI_CS_GPIO_PORT, OSPI_CS_PIN);
#endif
    HAL_GPIO_EnableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_CLK_Pin);
    HAL_GPIO_EnableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D0_Pin);
    HAL_GPIO_EnableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D1_Pin);
    HAL_GPIO_EnableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D2_Pin);
    HAL_GPIO_EnableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D3_Pin);

#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
    /* OctoSPI CS GPIO pin configuration  */
    GPIO_InitStruct.Pin       = DATA_FLASH_SPI_CS_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(DATA_FLASH_CS_GPIO_Port, &GPIO_InitStruct);
#else
 /* OctoSPI CS GPIO pin configuration  */
    GPIO_InitStruct.Pin       = OSPI_CS_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = OSPI_CS_PIN_AF;
    HAL_GPIO_Init(OSPI_CS_GPIO_PORT, &GPIO_InitStruct);
#endif

    /* OctoSPI CLK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = DATA_FLASH_SPI_CLK_Pin;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OSPI_CLK_GPIO_PORT, &GPIO_InitStruct);

    /* OctoSPI D0 GPIO pin configuration  */
    GPIO_InitStruct.Pin       = DATA_FLASH_SPI_D0_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(DATA_FLASH_SPI_GPIO_Port, &GPIO_InitStruct);

    /* OctoSPI D1 GPIO pin configuration  */
    GPIO_InitStruct.Pin       = DATA_FLASH_SPI_D1_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(DATA_FLASH_SPI_GPIO_Port, &GPIO_InitStruct);

    /* OctoSPI D2 GPIO pin configuration  */
    GPIO_InitStruct.Pin       = DATA_FLASH_SPI_D2_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(DATA_FLASH_SPI_GPIO_Port, &GPIO_InitStruct);

    /* OctoSPI D3 GPIO pin configuration  */
    GPIO_InitStruct.Pin       = DATA_FLASH_SPI_D3_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(DATA_FLASH_SPI_GPIO_Port, &GPIO_InitStruct);

    /* Configure the NVIC for OSPI */
    /* NVIC configuration for OSPI interrupt */
    HAL_NVIC_SetPriority(OCTOSPI1_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(OCTOSPI1_IRQn);
  }
  
}

/**
  * @brief XSPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hxspi: XSPI handle pointer
  * @retval None
  */
void HAL_XSPI_MspDeInit(XSPI_HandleTypeDef *hxspi)
{
  if(hxspi->Instance==HSPI1)
  {
    /* USER CODE BEGIN HSPI1_MspDeInit 0 */

    /* USER CODE END HSPI1_MspDeInit 1 */
  }

  if(hxspi->Instance==OCTOSPI1)
  {
    HAL_NVIC_DisableIRQ(OCTOSPI1_IRQn);

    HAL_GPIO_DisableHighSPeedLowVoltage(DATA_FLASH_CS_GPIO_Port,  DATA_FLASH_SPI_CS_Pin);
    HAL_GPIO_DisableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_CLK_Pin);
    HAL_GPIO_DisableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D0_Pin);
    HAL_GPIO_DisableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D1_Pin);
    HAL_GPIO_DisableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D2_Pin);
    HAL_GPIO_DisableHighSPeedLowVoltage(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D3_Pin);

    HAL_GPIO_DeInit(DATA_FLASH_CS_GPIO_Port,  DATA_FLASH_SPI_CS_Pin);
    HAL_GPIO_DeInit(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_CLK_Pin);
    HAL_GPIO_DeInit(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D0_Pin);
    HAL_GPIO_DeInit(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D1_Pin);
    HAL_GPIO_DeInit(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D2_Pin);
    HAL_GPIO_DeInit(DATA_FLASH_SPI_GPIO_Port, DATA_FLASH_SPI_D3_Pin);

    OSPI_FORCE_RESET();
    OSPI_RELEASE_RESET();
    OSPI_CLK_DISABLE();
  }
}

/**
  * @brief  This function enables delay block.
  * @param  hospi OSPI handle
  * @retval BSP status
  */
static int32_t OSPI_DLYB_Enable(XSPI_HandleTypeDef *hospi)
{
  LL_DLYB_CfgTypeDef dlyb_cfg, dlyb_cfg_test;
  int32_t ret = BSP_ERROR_NONE;
  uint32_t div_value = 4;

  /* Delay block configuration ------------------------------------------------ */
  if (HAL_XSPI_DLYB_GetClockPeriod(hospi, &dlyb_cfg) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  /* PhaseSel is divided by 4 (emperic value)*/
  dlyb_cfg.PhaseSel /= div_value;

  /* save the present configuration for check*/
  dlyb_cfg_test = dlyb_cfg;

  /*set delay block configuration*/
  if (HAL_XSPI_DLYB_SetConfig(hospi, &dlyb_cfg) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  /*check the set value*/
  if (HAL_XSPI_DLYB_GetConfig(hospi, &dlyb_cfg) != HAL_OK)
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  if ((dlyb_cfg.PhaseSel != dlyb_cfg_test.PhaseSel) || (dlyb_cfg.Units != dlyb_cfg_test.Units))
  {
    ret = BSP_ERROR_PERIPH_FAILURE;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  This function reset the OSPI memory.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
static int32_t OSPI_NOR_ResetMemory(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  if (GD25LQ128E_ResetEnable(&hospi[Instance], GD25_OSPI_NOR_SPI_MODE) != GD25LQ128E_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else if (GD25LQ128E_ResetMemory(&hospi[Instance], GD25_OSPI_NOR_SPI_MODE) != GD25LQ128E_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    Ospi_Ctx[Instance].IsInitialized = OSPI_ACCESS_INDIRECT;      /* After reset S/W setting to indirect access  */
    Ospi_Ctx[Instance].InterfaceMode = BSP_OSPI_NOR_SPI_MODE;     /* After reset H/W back to SPI mode by default */
    Ospi_Ctx[Instance].TransferRate  = BSP_OSPI_NOR_STR_TRANSFER; /* After reset S/W setting to STR mode         */

    /* After SWreset CMD, wait in case SWReset occurred during erase operation */
    HAL_Delay(GD25LQ128E_RESET_MAX_TIME);
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Set Flash to desired Interface mode. And this instance becomes current instance.
  *         If current instance running at MMP mode then this function doesn't work.
  *         Indirect -> Indirect
  * @param  Instance  OSPI instance
  * @param  Mode      OSPI mode
  * @param  Rate      OSPI transfer rate
  * @retval BSP status
  */
static int32_t GD25_OSPI_NOR_ConfigFlash(uint32_t Instance, GD25_OSPI_NOR_Interface_t Mode, GD25_OSPI_NOR_Transfer_t Rate)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if MMP mode locked ************************************************/
    if (Ospi_Ctx[Instance].IsInitialized == OSPI_ACCESS_MMP)
    {
      ret = BSP_ERROR_OSPI_MMP_LOCK_FAILURE;
    }
    else
    {
      /* Update OSPI context if all operations are well done */
      if (ret == BSP_ERROR_NONE)
      {
        /* Update current status parameter *****************************************/
        Ospi_Ctx[Instance].IsInitialized = OSPI_ACCESS_INDIRECT;
        Ospi_Ctx[Instance].InterfaceMode = Mode;
        Ospi_Ctx[Instance].TransferRate  = Rate;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Initializes the OSPI interface.
  * @param  Instance   OSPI Instance
  * @param  Init       OSPI Init structure
  * @retval BSP status
  */
static int32_t OSPI_NOR_Init(uint32_t Instance, GD25_OSPI_NOR_Init_t *Init)
{
  int32_t ret;
  GD25_OSPI_NOR_Info_t pInfo;
  MX_OSPI_InitTypeDef ospi_init;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Check if the instance is already initialized */
    if (Ospi_Ctx[Instance].IsInitialized == OSPI_ACCESS_NONE)
    {
      /* Msp OSPI initialization */
      OSPI_NOR_MspInit(&hospi[Instance]);

      /* Get Flash information of one memory */
      (void)GD25LQ128E_GetFlashInfo(&pInfo);

      /* Fill config structure */
      ospi_init.ClockPrescaler = 1;
      ospi_init.MemorySize     = (uint32_t)POSITION_VAL((uint32_t)pInfo.FlashSize);
      ospi_init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
      ospi_init.TransferRate   = (uint32_t) Init->TransferRate;

      /* STM32 OSPI interface initialization */
      if (MX_OSPI_NOR_Init(&hospi[Instance], &ospi_init) != HAL_OK)
      {
        ret = BSP_ERROR_PERIPH_FAILURE;
      }
      /* OSPI Delay Block enable */
      else if (OSPI_DLYB_Enable(&hospi[Instance]) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      /* OSPI memory reset */
      else if (OSPI_NOR_ResetMemory(Instance) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      /* Check if memory is ready */
      else if (GD25LQ128E_AutoPollingMemReady(&hospi[Instance], Ospi_Ctx[Instance].InterfaceMode) != GD25LQ128E_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      /* Configure the memory */
      else if (GD25_OSPI_NOR_ConfigFlash(Instance, Init->InterfaceMode, Init->TransferRate) != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        ret = BSP_ERROR_NONE;
      }
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }
  /* Return BSP status */
  return ret;
}

/**
  * @brief  Return the configuration of the OSPI memory.
  * @param  Instance  OSPI instance
  * @param  pInfo     pointer on the configuration structure
  * @retval BSP status
  */
static int32_t OSPI_NOR_GetInfo(uint32_t Instance, GD25_OSPI_NOR_Info_t *pInfo)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    (void)GD25LQ128E_GetFlashInfo(pInfo);
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Get flash ID 3 Bytes:
  *         Manufacturer ID, Memory type, Memory density
  * @param  Instance  OSPI instance
  * @param  Id Pointer to flash ID bytes
  * @retval BSP status
  */
int32_t OSPI_NOR_ReadID(uint32_t Instance, uint8_t *Id)
{
  int32_t ret;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (GD25LQ128E_ReadID(&hospi[Instance], Ospi_Ctx[Instance].InterfaceMode, Id) != GD25LQ128E_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Configure the OSPI in memory-mapped mode
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t OSPI_NOR_EnableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (GD25LQ128E_EnableMemoryMappedMode(&hospi[Instance], Ospi_Ctx[Instance].InterfaceMode,
                                                 GD25LQ128E_3BYTES_SIZE) != GD25LQ128E_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else /* Update OSPI context if all operations are well done */
    {
      Ospi_Nor_Ctx[Instance].IsInitialized = OSPI_ACCESS_MMP;
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief  Exit form memory-mapped mode
  *         Only 1 Instance can running MMP mode. And it will lock system at this mode.
  * @param  Instance  OSPI instance
  * @retval BSP status
  */
int32_t OSPI_NOR_DisableMemoryMappedMode(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t tickstart = HAL_GetTick();

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (Ospi_Ctx[Instance].IsInitialized != OSPI_ACCESS_MMP)
    {
      ret = BSP_ERROR_OSPI_MMP_UNLOCK_FAILURE;
    }/* Abort MMP back to indirect mode */
    else if (HAL_XSPI_Abort(&hospi[Instance]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      /* Wait until flag is in expected state */
      while ((HAL_XSPI_GET_FLAG(&hospi[Instance], HAL_XSPI_FLAG_BUSY)) != RESET)
      {
        /* Check for the Timeout */
        if (((HAL_GetTick() - tickstart) > hospi[Instance].Timeout) || (hospi[Instance].Timeout == 0U))
        {
          ret = BSP_ERROR_PERIPH_FAILURE;
          break;
        }
      }

      if (ret == BSP_ERROR_NONE)
      {
        /* Configure CR register with functional mode as indirect mode*/
        MODIFY_REG(hospi[Instance].Instance->CR, (OCTOSPI_CR_FMODE), 0U);
        Ospi_Ctx[Instance].IsInitialized = OSPI_ACCESS_INDIRECT;
      }
    }
  }

  /* Return BSP status */
  return ret;
}

/**
  * @brief OSPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_GD25LQ182E_Init(void)
{
  GD25_OSPI_NOR_Info_t sOSPI_NOR_Info;
  GD25_OSPI_NOR_Init_t sOSPI_NOR_Init;
  int32_t status;
  uint8_t devid[3] = {0};

  memset(&sOSPI_NOR_Info, 0, sizeof(sOSPI_NOR_Info));
  memset(&sOSPI_NOR_Init, 0, sizeof(sOSPI_NOR_Init));

  sOSPI_NOR_Init.InterfaceMode = GD25_OSPI_NOR_SPI_MODE;
  sOSPI_NOR_Init.TransferRate  = GD25_OSPI_NOR_STR_TRANSFER;

  /* Initialization of the XSPI / OCTOSPI ------------------------------------------ */
  status = OSPI_NOR_Init(0, &sOSPI_NOR_Init);

  if (status != BSP_ERROR_NONE)
  {
    printf("\r\nOSPI NOR Initialization : Failed\r\n");
  }

  OSPI_NOR_GetInfo(0, &sOSPI_NOR_Info);
  OSPI_NOR_ReadID(0, devid);
  printf("Device ID: %02X %02X %02X\r\n", devid[0], devid[1], devid[2]);
  printf("Access Method: %s, Interface Mode: %s, Transfer Rate: %s\r\n",
         (Ospi_Ctx[0].IsInitialized == OSPI_ACCESS_NONE)        ? "NONE" :
         (Ospi_Ctx[0].IsInitialized == OSPI_ACCESS_INDIRECT)    ? "INDIRECT" :
         (Ospi_Ctx[0].IsInitialized == OSPI_ACCESS_MMP)         ? "MMP" : "UNKNOWN",
         (Ospi_Ctx[0].InterfaceMode == GD25LQ128E_SPI_MODE)     ? "SPI" : "QSPI",
         (Ospi_Ctx[0].TransferRate  == GD25LQ128E_STR_TRANSFER) ? "STR" : "DTR");

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI2_Init(void)
{
#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi2, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
#endif

}

/**
  * @brief SPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInit.Spi2ClockSelection = RCC_SPI2CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOI_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PI0     ------> SPI2_NSS
    PI1     ------> SPI2_SCK
    PI2     ------> SPI2_MISO
    PI3     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = RADAR_SPI2_CS_Pin|RADAR_SPI2_CLK_Pin|RADAR_SPI2_MISO_Pin|RADAR_SPI2_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(RADAR_SPI2_GPIO_Port, &GPIO_InitStruct);

    /* SPI2 interrupt Init */
    HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */

  }

}

/**
  * @brief SPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */
    /* Reset peripherals */
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PI0     ------> SPI2_NSS
    PI1     ------> SPI2_SCK
    PI2     ------> SPI2_MISO
    PI3     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(RADAR_SPI2_GPIO_Port, RADAR_SPI2_CS_Pin|RADAR_SPI2_CLK_Pin|RADAR_SPI2_MISO_Pin|RADAR_SPI2_MOSI_Pin);

    /* SPI2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }

}

/* USER CODE END 0 */

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Transfer in transmission process is complete */

  /* Transfer in reception process is complete */
  if (hspi->Instance == SPI2) {
      wTransferState = TRANSFER_COMPLETE;

      osMessageQueuePut(spi2QueueHandle, (const void *)&wTransferState, 0, 0);
  }

}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    wTransferState = TRANSFER_ERROR;

    osMessageQueuePut(spi2QueueHandle, (const void *)&wTransferState, 0, 0);
}
