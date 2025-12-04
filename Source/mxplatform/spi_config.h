/**
  ******************************************************************************
  * @file           : spi_config.h
  * @brief          : Header for spi_config.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_CONFIG_H
#define __SPI_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported variables --------------------------------------------------------*/
extern XSPI_HandleTypeDef   hospi[OSPI_NOR_INSTANCES_NUMBER];
extern SPI_HandleTypeDef    hspi2;

/* Buffer used for transmission */
extern uint8_t spiTxBuf[];

/* Buffer used for reception */
extern uint8_t spiRxBuf[];
extern __IO uint8_t wTransferState;

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
int32_t OSPI_NOR_EnableMemoryMappedMode(uint32_t Instance);
int32_t OSPI_NOR_DisableMemoryMappedMode(uint32_t Instance);
void MX_GD25LQ182E_Init(void);
void MX_SPI2_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
enum
{
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

#define BUFFERSIZE                       (COUNTOF(spiTxBuf) - 1)

/** @defgroup GD25_OSPI_NOR_Exported_Types OSPI NOR Exported Types
  * @{
  */
#define GD25_OSPI_NOR_Info_t                GD25LQ128E_Info_t
#define GD25_OSPI_NOR_Interface_t           GD25LQ128E_Interface_t
#define GD25_OSPI_NOR_Transfer_t            GD25LQ128E_Transfer_t
#define GD25_OSPI_NOR_Erase_t               GD25LQ128E_Erase_t

typedef struct
{
  OSPI_Access_t               IsInitialized;  /*!<  Instance access Flash method     */
  GD25_OSPI_NOR_Interface_t   InterfaceMode;  /*!<  Flash Interface mode of Instance */
  GD25_OSPI_NOR_Transfer_t    TransferRate;   /*!<  Flash Transfer mode of Instance  */
} GD25_OSPI_NOR_Ctx_t;

typedef struct
{
  GD25_OSPI_NOR_Interface_t   InterfaceMode;      /*!<  Current Flash Interface mode */
  GD25_OSPI_NOR_Transfer_t    TransferRate;       /*!<  Current Flash Transfer rate  */
} GD25_OSPI_NOR_Init_t;

/* Definition for OSPI modes */
#define GD25_OSPI_NOR_SPI_MODE       (GD25_OSPI_NOR_Interface_t)GD25LQ128E_SPI_MODE /* 1 Cmd Line, 1 Address Line
                                                                                       and 1 Data Line    */

/* Definition for OSPI transfer rates */
#define GD25_OSPI_NOR_STR_TRANSFER   (GD25_OSPI_NOR_Transfer_t)GD25LQ128E_STR_TRANSFER   /* Single Transfer Rate */
#define GD25_OSPI_NOR_DTR_TRANSFER   (GD25_OSPI_NOR_Transfer_t)GD25LQ128E_DTR_TRANSFER   /* Double Transfer Rate */

/* OSPI erase types */
#define GD25_OSPI_NOR_ERASE_4K       GD25LQ128E_ERASE_4K
#define GD25_OSPI_NOR_ERASE_64K      GD25LQ128E_ERASE_64K
#define GD25_OSPI_NOR_ERASE_CHIP     GD25LQ128E_ERASE_BULK

/* OSPI block sizes */
#define GD25_OSPI_NOR_BLOCK_4K       GD25LQ128E_SUBSECTOR_4K
#define GD25_OSPI_NOR_BLOCK_64K      GD25LQ128E_SECTOR_64K

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_CONFIG_H */
