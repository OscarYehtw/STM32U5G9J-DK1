/**
  ******************************************************************************
  * @file    gd25lq128e.h
  * @modify  Agent Modification Team (Based on mx25um51245g.h for GD25LQ128E QSPI)
  * @brief   This file contains all the description of the
  * GD25LQ128E QSPI (Quad SPI) memory (128Mbit).
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GD25LQ128E_H
#define GD25LQ128E_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// #include "gd25lq128e_conf.h" /* Assume this file exists for HAL configuration */
/* STM32U5 uses XSPI peripheral for QSPI/OSPI functions */
// #include "stm32u5xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup GD25LQ128E
  * @{
  */
/** @defgroup GD25LQ128E_Exported_Constants GD25LQ128E Exported Constants
  * @{
  */

/**
  * @brief  GD25LQ128E Size configuration
  * GD25LQ128E is 128Mbit (16MBytes)
  */
#define GD25LQ128E_FLASH_SIZE                   (uint32_t)(16*1024*1024)   /* 128 Mbits => 16 MBytes        */
#define GD25LQ128E_PAGE_SIZE                    (uint32_t)256              /* Page size is 256 Bytes        */
#define GD25LQ128E_SECTOR_64K                   (uint32_t)(64 * 1024)      /* 64KBytes Sector               */
#define GD25LQ128E_SUBSECTOR_4K                 (uint32_t)(4  * 1024)      /* 4KBytes Subsector             */


/**
  * @brief  GD25LQ128E Timing configuration (Standard values)
  */
#define GD25LQ128E_BULK_ERASE_MAX_TIME          460000U /* Max Time in us */
#define GD25LQ128E_SECTOR_ERASE_MAX_TIME        1000U
#define GD25LQ128E_SUBSECTOR_4K_ERASE_MAX_TIME  400U
#define GD25LQ128E_WRITE_REG_MAX_TIME           40U

#define GD25LQ128E_RESET_MAX_TIME               100U
#define GD25LQ128E_AUTOPOLLING_INTERVAL_TIME    0x10U

/**
  * @brief  GD25LQ128E Error codes
  */
#define GD25LQ128E_OK                           (0)
#define GD25LQ128E_ERROR                        (-1)

/******************************************************************************
  * @brief  GD25LQ128E Commands (Standard JEDEC/GD QSPI Commands)
  ****************************************************************************/

/***** READ/WRITE MEMORY Operations with 3-Byte Address ***********************/
#define GD25LQ128E_READ_CMD                             0x03U   /*!< Normal Read (1-1-1)                                 */
#define GD25LQ128E_FAST_READ_CMD                        0x0BU   /*!< Fast Read (1-1-1)                                   */
#define GD25LQ128E_PAGE_PROG_CMD                        0x02U   /*!< Page Program (1-1-1)                                */
#define GD25LQ128E_SUBSECTOR_ERASE_4K_CMD               0x20U   /*!< SubSector Erase 4KB (1-1-1)                         */
#define GD25LQ128E_SECTOR_ERASE_32K_CMD                 0x52U   /*!< Sector Erase 32KB (1-1-1)                           */
#define GD25LQ128E_SECTOR_ERASE_64K_CMD                 0xD8U   /*!< Sector Erase 64KB (1-1-1)                           */
#define GD25LQ128E_BULK_ERASE_CMD                       0xC7U   /*!< Bulk Erase (or 0x60)                                */
#define GD25LQ128E_PROG_ERASE_SUSPEND_CMD               0x75U   /*!< Program/Erase Suspend (1-1-1)                       */
#define GD25LQ128E_PROG_ERASE_RESUME_CMD                0x7AU   /*!< Program/Erase Resume (1-1-1)                        */

/***** READ/WRITE MEMORY Operations with 4-Byte Address ***********************/
#define GD25LQ128E_4_BYTE_ADDR_READ_CMD                 0x13U   /*!< Normal Read 4 Byte address (1-1-1)                  */
#define GD25LQ128E_4_BYTE_ADDR_FAST_READ_CMD            0x0CU   /*!< Fast Read 4 Byte address (1-1-1)                    */
#define GD25LQ128E_4_BYTE_PAGE_PROG_CMD                 0x12U   /*!< Page Program 4 Byte Address (1-1-1)                 */
#define GD25LQ128E_4_BYTE_SUBSECTOR_ERASE_4K_CMD        0x21U   /*!< SubSector Erase 4KB 4 Byte Address (1-1-1)          */
#define GD25LQ128E_4_BYTE_SECTOR_ERASE_64K_CMD          0xDCU   /*!< Sector Erase 64KB 4 Byte Address (1-1-1)            */

/***** QSPI (4-line) READ/WRITE MEMORY Operations *****************************/
#define GD25LQ128E_QUAD_INPUT_FAST_READ_CMD             0x6BU   /*!< Quad Input Fast Read (1-1-4)                        */
#define GD25LQ128E_QUAD_IO_FAST_READ_CMD                0xEBU   /*!< Quad I/O Fast Read (1-4-4) - Preferred QSPI Read    */
#define GD25LQ128E_QUAD_PAGE_PROG_CMD                   0x32U   /*!< Quad Page Program (1-1-4)                           */

/***** Setting commands *******************************************************/
#define GD25LQ128E_WRITE_ENABLE_CMD                     0x06U   /*!< Write Enable                                        */
#define GD25LQ128E_WRITE_DISABLE_CMD                    0x04U   /*!< Write Disable                                       */
#define GD25LQ128E_ENTER_4_BYTE_ADDR_MODE_CMD           0xB7U   /*!< Enter 4-Byte Address Mode                           */
#define GD25LQ128E_EXIT_4_BYTE_ADDR_MODE_CMD            0xE9U   /*!< Exit 4-Byte Address Mode                            */

/***** ID/Register commands ***************************************************/
#define GD25LQ128E_READ_STATUS_REG_CMD                  0x05U   /*!< Read Status Register 1 (SR1)                        */
#define GD25LQ128E_WRITE_STATUS_REG_CMD                 0x01U   /*!< Write Status Register 1 & 2 (SR1/SR2)               */
#define GD25LQ128E_READ_CFG_REG_CMD                     0x35U   /*!< Read Status Register 2 (SR2)                        */
#define GD25LQ128E_READ_ID_CMD                          0x90U   /*!< Read Manufacturer/Device ID                         */
#define GD25LQ128E_READ_JEDEC_ID_CMD                    0x9FU   /*!< Read JEDEC ID                                       */

/***** RESET commands *********************************************************/
#define GD25LQ128E_RESET_ENABLE_CMD                     0x66U   /*!< Reset Enable                                        */
#define GD25LQ128E_RESET_MEMORY_CMD                     0x99U   /*!< Reset Memory                                        */

/**
  * @brief  Dummy cycles for QSPI modes (based on 133MHz clock, 1.8V operation)
  */
#define GD25LQ128E_DUMMY_CYCLES_READ_SPI                8U    /*!< Dummy cycles for standard SPI Fast Read (0Bh)   */
#define GD25LQ128E_DUMMY_CYCLES_READ_QUAD_IO            10U   /*!< Dummy cycles for 1-4-4 Quad I/O Fast Read (EBh) */
#define GD25LQ128E_DUMMY_CYCLES_READ_QUAD_INPUT         8U    /*!< Dummy cycles for 1-1-4 Quad Input Fast Read (6Bh) */

/******************************************************************************
  * @brief  GD25LQ128E Registers
  ****************************************************************************/
/* Status Register-SR No.1
   S7 SRP0 Status Register Protection Bit Non-volatile writable
   S6 BP4 Block Protect Bit Non-volatile writable
   S5 BP3 Block Protect Bit Non-volatile writable
   S4 BP2 Block Protect Bit Non-volatile writable
   S3 BP1 Block Protect Bit Non-volatile writable
   S2 BP0 Block Protect Bit Non-volatile writable
   S1 WEL Write Enable Latch Volatile, read only
   S0 WIP Erase/Write In Progress Volatile, read only

   Status Register-SR No.2
   S15 SUS1 Erase Suspend Bit Volatile, read only
   S14 CMP Complement Protect Bit Non-volatile writable
   S13 LB3 Security Register Lock Bit Non-volatile writable (OTP)
   S12 LB2 Security Register Lock Bit Non-volatile writable (OTP)
   S11 LB1 Security Register Lock Bit Non-volatile writable (OTP)
   S10 SUS2 Program Suspend Bit Volatile, read only
   S9 QE Quad Enable Bit Non-volatile writable
   S8 SRP1 Status Register Protection Bit Non-volatile writable
   */

/* Status Register 1 (SR1) */
#define GD25LQ128E_SR_WIP                               0x01U   /*!< Write in progress */
#define GD25LQ128E_SR_WEL                               0x02U   /*!< Write enable latch */

/* Status Register 2 (SR2) / Configuration Register (CR) */
#define GD25LQ128E_CR_QE_MASK                           0x02U   /*!< Quad Enable bit mask (SR2, Bit 1) */


/**
  * @brief  GD25LQ128E Information structure definition
  */
typedef struct
{
  uint32_t FlashSize;                  /*!< Size of the flash */
  uint32_t EraseSectorSize;            /*!< Size of sectors for the erase operation (64KB) */
  uint32_t EraseSectorsNumber;         /*!< Number of 64KB sectors */
  uint32_t EraseSubSectorSize;         /*!< Size of subsector for the erase operation (4KB) */
  uint32_t EraseSubSectorNumber;       /*!< Number of 4KB subsectors */
  uint32_t ProgPageSize;               /*!< Size of pages for the program operation (256B) */
  uint32_t ProgPagesNumber;            /*!< Number of pages */
} GD25LQ128E_Info_t;

/**
  * @brief  GD25LQ128E Interface definitions
  */
typedef enum
{
  GD25LQ128E_SPI_MODE = 0,             /*!< 1-1-1 commands, Default Power on mode */
  GD25LQ128E_QSPI_MODE                 /*!< 1-4-4 commands (Quad I/O) */
} GD25LQ128E_Interface_t;

/**
  * @brief  GD25LQ128E Transfer definitions
  * Note: DTR is not actively used in the provided functions, keeping STR only.
  */
typedef enum
{
  GD25LQ128E_STR_TRANSFER = 0,         /*!< Single Transfer Rate */
  GD25LQ128E_DTR_TRANSFER           /*!< Double Transfer Rate (Not implemented) */
} GD25LQ128E_Transfer_t;


/**
  * @brief  GD25LQ128E Address size definitions
  */
typedef enum
{
  GD25LQ128E_3BYTES_SIZE = 0,          /*!< 3 Bytes address size */
  GD25LQ128E_4BYTES_SIZE               /*!< 4 Bytes address size */
} GD25LQ128E_AddressSize_t;

/**
  * @brief  GD25LQ128E Erase definitions
  */
typedef enum
{
  GD25LQ128E_ERASE_4K = 0,             /*!< 4KBytes Erase (Subsector) */
  GD25LQ128E_ERASE_64K,                /*!< 64KBytes Erase (Sector) */
  GD25LQ128E_ERASE_BULK                /*!< Chip Erase */
} GD25LQ128E_Erase_t;

/** @defgroup GD25LQ128E_Exported_Functions GD25LQ128E Exported Functions
  * @{
  */
int32_t GD25LQ128E_GetFlashInfo(GD25LQ128E_Info_t *pInfo);

/* Read/Write Array Commands (3/4 Byte Address Command Set) *********************/
int32_t GD25LQ128E_ReadSTR(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                            GD25LQ128E_AddressSize_t AddressSize, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);

int32_t GD25LQ128E_PageProgram(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                                GD25LQ128E_AddressSize_t AddressSize, uint8_t *pData, uint32_t WriteAddr,
                                uint32_t Size);

/* Erase Commands ***************************************************************/
int32_t GD25LQ128E_BlockErase(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                              GD25LQ128E_AddressSize_t AddressSize, uint32_t BlockAddress, GD25LQ128E_Erase_t BlockSize);

int32_t GD25LQ128E_ChipErase(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);

/* Register Commands ************************************************************/
int32_t GD25LQ128E_WriteEnable(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);
int32_t GD25LQ128E_WriteDisable(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);
int32_t GD25LQ128E_AutoPollingMemReady(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);
int32_t GD25LQ128E_ReadStatusRegister(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t *Value);
int32_t GD25LQ128E_WriteStatusRegister(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t Value);
int32_t GD25LQ128E_ReadCfgRegister(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t *Value);
int32_t GD25LQ128E_Enter4ByteAddressMode(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);
int32_t GD25LQ128E_Exit4ByteAddressMode(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);
int32_t GD25LQ128E_EnableQuadMode(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);

/* ID Commands ******************************************************************/
int32_t GD25LQ128E_ReadID(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t *ID);
int32_t GD25LQ128E_ReadJEDECID(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t *ID);

/* Reset Commands ***************************************************************/
int32_t GD25LQ128E_ResetEnable(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);
int32_t GD25LQ128E_ResetMemory(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode);

/* Memory Mapped Mode Commands **************************************************/
int32_t GD25LQ128E_EnableMemoryMappedMode(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                                         GD25LQ128E_AddressSize_t AddressSize);


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* GD25LQ128E_H */