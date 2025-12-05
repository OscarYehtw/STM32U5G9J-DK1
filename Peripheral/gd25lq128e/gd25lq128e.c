/**
  ******************************************************************************
  * @file    gd25lq128e.c
  * @modify  Agent Modification Team (Based on mx25um51245g.c for GD25LQ128E QSPI)
  * @brief   This file provides the GD25LQ128E QSPI drivers.
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

/* Includes ------------------------------------------------------------------*/
#include "mxplatform.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @defgroup GD25LQ128E GD25LQ128E
  * @{
  */

/* Private constants for Dummy Cycles */
#define DUMMY_CYCLES_READ_SPI                   GD25LQ128E_DUMMY_CYCLES_READ_SPI
#define DUMMY_CYCLES_READ_QSPI_IO               GD25LQ128E_DUMMY_CYCLES_READ_QUAD_IO

/** @defgroup GD25LQ128E_Exported_Functions GD25LQ128E Exported Functions
  * @{
  */

/**
  * @brief  Get Flash information (128Mbit = 16MByte)
  * @param  pInfo pointer to information structure
  * @retval error status
  */
int32_t GD25LQ128E_GetFlashInfo(GD25LQ128E_Info_t *pInfo)
{
  /* Configure the structure with the memory configuration */
  pInfo->FlashSize              = GD25LQ128E_FLASH_SIZE;
  pInfo->EraseSectorSize        = GD25LQ128E_SECTOR_64K;
  pInfo->EraseSectorsNumber     = (GD25LQ128E_FLASH_SIZE / GD25LQ128E_SECTOR_64K);
  pInfo->EraseSubSectorSize     = GD25LQ128E_SUBSECTOR_4K;
  pInfo->EraseSubSectorNumber   = (GD25LQ128E_FLASH_SIZE / GD25LQ128E_SUBSECTOR_4K);
  pInfo->ProgPageSize           = GD25LQ128E_PAGE_SIZE;
  pInfo->ProgPagesNumber        = (GD25LQ128E_FLASH_SIZE / GD25LQ128E_PAGE_SIZE);

  return GD25LQ128E_OK;
}

/**
  * @brief  Write Enable (WREN) command
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode (SPI/QSPI)
  * @retval error status
  */
int32_t GD25LQ128E_WriteEnable(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode)
{
  XSPI_RegularCmdTypeDef s_command = {0};
  XSPI_AutoPollingTypeDef s_config = {0};

  /* Initialize the write enable command */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0; /* QSPI/SPI uses IO3_0 (4 lines) */
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_WRITE_ENABLE_CMD;  // 06H
  s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = HAL_XSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_command.Instruction    = GD25LQ128E_READ_STATUS_REG_CMD;  //05H
  s_command.AddressMode    = HAL_XSPI_ADDRESS_NONE;
  s_command.DataMode       = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
  s_command.DataDTRMode    = HAL_XSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles    = (Mode == GD25LQ128E_SPI_MODE) ? 0U : DUMMY_CYCLES_READ_QSPI_IO;
  s_command.DataLength     = 1U;
  s_command.DQSMode        = HAL_XSPI_DQS_DISABLE;

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  s_config.MatchValue      = GD25LQ128E_SR_WEL;
  s_config.MatchMask       = GD25LQ128E_SR_WEL;
  s_config.MatchMode       = HAL_XSPI_MATCH_MODE_AND;
  s_config.IntervalTime    = MX25UM51245G_AUTOPOLLING_INTERVAL_TIME;
  s_config.AutomaticStop   = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_XSPI_AutoPolling(Ctx, &s_config, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX25UM51245G_ERROR;
  }

  return GD25LQ128E_OK;
}

/**
  * @brief  Write Disable (WRDI) command
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode (SPI/QSPI)
  * @retval error status
  */
int32_t GD25LQ128E_WriteDisable(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode)
{
  XSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the write disable command */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_WRITE_DISABLE_CMD;  // 04H
  s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = HAL_XSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  return GD25LQ128E_OK;
}

/**
  * @brief  Polling WIP(Write In Progress) bit become to 0
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode (SPI/QSPI)
  * @retval error status
  */
int32_t GD25LQ128E_AutoPollingMemReady(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode)
{
  XSPI_RegularCmdTypeDef  s_command = {0};
  XSPI_AutoPollingTypeDef s_config = {0};

  /* Configure automatic polling mode to wait for memory ready */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_READ_STATUS_REG_CMD;  // 05H
  s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
  s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.DataLength         = 1U;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  s_config.MatchValue    = 0U;
  s_config.MatchMask     = GD25LQ128E_SR_WIP;
  s_config.MatchMode     = HAL_XSPI_MATCH_MODE_AND;
  s_config.IntervalTime  = GD25LQ128E_AUTOPOLLING_INTERVAL_TIME;
  s_config.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  if (HAL_XSPI_AutoPolling(Ctx, &s_config, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  return GD25LQ128E_OK;
}

/**
  * @brief  Reads an amount of data from the memory on STR mode.
  * SPI/QSPI; 1-1-1 or 1-4-4
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start address
  * @param  Size Size of data to read
  * @retval Memory status
  */
int32_t GD25LQ128E_ReadSTR(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                            GD25LQ128E_AddressSize_t AddressSize, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  XSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the read command */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = (Mode == GD25LQ128E_SPI_MODE) ? GD25LQ128E_FAST_READ_CMD : GD25LQ128E_QUAD_IO_FAST_READ_CMD;  // 0BH/EBH
  s_command.AddressMode        = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_ADDRESS_1_LINE : HAL_XSPI_ADDRESS_4_LINES;
  s_command.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressWidth       = HAL_XSPI_ADDRESS_24_BITS;
  s_command.Address            = ReadAddr;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
  s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = (Mode == GD25LQ128E_SPI_MODE) ? DUMMY_CYCLES_READ_SPI : DUMMY_CYCLES_READ_QSPI_IO;
  s_command.DataLength         = Size;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* Reception of the data */
  if (HAL_XSPI_Receive(Ctx, pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  return GD25LQ128E_OK;
}

/**
  * @brief  Writes an amount of data to the memory.
  * SPI/QSPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @param  pData Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size Size of data to write. Range 1 ~ GD25LQ128E_PAGE_SIZE
  * @retval Memory status
  */
int32_t GD25LQ128E_PageProgram(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                                uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  XSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the program command */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_PAGE_PROG_CMD;
  s_command.AddressMode        = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_ADDRESS_1_LINE : HAL_XSPI_ADDRESS_4_LINES;
  s_command.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressWidth       = HAL_XSPI_ADDRESS_24_BITS;
  s_command.Address            = WriteAddr;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
  s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.DataLength         = Size;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Configure the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* Transmission of the data */
  if (HAL_XSPI_Transmit(Ctx, pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }
  
  return GD25LQ128E_OK;
}

/**
  * @brief  Erases the specified block of the memory (4K or 64K).
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @param  BlockAddress Block address to erase
  * @param  BlockSize Block size to erase (4K or 64K)
  * @retval Memory status
  */
int32_t GD25LQ128E_BlockErase(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                              uint32_t BlockAddress, GD25LQ128E_Erase_t BlockSize)
{
  XSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the erase command */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = (BlockSize == GD25LQ128E_ERASE_64K) 
                                  ? GD25LQ128E_SECTOR_ERASE_64K_CMD 
                                  : GD25LQ128E_SUBSECTOR_ERASE_4K_CMD;
  s_command.AddressMode        = HAL_XSPI_ADDRESS_1_LINE; /* Instruction and address lines are usually 1-line for erase */
  s_command.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressWidth       = HAL_XSPI_ADDRESS_24_BITS;
  s_command.Address            = BlockAddress;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = HAL_XSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }
  
  return GD25LQ128E_OK;
}

/**
  * @brief  Erases the entire memory.
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @retval Memory status
  */
int32_t GD25LQ128E_ChipErase(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode)
{
    XSPI_RegularCmdTypeDef s_command = {0};

    /* 1. Enable Write */
    if (GD25LQ128E_WriteEnable(Ctx, Mode) != GD25LQ128E_OK) return GD25LQ128E_ERROR;

    s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
    s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
    s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    s_command.Instruction        = GD25LQ128E_BULK_ERASE_CMD;  // C7H
    s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
    s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    s_command.DataMode           = HAL_XSPI_DATA_NONE;
    s_command.DummyCycles        = 0U;
    s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
    s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

    /* 2. Send the command */
    if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return GD25LQ128E_ERROR;
    
    /* 3. Wait for completion */
    //return GD25LQ128E_AutoPollingMemReady(Ctx, Mode);
    return GD25LQ128E_OK;
}


/**
  * @brief  Reads the value of the Status Register (SR1)
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  Value Status Register 1 value pointer
  * @retval error status
  */
int32_t GD25LQ128E_ReadStatusRegister(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t *Value)
{
  XSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the reading of status register */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_READ_STATUS_REG_CMD;  // 05H (S7-S0) for SR1
  s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
  s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.DataLength         = 1U;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* Reception of the data */
  if (HAL_XSPI_Receive(Ctx, Value, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  return GD25LQ128E_OK;
}

/**
  * @brief  Reads the value of the Status Register 2 (SR2)
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  Value Status Register 2 value pointer
  * @retval error status
  */
int32_t GD25LQ128E_ReadCfgRegister(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t *Value)
{
  XSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the reading of Status Register 2 (SR2) */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_READ_CFG_REG_CMD; /* 0x35 for SR2 */
  s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
  s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.DataLength         = 1U;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* Reception of the data */
  if (HAL_XSPI_Receive(Ctx, Value, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  return GD25LQ128E_OK;
}

/**
  * @brief  Writes the value of the Status/Configuration Register
  * GD25LQ128E uses 0x01 command to write two bytes (SR1 and SR2)
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  Value Configuration Register 2 (SR2) value to be written (for QE, etc.)
  * @retval error status
  */
int32_t GD25LQ128E_WriteStatusRegister(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t Value)
{
  XSPI_RegularCmdTypeDef s_command = {0};
  uint8_t reg[2];
  uint8_t status_reg1;

  /* 1. Read the current Status Register 1 to preserve bits not being changed (SR1) */
  if (GD25LQ128E_ReadStatusRegister(Ctx, Mode, &status_reg1) != GD25LQ128E_OK)
  {
    return GD25LQ128E_ERROR;
  }
  
  /* 2. Prepare the two-byte data buffer (SR1, SR2/CR) */
  /* We only write the QE bit (Bit 1 of SR2) in the passed Value. SR1 is copied from the read value. */
  reg[0] = status_reg1; /* SR1: Copy the current SR1 bits (WIP, WEL, BP, etc.) */
  reg[1] = Value;       /* SR2: Value contains the desired QE setting */

  /* 3. Enable Write operation */
  if (GD25LQ128E_WriteEnable(Ctx, Mode) != GD25LQ128E_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* 4. Initialize the writing of status/configuration register */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_WRITE_STATUS_REG_CMD; /* 0x01h command */
  s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
  s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.DataLength         = 2U; /* Write SR1 (reg[0]) and SR2 (reg[1]) */
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* 5. Send the command and data */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }
  if (HAL_XSPI_Transmit(Ctx, reg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }
  
  /* 6. Wait for the memory to be ready after the write operation */
  return GD25LQ128E_AutoPollingMemReady(Ctx, Mode);
}

int32_t GD25LQ128E_WriteCfgRegister(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t Value)
{
    /* GD devices usually use WriteStatusRegister (0x01) to update both SR1 and SR2/CR. */
    /* If a specific single-byte configuration write is required, it must be implemented here. */
    /* Using WriteStatusRegister(0x01) is the standard approach for QE setting. */
    UNUSED(Ctx);
    UNUSED(Mode);
    UNUSED(Value);
    return GD25LQ128E_ERROR;
}

/**
  * @brief  Enables the Quad Enable (QE) bit in the Status/Configuration Register.
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @retval error status
  */
int32_t GD25LQ128E_EnableQuadMode(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode)
{
  uint8_t cfg_reg = 0U;

  /* 1. Read Status Register 2 (SR2) to check current QE status */
  if (GD25LQ128E_ReadCfgRegister(Ctx, Mode, &cfg_reg) != GD25LQ128E_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* 2. Check if Quad Enable bit is already set (Bit 1 of SR2/CR) */
  if ((cfg_reg & GD25LQ128E_CR_QE_MASK) == GD25LQ128E_CR_QE_MASK)
  {
    return GD25LQ128E_OK;
  }

  /* 3. Set Quad Enable bit (QE bit is Bit 1 of SR2 for GD memories) */
  cfg_reg |= GD25LQ128E_CR_QE_MASK;

  /* 4. Write the combined SR1 and SR2/CR back */
  return GD25LQ128E_WriteStatusRegister(Ctx, Mode, cfg_reg);
}

/**
  * @brief  Reads the JEDEC ID.
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  ID Pointer to the buffer that receives the ID value
  * @retval error status
  */
int32_t GD25LQ128E_ReadID(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t *ID)
{
    XSPI_RegularCmdTypeDef s_command = {0};

    /* Initialize the command */
    s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
    s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
    s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    s_command.Instruction        = GD25LQ128E_READ_JEDEC_ID_CMD;  // 9FH
    s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
    s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    s_command.DataMode           = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
    s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
    s_command.DummyCycles        = 0U;
    s_command.DataLength         = 3U; /* Read Manufacturer (GD: C8), Memory Type (40), Capacity (18) */
    s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
    s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

    /* Send the command */
    if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return GD25LQ128E_ERROR;
    
    /* Receive the ID */
    if (HAL_XSPI_Receive(Ctx, ID, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return GD25LQ128E_ERROR;
    return GD25LQ128E_OK;
}

/* --- Placeholder for remaining functions (if needed) --- */
int32_t GD25LQ128E_ReadDeviceID(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, uint8_t *ID)
{
    XSPI_RegularCmdTypeDef s_command = {0};

    s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
    s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
    s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    s_command.Instruction        = GD25LQ128E_READ_ID_CMD;  // 90H
    s_command.AddressMode        = HAL_XSPI_ADDRESS_1_LINE;
    s_command.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
    s_command.AddressWidth       = HAL_XSPI_ADDRESS_24_BITS;
    s_command.Address            = 0U;
    s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    s_command.DataMode           = (Mode == GD25LQ128E_QSPI_MODE) ? HAL_XSPI_DATA_4_LINES : HAL_XSPI_DATA_1_LINE;
    s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
    s_command.DummyCycles        = 0U;
    s_command.DataLength         = 2U;
    s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
    s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif
    if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return GD25LQ128E_ERROR;
    if (HAL_XSPI_Receive(Ctx, ID, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return GD25LQ128E_ERROR;
    return GD25LQ128E_OK;
}

/**
  * @brief  Enables the memory mapped mode.
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @retval Memory status
  */
int32_t GD25LQ128E_EnableMemoryMappedMode(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                                         GD25LQ128E_AddressSize_t AddressSize)
{
  XSPI_RegularCmdTypeDef s_command = {0};
  XSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};
  
  /* Initialize the read command for Memory Mapped mode */
  s_command.OperationType      = HAL_XSPI_OPTYPE_READ_CFG;  //Read configuration (memory-mapped mode)
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = (Mode == GD25LQ128E_SPI_MODE) ? GD25LQ128E_FAST_READ_CMD : GD25LQ128E_QUAD_IO_FAST_READ_CMD;  // 0BH/EBH
  s_command.AddressMode        = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_ADDRESS_1_LINE : HAL_XSPI_ADDRESS_4_LINES;
  s_command.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressWidth       = HAL_XSPI_ADDRESS_24_BITS;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_DATA_1_LINE : HAL_XSPI_DATA_4_LINES;
  s_command.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = (Mode == GD25LQ128E_SPI_MODE) ? DUMMY_CYCLES_READ_SPI : DUMMY_CYCLES_READ_QSPI_IO;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the read command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* Initialize the program command */
  s_command.OperationType      = HAL_XSPI_OPTYPE_WRITE_CFG;  //Write configuration (memory-mapped mode)
  s_command.Instruction        = GD25LQ128E_PAGE_PROG_CMD;   //02H
  s_command.DummyCycles        = 0U;

  /* Send the write command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation  = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
  s_mem_mapped_cfg.TimeoutPeriodClock = 0x40;

  if (HAL_XSPI_MemoryMapped(Ctx, &s_mem_mapped_cfg) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  return GD25LQ128E_OK;
}

/**
  * @brief  Flash suspend program or erase command
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface select
  * @param  Rate Transfer rate STR or DTR
  * @retval error status
  */
int32_t GD25LQ128E_Suspend(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, GD25LQ128E_Transfer_t Rate)
{
  XSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the suspend command */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE)
                                 ? HAL_XSPI_INSTRUCTION_1_LINE
                                 : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_PROG_ERASE_SUSPEND_CMD;
  s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = HAL_XSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  return GD25LQ128E_OK;
}

/**
  * @brief  Flash resume program or erase command
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface select
  * @param  Rate Transfer rate STR or DTR
  * @retval error status
  */
int32_t GD25LQ128E_Resume(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode, GD25LQ128E_Transfer_t Rate)
{
  XSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the resume command */
  s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
  s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
  s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE)
                                 ? HAL_XSPI_INSTRUCTION_1_LINE
                                 : HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = GD25LQ128E_PROG_ERASE_RESUME_CMD;
  s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode           = HAL_XSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
  s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif

  /* Send the command */
  if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return GD25LQ128E_ERROR;
  }

  return GD25LQ128E_OK;
}

/* Reset Commands *************************************************************/
/**
  * @brief  Flash reset enable command
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface select
  * @retval error status
  */
int32_t GD25LQ128E_ResetEnable(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode)
{
    XSPI_RegularCmdTypeDef s_command = {0};

    /* Initialize the reset enable command */
    s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
    s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
    s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    s_command.Instruction        = GD25LQ128E_RESET_ENABLE_CMD;
    s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
    s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    s_command.DataMode           = HAL_XSPI_DATA_NONE;
    s_command.DummyCycles        = 0U;
    s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
    s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif
    if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return GD25LQ128E_ERROR;
    return GD25LQ128E_OK;
}

/**
  * @brief  Flash reset memory command
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface select
  * @retval error status
  */
int32_t GD25LQ128E_ResetMemory(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode)
{
    XSPI_RegularCmdTypeDef s_command = {0};

    /* Initialize the reset enable command */
    s_command.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    s_command.IOSelect           = HAL_XSPI_SELECT_IO_3_0;
    s_command.InstructionMode    = (Mode == GD25LQ128E_SPI_MODE) ? HAL_XSPI_INSTRUCTION_1_LINE : HAL_XSPI_INSTRUCTION_4_LINES;
    s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    s_command.InstructionWidth   = HAL_XSPI_INSTRUCTION_8_BITS;
    s_command.Instruction        = GD25LQ128E_RESET_MEMORY_CMD;
    s_command.AddressMode        = HAL_XSPI_ADDRESS_NONE;
    s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    s_command.DataMode           = HAL_XSPI_DATA_NONE;
    s_command.DummyCycles        = 0U;
    s_command.DQSMode            = HAL_XSPI_DQS_DISABLE;
#if defined (XSPI_CCR_SIOO)
    s_command.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif
    if (HAL_XSPI_Command(Ctx, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return GD25LQ128E_ERROR;
    return GD25LQ128E_OK;
}

/**
  * @brief  Flash enter deep power-down command
  *         SPI/QPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface select
  * @param  Rate Transfer rate STR or DTR
  * @retval error status
  */
int32_t GD25LQ128E_EnterPowerDown(XSPI_HandleTypeDef *Ctx, GD25LQ128E_Interface_t Mode,
                                    GD25LQ128E_Transfer_t Rate)
{

    return GD25LQ128E_OK;
}

/**
  * @brief  Writes an amount of data to the OSPI memory.
  * @param  Instance  OSPI instance
  * @param  pData     Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size      Size of data to write
  * @retval BSP status
  */
int32_t GD25LQ128E_Write(uint32_t Instance, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  int32_t ret = BSP_ERROR_NONE;
  uint32_t end_addr;
  uint32_t current_size;
  uint32_t current_addr;
  uint32_t data_addr;

  /* Check if the instance is supported */
  if (Instance >= OSPI_NOR_INSTANCES_NUMBER)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* Calculation of the size between the write address and the end of the page */
    current_size = GD25LQ128E_PAGE_SIZE - (WriteAddr % GD25LQ128E_PAGE_SIZE);

    /* Check if the size of the data is less than the remaining place in the page */
    if (current_size > Size)
    {
      current_size = Size;
    }

    /* Initialize the address variables */
    current_addr = WriteAddr;
    end_addr = WriteAddr + Size;
    data_addr = (uint32_t)pData;

    /* Perform the write page by page */
    do
    {
      /* Check if Flash busy ? */
      if (GD25LQ128E_AutoPollingMemReady(&hospi[Instance], Ospi_Ctx[Instance].InterfaceMode) != GD25LQ128E_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }/* Enable write operations */
      else if (GD25LQ128E_WriteEnable(&hospi[Instance], Ospi_Ctx[Instance].InterfaceMode) != GD25LQ128E_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        /* Issue page program command */
        if (GD25LQ128E_PageProgram(&hospi[Instance], Ospi_Ctx[Instance].InterfaceMode,
                                     (uint8_t *)data_addr, current_addr, current_size) != GD25LQ128E_OK)
        {
          ret = BSP_ERROR_COMPONENT_FAILURE;
        }

        if (ret == BSP_ERROR_NONE)
        {
          /* Configure automatic polling mode to wait for end of program */
          if (GD25LQ128E_AutoPollingMemReady(&hospi[Instance], Ospi_Ctx[Instance].InterfaceMode) != GD25LQ128E_OK)
          {
            ret = BSP_ERROR_COMPONENT_FAILURE;
          }
          else
          {
            /* Update the address and size variables for next page programming */
            current_addr += current_size;
            data_addr += current_size;
            current_size = ((current_addr + GD25LQ128E_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : \
                           GD25LQ128E_PAGE_SIZE;
          }
        }
      }
    } while ((current_addr < end_addr) && (ret == BSP_ERROR_NONE));
  }

  /* Return BSP status */
  return ret;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
