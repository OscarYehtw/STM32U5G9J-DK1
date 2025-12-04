#include "gd25lq128e.h"


// This file exposes two functions to enable/disable memory-mapped XIP.


uint8_t GD25_EnableMemoryMapped(XSPI_HandleTypeDef *hxspi)
{
  XSPI_RegularCmdTypeDef cmd = {0};
  XSPI_MemoryMappedTypeDef memcfg = {0};


  // Configure command for quad fast read (0xEB)
  cmd.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
  cmd.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES; // 4-line instruction
  cmd.Instruction = GD25LQ128E_READ_QUAD_IO;
  cmd.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
  //cmd.AddressSize = HAL_XSPI_ADDRESS_24_BITS;
  cmd.DataMode = HAL_XSPI_DATA_4_LINES;
  cmd.DummyCycles = 8; // depends on device/clock
  cmd.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

  if (HAL_XSPI_Command(hxspi, &cmd, GD25_DEFAULT_TIMEOUT) != HAL_OK)
    return GD25_ERROR;


  memcfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE; // no timeout

  if (HAL_XSPI_MemoryMapped(hxspi, &memcfg) != HAL_OK)
    return GD25_ERROR;

  return GD25_OK;
}

//uint8_t *flash = (uint8_t *)0x90000000;  // OSPI region
//uint8_t value = flash[0x1000];

void GD25_DisableMemoryMapped(XSPI_HandleTypeDef *hxspi)
{
  // To disable memory mapped mode call HAL_XSPI_Abort then re-init command if needed.
  HAL_XSPI_Abort(hxspi);
}
