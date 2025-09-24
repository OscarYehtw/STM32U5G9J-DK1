/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
 * EXCLUDED.                                                                 *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

#include <stdint.h>
#include <stdio.h>

#include "ams_errno.h"
#include "master_i2c.h"
#include "stm32u5xx_hal.h"

#include "ams_device.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);

extern I2C_HandleTypeDef hbus_i2c1;

I2C_HandleTypeDef *hi2c = &hbus_i2c1;

#if DEBUG_TWI
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
        {
            NRF_LOG_RAW_INFO("I2C EVT_HNDLR: NRF_DRV_TWI_EVT_DONE\n");  
            switch (p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_RX:
                {
                    NRF_LOG_RAW_INFO("\t\txfer_type: NRF_DRV_TWI_XFER_RX\n");
                    break;
                }
                case NRF_DRV_TWI_XFER_TX:
                {
                    NRF_LOG_RAW_INFO("\t\txfer_type: NRF_DRV_TWI_XFER_TX\n");
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }    
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        {
            NRF_LOG_RAW_INFO("I2C EVT_HNDLR: NRF_DRV_TWI_EVT_ADDRESS_NACK\n");
            break;
        }
        case NRF_DRV_TWI_EVT_DATA_NACK:
        {
            NRF_LOG_RAW_INFO("I2C EVT_HNDLR: NRF_DRV_TWI_EVT_DATA_NACK\n");
            break;
        }
        default:
        {
            NRF_LOG_RAW_INFO("I2C EVT_HNDLR: Unrecognized Event: %d\n", p_event->type);
            break;
        }
    }
    return;
}
#endif

int ams_i2c_block_read(uint8_t addr, uint8_t reg, uint8_t *data, int size)
{

  /* Timeout is set to 1S */
  while (HAL_I2C_Master_Transmit(hi2c, (uint16_t)addr, (uint8_t *)&reg, 1, 1000) != HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge its address)
       Master restarts communication */
    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
	
	return HAL_ERROR;
  }
  
  while (HAL_I2C_Master_Receive(hi2c, (uint16_t)addr, (uint8_t *)data, size, 10000) != HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }

	return HAL_ERROR;
  }
  
  return HAL_OK;
}

int ams_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data)
{
    return ams_i2c_block_read(addr, reg, data, 1);
}

int ams_i2c_block_write(uint8_t addr, uint8_t reg, uint8_t *data, int size)
{
    int i;
    uint8_t buffer[40] = { 0 };

    buffer[0] = reg;

    
    if (size >= sizeof(buffer) - 1)
    {
        size = sizeof(buffer) - 1;
    }
    for (i = 0; i < size; ++i)
    {
        buffer[i + 1] = data[i];
    }

    /* Timeout is set to 1S */
    while (HAL_I2C_Master_Transmit(hi2c, (uint16_t)addr, (uint8_t *)buffer, size + 1, 1000) != HAL_OK)
    {
      /* Error_Handler() function is called when Timeout error occurs.
         When Acknowledge failure occurs (Slave don't acknowledge its address)
         Master restarts communication */
      if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
	
      return HAL_ERROR;
    }
	
    return HAL_OK;
}

int ams_i2c_write(uint8_t addr, uint8_t *sh, uint8_t reg, uint8_t data)
{
    int ret_val;

    ret_val = ams_i2c_block_write(addr, reg, &data, 1);
    sh[reg] = data;
    
    return (ret_val);
    
}

int ams_i2c_write_direct(uint8_t addr, uint8_t reg, uint8_t data)
{
    int ret_val;

    ret_val = ams_i2c_block_write(addr, reg, &data, 1);
    
    return (ret_val);
    
}

int ams_i2c_modify(uint8_t addr, uint8_t *sh, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t temp;
    int ret_val;

    ams_i2c_read(addr, reg, &temp);
    temp &= ~mask;
    temp |= val;
    ret_val =  ams_i2c_write(addr, sh, reg, temp);

    return(ret_val);
}

ams_errno_t ams_i2c_init(uint8_t scl, uint8_t sda)
{
    ams_errno_t ret;  /* mis-using nordic and ams error code types */

    ret = 0;  /* add twi_handler for debugging - 3rd param */
    if (!ret)
    {
        ret = AMS_SUCCESS;
    }

    return ret;
}

