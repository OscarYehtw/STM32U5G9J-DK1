/**
  ******************************************************************************
  * @file    i2c_config.c
  * @brief   I2C Configuration.
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
#include "ams_device.h"
#include "tcs3410_hwdef.h"
#include "sensirion_i2c_hal.h"
#include "sensirion_sensor.h"
/* USER CODE END Includes */
int i2c_block_write(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *data, int size);

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
  * @brief I2C Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C_Init(void)
{
  uint8_t data[6];

  /* USER CODE BEGIN I2C_Init 0 */
  if (BSP_I2C1_Init() != BSP_ERROR_NONE)
  {
    printf("BSP_I2C1_Init failed!!! \n\r");
  }

  if (BSP_I2C2_Init() != BSP_ERROR_NONE)
  {
    printf("BSP_I2C2_Init failed!!! \n\r");
  }

  #if 0
  if (BSP_I2C3_Init() != BSP_ERROR_NONE)
  {
    printf("BSP_I2C3_Init failed!!! \n\r");
  }
  #endif

  if (BSP_I2C4_Init() != BSP_ERROR_NONE)
  {
    printf("BSP_I2C4_Init failed!!! \n\r");
  }

  #if 0
  if (BSP_I2C5_Init() != BSP_ERROR_NONE)
  {
    printf("BSP_I2C5_Init failed!!! \n\r");
  }
  #endif

  if (BSP_I2C6_Init() != BSP_ERROR_NONE)
  {
    printf("BSP_I2C6_Init failed!!! \n\r");
  }
  /* USER CODE END I2C_Init 0 */

  data[0] = 0;
  i2c_block_write(&hbus_i2c6, 0x20, 0x03, data, 1);
  i2c_block_write(&hbus_i2c6, 0x20, 0x07, data, 1);

}

/**
  * @brief AMS IRQ Initialization Function
  * @param None
  * @retval None
  */
void MX_AMS_Init(void)
{
  if (BSP_I2C2_IsReady(SLAVE_ADDR_0, 2) != BSP_ERROR_NONE)
  {
      printf("als ams_tcs3410 not found on I2C2!!! \n\r");
      //return;
  }

  if (ams_device_init() != AMS_SUCCESS)
  {
      printf("als ams_tcs3410 init failed!!! \n\r");
  }
}

/**
  * @brief AMS IRQ Initialization Function
  * @param None
  * @retval None
  */
void AMS_IRQ_Init(void)
{
#if 0
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : AMS_INT_Pin */
  GPIO_InitStruct.Pin = AMS_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(AMS_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(AMS_INT_EXTI_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(AMS_INT_EXTI_IRQn);
#endif
}

/**
  * @brief Sensirion Sensors Initialization Function
  * @param None
  * @retval None
  */
sensirion_sensor_t sht4x_sensor;   // Temperature + Humidity sensor
sensirion_sensor_t sts4x_sensor;   // Temperature only sensor

void MX_SENSIRION_Init(void)
{
  int16_t error = 0;

  // Initialize I2C HAL only once for all Sensirion sensors
  sensirion_i2c_hal_init();

  // Initialize SHT4x (Temperature & Humidity sensor on I2C6)
  if (BSP_I2C6_IsReady(SENSIRION_I2C_ADDR_44 << 1, 2) == BSP_ERROR_NONE)
  {
      sht4x_sensor.i2c_bus = &hbus_i2c6;
      error = sensirion_sensor_init(&sht4x_sensor, 
                SENSIRION_I2C_ADDR_44, SENSOR_TYPE_SHT4X);
      if (error != 0)
      {
          printf("SHT4x sensor init failed. \n\r");
      }
  }
  else
  {
      printf("SHT4x sensor not found on I2C6. \n\r");
  }

  // Initialize STS4x (Temperature sensor on I2C2)
  if (BSP_I2C2_IsReady(SENSIRION_I2C_ADDR_44 << 1, 2) == BSP_ERROR_NONE)
  {
      sts4x_sensor.i2c_bus = &hbus_i2c2;
      error = sensirion_sensor_init(&sts4x_sensor, 
                SENSIRION_I2C_ADDR_44, SENSOR_TYPE_STS4X);
      if (error != 0)
      {
          printf("STS4x sensor init failed. \n\r");
      }
  }
  else
  {
      printf("STS4x sensor not found on I2C2. \n\r");
  }
}

/**
  * @brief  AMS TCS3410 initialization.
  * @retval ams error status
  */
__weak ams_errno_t ams_device_init(void)
{
    return(AMS_SUCCESS);
}

/* USER CODE END 0 */

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
