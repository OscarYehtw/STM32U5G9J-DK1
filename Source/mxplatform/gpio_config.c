/**
  ******************************************************************************
  * @file    gpio_config.c
  * @brief   GPIO Configuration.
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
  /*Configure GPIO pins : MCU_PMIC_GPIO3 */
  GPIO_InitStruct.Pin  = PMIC_GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PMIC_GPIO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_GPIO1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_GPIO2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MCU_DBG_GPIO 1 2 */
  GPIO_InitStruct.Pin  = DBG_GPIO2_Pin|DBG_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DBG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_REG_ON_GPIO_Port, BT_REG_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MCU_BT_REG_ON */
  GPIO_InitStruct.Pin  = BT_REG_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT_REG_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_REG_ON_GPIO_Port, WIFI_REG_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MCU_WIFI_REG_ON */
  GPIO_InitStruct.Pin  = WIFI_REG_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WIFI_REG_ON_GPIO_Port, &GPIO_InitStruct);

  /* Enable WakeUp Pin PWR_WAKEUP_PIN1 connected to PE.4 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH_2);

  /* Enable WakeUp Pin PWR_WAKEUP_PIN2 connected to PC.13 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH_1);

  /* Enable WakeUp Pin PWR_WAKEUP_PIN3 connected to PE.6 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH_0);

  /* Enable WakeUp Pin PWR_WAKEUP_PIN5 connected to PB.8 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN5_HIGH_2);

  /* Enable WakeUp Pin PWR_WAKEUP_PIN6 connected to PA.5 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6_HIGH_1);

  /* Enable WakeUp Pin PWR_WAKEUP_PIN7 connected to PB.15 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_0);

  /* Enable WakeUp Pin PWR_WAKEUP_PIN8 connected to PF.2 */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN8_HIGH_0);

  /* Clear all related wakeup flags*/
  //__HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_FLAG2);
  /* Enter the Standby mode */
  //HAL_PWR_EnterSTANDBYMode();
#else
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

#endif

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DSI_PWR_ON_GPIO_Port, DSI_PWR_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, VSYNC_FREQ_Pin|RENDER_TIME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MCU_ACTIVE_Pin|FRAME_RATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  /* Comms enable. Drive this pin to logic 0 to disable the I2C comms. 
                   Drive this pin to logic 1 to enable I2C comms.*/
  HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RESET_Pin VSYNC_FREQ_Pin RENDER_TIME_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|VSYNC_FREQ_Pin|RENDER_TIME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DSI_PWR_ON_Pin */
  GPIO_InitStruct.Pin = DSI_PWR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DSI_PWR_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DSI_TOUCH_INT_Pin */
  GPIO_InitStruct.Pin = DSI_TOUCH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DSI_TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_ACTIVE_Pin FRAME_RATE_Pin */
  GPIO_InitStruct.Pin = MCU_ACTIVE_Pin|FRAME_RATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF LPN */
  GPIO_InitStruct.Pin = TOF_LPN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TOF_LPN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI8_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI8_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE END 0 */

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
