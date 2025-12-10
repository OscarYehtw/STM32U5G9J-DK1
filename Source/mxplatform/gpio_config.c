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
  * @brief Input Pinmux Initialization Function
  * @param None
  * @retval None
  */
void INPUT_PINMUX_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pins : PA.8 MCU_CID0 */
  GPIO_InitStruct.Pin  = CID0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CID0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD.1 MCU_CID1 */
  GPIO_InitStruct.Pin  = CID1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CID1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PH.2 MCU_CID2 */
  GPIO_InitStruct.Pin  = CID2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CID2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB.0 MCU_PMIC_GPIO3 */
  GPIO_InitStruct.Pin  = PMIC_GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PMIC_GPIO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD.4 & PD.9, DISP_MCU_ID0 & DISP_MCU_ID1 */
  GPIO_InitStruct.Pin  = DISP_ID0_Pin|DISP_ID1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISP_PORTD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PH.11 DISP_MCU_ID2 */
  GPIO_InitStruct.Pin  = DISP_ID2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISP_ID2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE.0 MCU_BID0 */
  GPIO_InitStruct.Pin  = BID0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BID0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE.1 DISP_MCU_TE_INT_EDGE */
  GPIO_InitStruct.Pin  = DISP_TE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISP_PORTE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG.12 MCU_BID1 */
  GPIO_InitStruct.Pin  = BID1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BID1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PI.5 MCU_BID2 */
  GPIO_InitStruct.Pin  = BID2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BID2_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief Output Pinmux Initialization Function
  * @param None
  * @retval None
  */
void OUTPUT_PINMUX_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_GPIO1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_GPIO2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PB.9 PB.12 MCU_DBG_GPIO 1 2 */
  GPIO_InitStruct.Pin  = DBG_GPIO2_Pin|DBG_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DBG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_REG_ON_GPIO_Port, BT_REG_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC.4 MCU_BT_REG_ON */
  GPIO_InitStruct.Pin  = BT_REG_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT_REG_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_REG_ON_GPIO_Port, WIFI_REG_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC.7 MCU_WIFI_REG_ON */
  GPIO_InitStruct.Pin  = WIFI_REG_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WIFI_REG_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISP_PORTD_GPIO_Port, DISP_CP_ENN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PD.3 MCU_DISP_CP_ENN */
  GPIO_InitStruct.Pin  = DISP_CP_ENN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISP_PORTD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADAR_PORTD_GPIO_Port, RADAR_PP1V8_LDO_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PD.7 MCU_RADAR_PP1V8_LDO_EN */
  GPIO_InitStruct.Pin  = RADAR_PP1V8_LDO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RADAR_PORTD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISP_PORTD_GPIO_Port, DISP_CP_ENP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PD.8 MCU_DISP_CP_ENP */
  GPIO_InitStruct.Pin  = DISP_CP_ENP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISP_PORTD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADAR_PORTD_GPIO_Port, RADAR_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PD.10 MCU_RADAR_RST_L */
  GPIO_InitStruct.Pin  = RADAR_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RADAR_PORTD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_DEV_WAKE_GPIO_Port, BT_DEV_WAKE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PD.14 MCU_BT_DEV_WAKE */
  GPIO_InitStruct.Pin  = BT_DEV_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT_DEV_WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADAR_PORTD_GPIO_Port, RADAR_OSC_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PD.15 MCU_RADAR_OSC_EN */
  GPIO_InitStruct.Pin  = RADAR_OSC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RADAR_PORTD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(THERM_SOU_EST_GPIO_Port, THERM_SOU_EST_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PE.3 MCU_ADC_THERM_SOU_EST_EN */
  GPIO_InitStruct.Pin  = THERM_SOU_EST_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(THERM_SOU_EST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISP_PORTE_GPIO_Port, DISP_BL_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PE.5 MCU_DISP_BL_EN */
  GPIO_InitStruct.Pin  = DISP_BL_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISP_PORTE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISP_PORTE_GPIO_Port, DISP_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PE.11 MCU_DISP_RST_L */
  GPIO_InitStruct.Pin  = DISP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISP_PORTE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISP_PORTE_GPIO_Port, DISP_IO_SW_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PE.12 MCU_PP1800_DISP_IO_SW_EN */
  GPIO_InitStruct.Pin  = DISP_IO_SW_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DISP_PORTE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SAPS_GPIO_Port, SAPS_CTRL_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PE.13 MCU_SAPS_CTRL_EN */
  GPIO_InitStruct.Pin  = SAPS_CTRL_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SAPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SAPS_GPIO_Port, SAPS_PHASE_DELAY_SEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PE.14 MCU_SAPS_PHASE_DELAY_SEL */
  GPIO_InitStruct.Pin  = SAPS_PHASE_DELAY_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SAPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SAPS_GPIO_Port, SAPS_PULSE_WIDTH_SEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PE.15 MCU_SAPS_PULSE_WIDTH_SEL */
  GPIO_InitStruct.Pin  = SAPS_PULSE_WIDTH_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SAPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WIFI_DEV_WAKE_GPIO_Port, WIFI_DEV_WAKE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PF.13 MCU_WIFI_DEV_WAKE */
  GPIO_InitStruct.Pin  = WIFI_DEV_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WIFI_DEV_WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_HW_ID_GPIO_Port, ADC_HW_ID_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PG.4 MCU_ADC_HW_ID_EN */
  GPIO_InitStruct.Pin  = ADC_HW_ID_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADC_HW_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADAR_PORTG_GPIO_Port, RADAR_PP1V2_LDO_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PG.9 MCU_RADAR_PP1V2_LDO_EN */
  GPIO_InitStruct.Pin  = RADAR_PP1V2_LDO_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RADAR_PORTG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WHEEL_SENSOR_GPIO_Port, WHEEL_SENSOR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PG.11 MCU_WHEEL_SENSOR_EN */
  GPIO_InitStruct.Pin  = WHEEL_SENSOR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WHEEL_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(THERM_SOU_WST_GPIO_Port, THERM_SOU_WST_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PH.10 MCU_ADC_THERM_SOU_WST_EN */
  GPIO_InitStruct.Pin  = THERM_SOU_WST_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(THERM_SOU_WST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_AMBER_EN_GPIO_Port, LED_AMBER_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PI.4 MCU_LED_AMBER_EN_L */
  GPIO_InitStruct.Pin  = LED_AMBER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LED_AMBER_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(THERM_NOR_WST_GPIO_Port, THERM_NOR_WST_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PI.6 MCU_ADC_THERM_NOR_WST_EN */
  GPIO_InitStruct.Pin  = THERM_NOR_WST_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(THERM_NOR_WST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PMIC_RST_GPIO_Port, PMIC_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PI.10 MCU_PMIC_RST_L */
  GPIO_InitStruct.Pin  = PMIC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PMIC_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VBAT_MEAS_EN_GPIO_Port, VBAT_MEAS_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PI.12  */
  GPIO_InitStruct.Pin  = VBAT_MEAS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VBAT_MEAS_EN_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 0 */
/**
  * @brief Wakeup Pin Initialization Function
  * @param None
  * @retval None
  */
void WAKEUP_PIN_Enable(void)
{
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
}

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
  INPUT_PINMUX_Init();
  OUTPUT_PINMUX_Init();
  WAKEUP_PIN_Enable();
#endif

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DSI_PWR_ON_GPIO_Port, DSI_PWR_ON_Pin, GPIO_PIN_RESET);

#if 0
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, VSYNC_FREQ_Pin|RENDER_TIME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MCU_ACTIVE_Pin|FRAME_RATE_Pin, GPIO_PIN_RESET);
#endif

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

#if 0
  /*Configure GPIO pins : LCD_RESET_Pin VSYNC_FREQ_Pin RENDER_TIME_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|VSYNC_FREQ_Pin|RENDER_TIME_Pin;
#endif
  /*Configure GPIO pins : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
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

#if 0
  /*Configure GPIO pins : MCU_ACTIVE_Pin FRAME_RATE_Pin */
  GPIO_InitStruct.Pin = MCU_ACTIVE_Pin|FRAME_RATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
#endif

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
