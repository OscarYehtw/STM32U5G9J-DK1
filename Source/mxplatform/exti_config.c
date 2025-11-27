/**
  ******************************************************************************
  * @file    exti_config.c
  * @brief   EXTI Configuration.
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
  * @brief  Configures EXTI line in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI_IRQHandler_Config(void)
{
#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Configure PA.5 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin  = SSR_INT_Pin;
  HAL_GPIO_Init(SSR_INT_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set line 5 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI5_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI5_IRQn);

  /* Configure PB.8 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin  = RADAR_INT_Pin;
  HAL_GPIO_Init(RADAR_INT_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set line 8 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI8_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI8_IRQn);

  /* Configure PB.15 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin  = BP_DETECT_Pin;
  HAL_GPIO_Init(BP_DETECT_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set line 15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_IRQn);

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin  = PMIC_INT_Pin;
  HAL_GPIO_Init(PMIC_INT_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set line 13 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI13_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

  /* Configure PE.4 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin  = PMIC_UX_BUTTON_Pin;
  HAL_GPIO_Init(PMIC_UX_BUTTON_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set line 4 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* Configure PH.14 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin  = PMIC_PGOOD_Pin;
  HAL_GPIO_Init(PMIC_PGOOD_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set line 14 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI14_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI14_IRQn);

  /* Configure PI.9 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin  = BP_IOEXP_INT_Pin;
  HAL_GPIO_Init(BP_IOEXP_INT_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set line 9 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI9_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI9_IRQn);

  /* Configure PJ.0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin  = TOUCH_INT_Pin;
  HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStructure);

  /* Enable and set line 0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#endif

}

/* USER CODE END 0 */

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
  if (GPIO_Pin == SSR_INT_Pin)
  {
    printf("SSR_Y_MCU_INT_L: Interrupt Triggered!\r\n");
  }
  if (GPIO_Pin == RADAR_INT_Pin)
  {
    printf("RADAR_MCU_INT_L: Interrupt Triggered!\r\n");
  }
  if (GPIO_Pin == BP_DETECT_Pin)
  {
    printf("BP_DETECT: Interrupt Triggered!\r\n");
  }
  if (GPIO_Pin == PMIC_INT_Pin)
  {
    printf("PMIC_MCU_INT_L: Interrupt Triggered!\r\n");
  }
  if (GPIO_Pin == PMIC_UX_BUTTON_Pin)
  {
    printf("PMIC_MCU_UX_BUTTON_L: Interrupt Triggered!\r\n");
  }
#endif

}
