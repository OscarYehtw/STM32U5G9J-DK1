/**
  ******************************************************************************
  * @file           : mxplatform.h
  * @brief          : Header for mxplatform.c file.
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
#ifndef __MX_PLATFORM_H
#define __MX_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "stm32u5xx_hal.h"
#include "stm32u5x9j_discovery.h"
#include "stm32u5x9j_discovery_hspi.h"
#include "stm32u5x9j_discovery_ospi.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include "jpeg_utils_conf.h"
#include "stm32_lcd.h"
#include "linked_list.h"
#include "image_320x240_argb8888.h"
#include "life_augmented_argb8888.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported variables --------------------------------------------------------*/
extern DMA_HandleTypeDef   handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef   handle_GPDMA1_Channel0;
extern DMA_HandleTypeDef   hdma_adc4;
extern UART_HandleTypeDef  huart1;
extern UART_HandleTypeDef  huart2;
extern DMA2D_HandleTypeDef hdma2d;
extern GPU2D_HandleTypeDef hgpu2d;
extern JPEG_HandleTypeDef  hjpeg;
extern LTDC_HandleTypeDef  hltdc;
extern DMA_QListTypeDef    ADCQueue;

extern uint32_t aADCxConvertedData[];
extern volatile uint8_t txBusy;
extern uint8_t txByte;
extern uint8_t rxByte;

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void Error_Handler(void);
void MX_PLATFORM_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  2)   /* Size of array aADCxConvertedData[] */

#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOE
#define LCD_RESET_Pin GPIO_PIN_5
#define LCD_RESET_GPIO_Port GPIOD
#define DSI_PWR_ON_Pin GPIO_PIN_5
#define DSI_PWR_ON_GPIO_Port GPIOI
#define VSYNC_FREQ_Pin GPIO_PIN_1
#define VSYNC_FREQ_GPIO_Port GPIOD
#define DSI_BL_CTRL_Pin GPIO_PIN_6
#define DSI_BL_CTRL_GPIO_Port GPIOI
#define RENDER_TIME_Pin GPIO_PIN_0
#define RENDER_TIME_GPIO_Port GPIOD
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define OSPI_CLK_Pin GPIO_PIN_10
#define OSPI_CLK_GPIO_Port GPIOF
#define OSPI_D2_Pin GPIO_PIN_7
#define OSPI_D2_GPIO_Port GPIOF
#define OSPI_D1_Pin GPIO_PIN_9
#define OSPI_D1_GPIO_Port GPIOF
#define OSPI_D4_Pin GPIO_PIN_1
#define OSPI_D4_GPIO_Port GPIOC
#define OSPI_D3_Pin GPIO_PIN_6
#define OSPI_D3_GPIO_Port GPIOF
#define OSPI_D0_Pin GPIO_PIN_8
#define OSPI_D0_GPIO_Port GPIOF
#define OSPI_CS_Pin GPIO_PIN_2
#define OSPI_CS_GPIO_Port GPIOA
#define DSI_TOUCH_INT_Pin GPIO_PIN_8
#define DSI_TOUCH_INT_GPIO_Port GPIOE
#define DSI_TOUCH_INT_EXTI_IRQn EXTI8_IRQn
#define OSPI_D6_Pin GPIO_PIN_3
#define OSPI_D6_GPIO_Port GPIOC
#define MCU_ACTIVE_Pin GPIO_PIN_12
#define MCU_ACTIVE_GPIO_Port GPIOF
#define OSPI_D5_Pin GPIO_PIN_2
#define OSPI_D5_GPIO_Port GPIOC
#define OSPI_D7_Pin GPIO_PIN_0
#define OSPI_D7_GPIO_Port GPIOC
#define OSPI_DQS_Pin GPIO_PIN_1
#define OSPI_DQS_GPIO_Port GPIOA
#define FRAME_RATE_Pin GPIO_PIN_14
#define FRAME_RATE_GPIO_Port GPIOF

#define AMS_INT_Pin       GPIO_PIN_4
#define AMS_INT_GPIO_Port GPIOC
#define AMS_INT_EXTI_IRQn EXTI4_IRQn

#define TOF_LPN_Pin       GPIO_PIN_14
#define TOF_LPN_GPIO_Port GPIOE

#define VFP 50
#define LCD_WIDTH 480
#define VBP 12
#define LCD_FRAME_BUFFER 0x200D0000
#define HACT 480
#define VSYNC 1
#define HFP 1
#define VACT 481
#define IMAGE_HEIGHT 240
#define HBP 1
#define IMAGE_WIDTH 320
#define LCD_HEIGHT 481
#define HSYNC 2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MX_PLATFORM_H */
