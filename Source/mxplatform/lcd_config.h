/**
  ******************************************************************************
  * @file           : lcd_config.h
  * @brief          : Header for lcd_config.c file.
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
#ifndef __LCD_CONFIG_H
#define __LCD_CONFIG_H

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

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void MX_DMA2D_Init(void);
void MX_GPU2D_Init(void);
void MX_DSIHOST_DSI_Init(void);
void MX_LTDC_Init(void);
void MX_LCD_Init(void);
void MX_JPEG_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define HSYNC_WIDTH        10
#define HBP                100
#define HFP                75
#define VSYNC_WIDTH        3
#define VBP                150
#define VFP                120
#define ACTIVE_WIDTH       480
#define ACTIVE_HEIGHT      480

#define HSYNC              (HSYNC_WIDTH - 1)
#define VSYNC              (VSYNC_WIDTH - 1)

#define ACC_HBP            (HSYNC_WIDTH + HBP - 1)
#define ACC_VBP            (VSYNC_WIDTH + VBP - 1)

#define ACC_ACTIVE_WIDTH   (HSYNC_WIDTH + HBP + ACTIVE_WIDTH - 1)
#define ACC_ACTIVE_HEIGHT  (VSYNC_WIDTH + VBP + ACTIVE_HEIGHT - 1)

#define TOTAL_WIDTH        (HSYNC_WIDTH + HBP + ACTIVE_WIDTH + HFP - 1)
#define TOTAL_HEIGHT       (VSYNC_WIDTH + VBP + ACTIVE_HEIGHT + VFP - 1)

#define LCD_WIDTH          480
#define LCD_HEIGHT         481
#define IMAGE_HEIGHT       240
#define IMAGE_WIDTH        320
#define LCD_FRAME_BUFFER   0x200D0000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __LCD_CONFIG_H */
