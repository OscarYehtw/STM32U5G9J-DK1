/**
  ******************************************************************************
  * @file           : adc_config.h
  * @brief          : Header for adc_config.c file.
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
#ifndef __ADC_CONFIG_H
#define __ADC_CONFIG_H

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
extern ADC_HandleTypeDef    hadc1;
extern ADC_HandleTypeDef    hadc4;

extern __IO uint8_t aAdc1ConvStatus;
extern __IO uint16_t aADC1ConvertedData;
extern uint16_t aADC1ConvertedData_mVolt;
extern uint32_t aADC4ConvertedData[];

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void MX_ADC1_Init(void);
void MX_ADC4_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  8)   /* Size of array aADC4ConvertedData[] */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_CONFIG_H */
