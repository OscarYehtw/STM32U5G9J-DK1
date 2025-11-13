/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "cli.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TX_QUEUE_LEN   4096
#define UART_RX_QUEUE_LEN   64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* External variables --------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t uartTxQueueHandle;
osMessageQueueId_t uartRxQueueHandle;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for GUI_Task */
osThreadId_t GUI_TaskHandle;
const osThreadAttr_t GUI_Task_attributes = {
  .name = "GUI_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for UART_TxTask */
osThreadId_t UART_TxTaskHandle;
const osThreadAttr_t UART_TxTask_attributes = {
  .name = "UART_TxTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 512 * 4,
};
/* Definitions for CLI_Task */
osThreadId_t CLI_TaskHandle;
const osThreadAttr_t CLI_Task_attributes = {
  .name = "CLI_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 16,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//extern portBASE_TYPE IdleTaskHook(void* p);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TouchGFX_Task(void *argument);
void UART_TxTask(void *argument);
void CLI_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
  
   //vTaskSetApplicationTaskTag(NULL, IdleTaskHook);
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GUI_Task */
  GUI_TaskHandle = osThreadNew(TouchGFX_Task, NULL, &GUI_Task_attributes);

  /* creation of UART_TxTask */
  UART_TxTaskHandle = osThreadNew(UART_TxTask, NULL, &UART_TxTask_attributes);
 
  /* creation of GUI_Task */
  CLI_TaskHandle = osThreadNew(CLI_Task, NULL, &CLI_Task_attributes);

  uartTxQueueHandle = osMessageQueueNew(UART_TX_QUEUE_LEN, sizeof(uint8_t), NULL);

  /* Receive an amount of data in interrupt mode */
  uartRxQueueHandle = osMessageQueueNew(UART_RX_QUEUE_LEN, sizeof(uint8_t), NULL);
  HAL_UART_Receive_IT(&huart1, &rxByte, 1);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

int sendchar(int ch) {
    uint8_t c = (uint8_t)(ch & 0xFFU);

    if (ch == '\n') {
        sendchar('\r');
    }

    if (uartTxQueueHandle != NULL) {
        if (osMessageQueuePut(uartTxQueueHandle, &c, 0, 0) != osOK) {
            return -1;
        }
    }

    return ch;
}

int getkey(void) {
    uint8_t ch;
    if (osMessageQueueGet(uartRxQueueHandle, &ch, NULL, osWaitForever) == osOK) {
        return (int)ch;
    }
    return -1;
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

void TouchGFX_Task(void *argument)
{
  uint8_t u8InChar;

  /* USER CODE BEGIN defaultTask */
  //printf("TouchGFX_Task \n\r");

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

void UART_TxTask(void *argument)
{
  uint8_t ch;

  for (;;) {
      if (osMessageQueueGet(uartTxQueueHandle, &ch, NULL, osWaitForever) == osOK) {
          while (txBusy) {
              osDelay(1);
          }

          txBusy = 1;
          txByte = ch;
          HAL_UART_Transmit_IT(&huart1, &txByte, 1);
      }
  }
}

void CLI_Task(void *argument)
{
  for (;;) {
      DispatchCmd ();
      osDelay(10);
    }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
