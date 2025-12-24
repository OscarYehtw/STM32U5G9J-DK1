/**
  ******************************************************************************
  * @file    sdio_config.c
  * @brief   SDIO Configuration.
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
typedef struct
{
    uint32_t result_count;
} wcm_scan_data_t;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Wi-Fi Connection related parameters */
#if  !defined(WIFI_CONNECT_ENABLE)
   #define WIFI_CONNECT_ENABLE             (0) /* Enable/Disable Wi-Fi Connect */
#endif /* (WIFI_CONNECT_ENABLE) */
#if  !defined(WIFI_SSID)
   #define WIFI_SSID                       "WIFI_SSID"
#endif /* (WIFI_SSID) */
#if  !defined(WIFI_PASSWORD)
   #define WIFI_PASSWORD                   "WIFI_PASSWORD"
#endif /* (WIFI_PASSWORD) */

#define WIFI_SECURITY                       CY_WCM_SECURITY_WPA2_AES_PSK
#define MAX_WIFI_RETRY_COUNT                (3u)
#define WIFI_CONN_RETRY_INTERVAL_MSEC       (100u)

/* The delay in milliseconds between successive scans.*/
#define SCAN_DELAY_MS                           (3000u)

#define PRINT_SCAN_TEMPLATE() \
    printf("\r\n--------------------------------------------------" \
           "--------------------------------------------------\r\n" \
           "  #                  SSID                  RSSI   Channel       " \
           "MAC Address              Security\r\n" \
           "--------------------------------------------------" \
           "--------------------------------------------------\r\n");

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
SD_HandleTypeDef SDHandle = { .Instance = SDMMC1 };

/* Connection parameters to the Wi-Fi connection manager (WCM). */
//cy_wcm_connect_params_t connect_param;
//cy_wcm_ip_address_t     ip_addr;
wcm_scan_data_t         scan_data;
cy_wcm_mac_t            last_bssid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern osThreadId_t WiFi_TaskHandle;

/* USER CODE END EV */

/* External function --------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
* @brief SD MSP Initialization
* This function configures the hardware resources used in this example
* @param hsd: SD handle pointer
* @retval None
*/
void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hsd->Instance==SDMMC1)
  {
  /* USER CODE BEGIN SDMMC1_MspInit 0 */

  /* USER CODE END SDMMC1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_CLK48;
    PeriphClkInit.IclkClockSelection = RCC_CLK48CLKSOURCE_HSI48;
    PeriphClkInit.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_CLK48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_SDMMC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDMMC1 GPIO Configuration
    PC11     ------> SDMMC1_D3
    PC10     ------> SDMMC1_D2
    PC12     ------> SDMMC1_CK
    PD2      ------> SDMMC1_CMD
    PC9      ------> SDMMC1_D1
    PC8      ------> SDMMC1_D0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_9
                          |GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN SDMMC1_MspInit 1 */

  /* USER CODE END SDMMC1_MspInit 1 */
  }

}

/**
* @brief SD MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hsd: SD handle pointer
* @retval None
*/
void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd)
{
  if(hsd->Instance==SDMMC1)
  {
  /* USER CODE BEGIN SDMMC1_MspDeInit 0 */

  /* USER CODE END SDMMC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDMMC1_CLK_DISABLE();

    /**SDMMC1 GPIO Configuration
    PC11     ------> SDMMC1_D3
    PC10     ------> SDMMC1_D2
    PC12     ------> SDMMC1_CK
    PD2     ------> SDMMC1_CMD
    PC9     ------> SDMMC1_D1
    PC8     ------> SDMMC1_D0
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_9
                          |GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  /* USER CODE BEGIN SDMMC1_MspDeInit 1 */

  /* USER CODE END SDMMC1_MspDeInit 1 */
  }

}

/***************************************************************************************************
 * Function Name: WiFiTask
 ***************************************************************************************************
 * Summary: This task initializes the Wi-Fi device, Wi-Fi transport, lwIP
 * network stack, and issues a scan for the available networks. A scan filter
 * is applied depending on the value of scan_filter_mode_select. After starting
 * the scan, it waits for the notification from the scan callback for completion
 * of the scan. It then waits for a delay specified by SCAN_DELAY_MS
 * before repeating the process.
 *
 * Parameters:
 *  void* arg: Task parameter defined during task creation (unused).
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void WiFi_Task(void* argument)
{
  cy_rslt_t       result = CY_RSLT_SUCCESS;
  cy_wcm_config_t wcm_config;
  wcm_config.interface = CY_WCM_INTERFACE_TYPE_STA;

  printf(" ******************* WiFi-Join-WPA3 app ******************* \r\n\r\n");

  if (stm32_cypal_wifi_sdio_init(&SDHandle) != CY_RSLT_SUCCESS)
  {
      printf("\r\n    ERROR: Init failed\r\n\r\n");
      Error_Handler();
  }

  /* wcm init */
  result = cy_wcm_init(&wcm_config);

  #if WIFI_CONNECT_ENABLE
  /* Wi-Fi connect */
  wifi_connect();

  #else /* Wi-Fi Scanning in loop */
  if (CY_RSLT_SUCCESS == result)
  {
      cy_wcm_scan_filter_t scan_filter =
      {
          .mode             = CY_WCM_SCAN_FILTER_TYPE_RSSI,
          .param.rssi_range = CY_WCM_SCAN_RSSI_FAIR
      };

      while (true)
      {
          /* Reset the count everytime before scanning */
          scan_data.result_count = 0;
          memset(last_bssid, 0, sizeof(cy_wcm_mac_t));

          /* Print out the scan results*/
          PRINT_SCAN_TEMPLATE();

          /* Start the scan */
          result =  cy_wcm_start_scan(scan_result_callback, &scan_data, &scan_filter);
          if (CY_RSLT_SUCCESS == result)
          {
              xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
          }
          /* Add Delay before starting the scan again */
          vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
      }
    } else
    {
      printf("Failed to initialize Wi-Fi\r\n");
      //Error_Handler();

      while (true)
      {
          /* Add Delay before starting the scan again */
          vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
      }
    }
  #endif /* WIFI_CONNECT_ENABLE */

}

/***************************************************************************************************
 * Function Name: scan_check_bssid_in_list
 ***************************************************************************************************
 * Summary: This Function removes duplicate entries in scan results.
 *
 * Parameters:
 *  cy_wcm_scan_result_t *result_ptr: Pointer to the scan result
 *
 * Return:
 *  bool
 *
 **************************************************************************************************/
bool scan_check_bssid_in_list(cy_wcm_scan_result_t* result_ptr)
{
    bool present = false;
    if (memcmp(result_ptr->BSSID, last_bssid, sizeof(cy_wcm_mac_t)) == 0)
    {
        /* Already existing BSSID, ignore the result */
        present = true;
        return present;
    }
    return present;
}

/***************************************************************************************************
 * Function Name: scan_result_callback
 ***************************************************************************************************
 * Summary: The callback function which accumulates the scan results. After
 * completing the scan, it sends a task notification to scan_task.
 *
 * Parameters:
 *  cy_wcm_scan_result_t *result_ptr: Pointer to the scan result
 *  void *user_data: User data.
 *  cy_wcm_scan_status_t status: Status of scan completion.
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void scan_result_callback(cy_wcm_scan_result_t* result_ptr, void* user_data,
                          cy_wcm_scan_status_t status)
{
    wcm_scan_data_t* scan_data = (wcm_scan_data_t*)user_data;

    if (scan_data != NULL)
    {
        if (status == CY_WCM_SCAN_COMPLETE)
        {
            xTaskNotify(WiFi_TaskHandle, 0, eNoAction);
        }
        else
        {
            if ((result_ptr != NULL) && (status == CY_WCM_SCAN_INCOMPLETE))
            {
                if ((strlen((const char*)result_ptr->SSID) != 0) &&
                    (scan_check_bssid_in_list(result_ptr) == false))
                {
                    scan_data->result_count++;
                    /* Copy BSSID to last bssid which will be used in scan_check_bssid_in_list to
                       avoid
                     * repeating BSSID in scan results
                     */
                    memcpy(&last_bssid, &result_ptr->BSSID[0], sizeof(last_bssid));
                    printf(" %2ld   %-32s     %4d     %3d      %02X:%02X:%02X:%02X:%02X:%02X"
                           "         %-15s\r\n",
                           scan_data->result_count, result_ptr->SSID,
                           result_ptr->signal_strength, result_ptr->channel,
                           result_ptr->BSSID[0], result_ptr->BSSID[1],
                           result_ptr->BSSID[2], result_ptr->BSSID[3],
                           result_ptr->BSSID[4], result_ptr->BSSID[5],
                           security_to_str((whd_security_t)result_ptr->security));
                }
            }
        }
    }
}

/***************************************************************************************************
 * Function Name: security_to_str
 ***************************************************************************************************
 * Summary: This Function returns string for each of WHD security types which
 * will be used in scan results.
 * Parameters:
 *  whd_security_t security: WHD security type enum
 *
 * Return:
 *  char *
 *
 **************************************************************************************************/
const char* security_to_str(whd_security_t security)
{
    switch (security)
    {
        case WHD_SECURITY_OPEN:
            return "OPEN";

        case WHD_SECURITY_WEP_PSK:
            return "WEP_PSK";

        case WHD_SECURITY_WEP_SHARED:
            return "WEP_SHARED";

        case WHD_SECURITY_WPA_TKIP_PSK:
            return "WPA_TKIP_PSK";

        case WHD_SECURITY_WPA_AES_PSK:
            return "WPA_AES_PSK";

        case WHD_SECURITY_WPA_MIXED_PSK:
            return "WPA_MIXED_PSK";

        case WHD_SECURITY_WPA2_AES_PSK:
            return "WPA2_AES_PSK";

        case WHD_SECURITY_WPA2_TKIP_PSK:
            return "WPA2_TKIP_PSK";

        case WHD_SECURITY_WPA2_MIXED_PSK:
            return "WPA2_MIXED_PSK";

        case WHD_SECURITY_WPA2_FBT_PSK:
            return "WPA2_FBT_PSK";

        case WHD_SECURITY_WPA3_SAE:
            return "WPA3_SAE";

        case WHD_SECURITY_WPA3_WPA2_PSK:
            return "WPA3_WPA2_PSK";

        case WHD_SECURITY_WPA_TKIP_ENT:
            return "WPA_TKIP_ENTERPRISE";

        case WHD_SECURITY_WPA_AES_ENT:
            return "WPA_AES_ENTERPRISE";

        case WHD_SECURITY_WPA_MIXED_ENT:
            return "WPA_MIXED_ENTERPRISE";

        case WHD_SECURITY_WPA2_TKIP_ENT:
            return "WPA2_TKIP_ENTERPRISE";

        case WHD_SECURITY_WPA2_AES_ENT:
            return "WPA2_AES_ENTERPRISE";

        case WHD_SECURITY_WPA2_MIXED_ENT:
            return "WPA2_MIXED_ENTERPRISE";

        case WHD_SECURITY_WPA2_FBT_ENT:
            return "WPA2_FBT_ENTERPRISE";

        case WHD_SECURITY_IBSS_OPEN:
            return "IBSS_OPEN";

        case WHD_SECURITY_WPS_SECURE:
            return "WPS_SECURE";

        case WHD_SECURITY_FORCE_32_BIT:
        case WHD_SECURITY_UNKNOWN:
        default:
            return "UNKNOWN_SECURITY";
    }
}


/***************************************************************************************************
 * print_ip4
 **************************************************************************************************/
static void print_ip4(uint32_t ip, char* str)
{
    unsigned char bytes[4];
    bytes[0] = ip & 0xFF;
    bytes[1] = (ip >> 8) & 0xFF;
    bytes[2] = (ip >> 16) & 0xFF;
    bytes[3] = (ip >> 24) & 0xFF;
    printf("%s addr = %d.%d.%d.%d\r\n", str, bytes[0], bytes[1], bytes[2], bytes[3]);
}


/***************************************************************************************************
 * Function Name: wifi_connect
 ***************************************************************************************************
 * Summary: This function executes a connect to the AP. The maximum number of
 * times it attempts to connect to the AP is specified by MAX_RETRY_COUNT. Then
 * ping to Gateway IP address.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void wifi_connect()
{
    cy_rslt_t                   result = CY_RSLT_SUCCESS;
    cy_wcm_connect_params_t     connect_param;
    cy_wcm_ip_address_t         ip_address;
    cy_wcm_ip_address_t         gateway_addr;
    uint32_t                    elapsed_time_ms;
    cy_wcm_associated_ap_info_t ap_info;

    memset(&connect_param, 0, sizeof(cy_wcm_connect_params_t));
    memset(&ip_address, 0, sizeof(cy_wcm_ip_address_t));
    memcpy(connect_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(connect_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
    connect_param.ap_credentials.security = WIFI_SECURITY;

    while (true)
    {
        printf("\r\n***** Connecting to '%s' AP ***** \r\n", connect_param.ap_credentials.SSID);

        /* Attempt to connect to Wi-Fi until a connection is made or
         * MAX_WIFI_RETRY_COUNT attempts have been made.
         */
        for (uint32_t conn_retries = 0; conn_retries < MAX_WIFI_RETRY_COUNT; conn_retries++)
        {
            result = cy_wcm_connect_ap(&connect_param, &ip_address);
            if (result == CY_RSLT_SUCCESS)
            {
                printf("Successfully connected to Wi-Fi network '%s'.\r\n",
                       connect_param.ap_credentials.SSID);
                break;
            }
            printf("Connection to Wi-Fi network failed with error code %d."
                   "Retrying in %d ms...\r\n", (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC);
            vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
        }
        if (result != CY_RSLT_SUCCESS)
        {
            printf("Wi-FI connection failed even with multiple retries \r\n");
            return;
        }
        /* Get RSSI */
        result = cy_wcm_get_associated_ap_info(&ap_info);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_wcm_get_associated_ap_info failed with result %ld \r\n", result);
            return;
        }
        printf("RSSI : %d \r\n", ap_info.signal_strength);

        /* Get IPv4 address */
        result = cy_wcm_get_ip_addr(CY_WCM_INTERFACE_TYPE_STA, &ip_address);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_wcm_get_ip_addr failed with result %ld \r\n", result);
            return;
        }
        print_ip4(ip_address.ip.v4, "IPV4");

        /* Get gateway address */
        result = cy_wcm_get_gateway_ip_address(CY_WCM_INTERFACE_TYPE_STA, &gateway_addr);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_wcm_get_gateway_ip_address failed with result %ld \r\n", result);
            return;
        }

        print_ip4(gateway_addr.ip.v4, "Pinging to gateway IPV4");
        vTaskDelay(2000);

        /* Send PING request with 3000ms ping timeout */
        result = cy_wcm_ping(CY_WCM_INTERFACE_TYPE_STA, &gateway_addr, 3000, &elapsed_time_ms);
        if (result == CY_RSLT_SUCCESS)
        {
            printf("Ping was successful time elapsed = %lu ms\r\n", elapsed_time_ms);
        }
        else
        {
            printf("Ping failed !! Module %lx Code %lx\r\n", CY_RSLT_GET_MODULE(
                       result), CY_RSLT_GET_CODE(result));
        }
        printf("Disconnecting ... \r\n");
        result = cy_wcm_disconnect_ap();
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_wcm_disconnect_ap failed with result %ld \r\n", result);
        }
        /* Delay before Connecting back to AP */
        vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
    }
}

/* USER CODE END 0 */

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/***************************************************************************************************
 * Function Name: SDMMC1_IRQHandler
 ***************************************************************************************************
 * Summary: This Function handles SDMMC Interrupt and calls STM hal layer
 * function which handles interrupt masking and calling sdio event callback
 * function.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void SDMMC1_IRQHandler(void)
{
    stm32_cyhal_sdio_irq_handler();
}

