/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <applicfg.h>
#include <stdlib.h>
#include "canfestival.h"
// #include "config.h" // dardware下
#include "TestAll.h"
#include "data.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static UNS32 SDO_TIMEOUT = 1000; // 1秒超时
static volatile UNS8 SDO_COMPLETE = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// SDO回调函数
void SDOCallback(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS32 abortCode)
{
    if(abortCode == 0) {
        printf("SDO写入成功\n");
    } else {
        printf("SDO写入失败，错误码：0x%X\n", abortCode);
    }
    SDO_COMPLETE = 1;
}


// SDO写入函数
UNS8 SDO_Write(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8* data, UNS32 size)
{
    UNS8 err;
    UNS32 startTime;
    
    // 重置状态
    SDO_COMPLETE = 0;
    
    // 发送SDO请求
    err = writeNetworkDictCallBack(
        d,                  // CO_Data结构体
        nodeId,            // 目标节点ID
        index,             // 目标索引
        subIndex,          // 子索引
        size,              // 数据长度
        uint8,             // 数据类型
        data,              // 数据指针
        SDOCallback,       // 回调函数
        0                  // 字节序
    );
    
    if(err) return err;
    
    // 等待完成或超时
    startTime = HAL_GetTick();
    while(!SDO_COMPLETE) {
        if(HAL_GetTick() - startTime > SDO_TIMEOUT) {
            printf("SDO写入超时\n");
            return 0xFF;
        }
        HAL_Delay(1);
    }
    
    return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("hello world\n");
  Configure_Filter();
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_TIM_Base_Start_IT(&htim3);

	unsigned char nodeID = 0x00;                   //节点ID
 setNodeId(&TestAll_Data, nodeID);
 setState(&TestAll_Data, Initialisation);				//节点初始化
 setState(&TestAll_Data, Operational);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    printf("master test\n");

//测试SDO

    // 要写入的数据
//    INTEGER8 data1 = 0x12;  // 因为目标变量test是INTEGER16类型

    // // 发送SDO写入请求
    // UNS8 res = writeNetworkDict(
    //     &TestAll_Data,        // CO_Data结构体
    //     0x02,                 // 目标节点ID (从SdoTest_obj1280_Node_ID_of_the_SDO_Server)
    //     0x2001,               // 目标索引 (test变量的索引)
    //     0x00,                 // 子索引
    //     sizeof(INTEGER8),    // 数据长度
    //     int8,                // 数据类型 (与test变量类型匹配)
    //     &data1,                // 数据指针
    //     0                     // 不使用块传输
    // );
    // if(res == 0) {
    //   printf("SDO写入请求发送成功\r\n");
    // } else {
    //   printf("SDO写入请求发送失败: %d\r\n", res); 
    // }
    // INTEGER8 data1 = 0x12;
    // writeNetworkDictCallBack(
    //     &TestAll_Data,        // CO_Data结构体
    //     0x02,                 // 目标节点ID
    //     0x2001,              // 目标索引
    //     0x00,                // 子索引
    //     sizeof(INTEGER8),    // 数据长度
    //     int8,               // 数据类型
    //     &data1,              // 数据指针
    //     SDOWriteCallback,    // 回调函数
    //     0                    // 字节序
    // );

    INTEGER8 data1 = 0x12;
    UNS8 res = SDO_Write(&TestAll_Data, 0x02, 0x2001, 0x00, (UNS8*)&data1, sizeof(INTEGER8));
    if(res == 0) {
      printf("SDO写入请求发送成功\r\n");
    } else {
      printf("SDO写入请求发送失败: %d\r\n", res); 
    }

// 调试信息
		data[0]++;
    // 添加调试信息
    printf("Node state: %d\n", TestAll_Data.nodeState);
    printf("PDO state: %d\n", TestAll_Data.CurrentCommunicationState.csPDO);
    // 打印更多调试信息
    printf("PDO transmission type: %d\n", 
            TestAll_Data.CurrentCommunicationState.csPDO);
    printf("PDO event timer: %d\n", 
            TestAll_Data.CurrentCommunicationState.csPDO);
    printf("PDO inhibit time: %d\n", 
            TestAll_Data.CurrentCommunicationState.csPDO);

    HAL_Delay(1000);
	
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
