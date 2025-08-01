/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (JustFloat ˫ͨ�����Ҳ�����)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"       
#include "usart.h"      
#include "gpio.h"      


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
float x = 0.0f;          
const float dt = 0.01f;  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  
/* USER CODE BEGIN PFP */
// USART1 ���ֽڷ��ͺ���
void USART1_SendByte(uint8_t data);
// JustFloat Э�鷢�ͺ���
void JustFloat_Send(float *data, uint8_t channel_count);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  USART1 ���ֽڷ��ͣ���ѯ��ʽ��ȷ�����ݷ�����ɣ�
 * @param  data: Ҫ���͵��ֽ�
 * @retval None
 */
void USART1_SendByte(uint8_t data) {
    // �ȴ����ͼĴ�����
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
    // д�����ݼĴ���
    huart1.Instance->DR = data;
}

/**
 */
void JustFloat_Send(float *data, uint8_t channel_count) {
    // 1. ��������ͨ����float���ݣ�ÿ��float���Ϊ4�ֽڣ�
    for (uint8_t ch = 0; ch < channel_count; ch++) {
        uint8_t *byte_ptr = (uint8_t *)&data[ch]; 
        for (uint8_t i = 0; i < 4; i++) {         
            USART1_SendByte(byte_ptr[i]);
        }
    }

    // 2. ���� JustFloat Э��Ҫ��Ĺ̶�֡β
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    for (uint8_t i = 0; i < 4; i++) {
        USART1_SendByte(tail[i]);
    }
}
/* USER CODE END 0 */

/**
 */
int main(void)
{
  HAL_Init();                     
  SystemClock_Config();           
  MX_GPIO_Init();                 
  MX_USART1_UART_Init();          

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  while (1)
  {
    // ����˫ͨ�����Ҳ�����
    float ch_data[2] = {
        sin(x),       // ͨ��0�����Ҳ�1
        sin(2 * x)    // ͨ��1�����Ҳ�2
    };


    JustFloat_Send(ch_data, 2);


    x += dt;


    HAL_Delay(10);  

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief  
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

 
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

/**
 * @brief  
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq(); 
  while (1)       
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
