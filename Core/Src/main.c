/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t bufferx;
uint8_t i;
uint32_t ad1, ad2;
uint32_t ADC_Value[100];
uint32_t pdx[16];
/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Value, 100);
  //  HAL_ADC_Start(&hadc1);//启动ADC装换
  //  HAL_ADC_PollForConversion(&hadc1, 10);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //A03  底盘车电机  一个触头
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //A04 底盘车电机 另一个触头
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //A05 接地刀电机 一个触头
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); //A06 接地刀电机 另一个触头
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //A07 储能电机
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //A09 分闸线圈
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //A10 合闸线圈
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //A11 继电器合闸
  pdx[0] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);         //A08 储能电机辅助开关  需要和 弹簧未储能按钮 配合
  pdx[1] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);         //A12 手动分闸按钮  A21 遥控分闸出口
  pdx[2] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);         //A13 保护分闸按钮
  pdx[3] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);         //A14合闸按钮
  pdx[4] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);         //A23底盘车试验位置
  pdx[5] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);         //A24底盘车工作位置
  pdx[6] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6);         //A25底盘车遥进按钮手动
  pdx[7] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);         //A26底盘车遥出按钮手动
  pdx[8] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);         //A27弹簧未储能按钮
  pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);         //A29遥控允许按钮
  pdx[10] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);       //A30接地刀合闸输入
  pdx[11] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);       //A31接地刀分闸输入
  pdx[12] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);       //A32接地刀机构合闸位置（信号快）
  pdx[13] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);       //A33接地刀机构分闸位置（信号快）
  pdx[14] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);       //A34接地刀合闸位置   需要和A32 配合
  pdx[15] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);       //A35接地刀分闸位置

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //	  HAL_ADC_Start(&hadc1);//启动ADC装换
    //	  HAL_ADC_PollForConversion(&hadc1, 10);//等待转换完成，第二个参数表示超时时间�????
    //
    //	  if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)){
    //	  bufferx = HAL_ADC_GetValue(&hadc1);//读取ADC转换数据，数据为12�????
    //    printf("[\tmain]info:v=%.1fmv\r\n",AD_Value );//打印日志
    //	  }
    //	  HAL_Delay(20);
    //
    //	  for(i = 0,ad1 =0; i < 100;){
    //	  ad1 += ADC_Value[i++];
    //	  }
    //	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 100);
    //    printf(" %4.4f \n", ad1 );

    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

#ifdef USE_FULL_ASSERT
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
