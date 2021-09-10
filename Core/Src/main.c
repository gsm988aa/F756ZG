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
#include "tim.h"
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
uint32_t pbx[8];
uint32_t pdx[16];
uint8_t statesum;
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC_Value, 100);
  //  HAL_ADC_Start(&hadc1);//启动ADC装换
  //  HAL_ADC_PollForConversion(&hadc1, 10);
  //PB0    PB1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //A03  底盘车电机  一个触头
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //A04 底盘车电机 另一个触头
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  //PB8    PB9
  // HAL_TIM_PWM_Stop();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //A05 接地刀电机 一个触头
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //A06 接地刀电机 另一个触头
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //A07 储能电机
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //A09 分闸线圈
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //A10 合闸线圈
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //A11 继电器合闸

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(5);
    statesum = 0;
    pdx[0] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);   //A08 储能电机辅助按钮 要和 A27弹簧未储能按钮配合       重要4   [按钮  外部中断]
    pdx[1] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);   //A12 手动分闸按钮  A21 遥控分闸出口                   重要1    [按钮  外部中断]
    pdx[2] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);   //A13 保护分闸按钮                                    重要2    [按钮  外部中断]
    pdx[3] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);   //A14合闸按钮                                         重要5    [按钮  外部中断]
    pdx[4] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);   //A23底盘车试验位置                 {状态}
    pdx[5] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);   //A24底盘车工作位置                 {状态}
    pdx[7] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);   //A26底盘车遥出按钮手动                                重要7    [按钮  外部中断]
    pdx[6] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6);   //A25底盘车遥进按钮手动                                重要6    [按钮  外部中断]
    pdx[8] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);   //A27弹簧未储能                     {状态}
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);   //A29遥控允许                       {状态}
    pdx[10] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10); //A30接地刀合闸输入按钮                               重要8    [按钮  外部中断]
    pdx[11] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11); //A31接地刀分闸输入按钮                               重要3    [按钮  外部中断]
    pdx[12] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12); //A32接地刀机构合闸位置（信号快)      {状态}
    pdx[13] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13); //A33接地刀机构分闸位置（信号快)      {状态}
    pdx[14] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14); //A34接地刀合闸位置   要和A32 配合    {状态}
    pdx[15] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15); //A35接地刀分闸位置                  {状态}
    //statesum  = pdx[0] | pdx[1] | pdx[2] | pdx[3] | pdx[4] | pdx[5]| pdx[6]| pdx[7]| pdx[8] | pdx[9] |pdx[10] | pdx[11] | pdx[12] | pdx[13] | pdx[14] | pdx[15];
    //statesum  = pdx[4] | pdx[5] | pdx[8] | pdx[9] | pdx[12] | pdx[13] | pdx[14] | pdx[15];

    // 断路器合�?

    //	  HAL_ADC_Start(&hadc1);//启动ADC装换
    //	  HAL_ADC_PollForConversion(&hadc1, 10);//等待转换完成，第二个参数表示超时时间
    //	  if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)){
    //	  bufferx = HAL_ADC_GetValue(&hadc1);//读取ADC转换数据，数据为12�?
    //    printf("[\tmain]info:v=%.1fmv\r\n",AD_Value );//打印日志
    //	  }
    //	  HAL_Delay(20);
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

//PD0 A08 储能电机辅助按钮 要和 A27弹簧未储能按钮配合       重要4   [按钮  外部中断]
//PD1 A12 手动分闸按钮  A21 遥控分闸出口                   重要1    [按钮  外部中断]
//PD2 A13 保护分闸按钮                                    重要2    [按钮  外部中断]
//PD3 A14 合闸按钮                                         重要5    [按钮  外部中断]
//PD4 A23 底盘车试验位置                 {状态}
//PD5 A24 底盘车工作位置                 {状态}
//PD6 A26 底盘车遥出按钮手动                                重要7    [按钮  外部中断]
//PD7 A25 底盘车遥进按钮手动                                重要6    [按钮  外部中断]
//PD8 A27 弹簧未储能                     {状态}
//PD9 A29 遥控允许                       {状态}
//PD10 A30 接地刀合闸输入按钮                               重要8    [按钮  外部中断]
//PD11 A31 接地刀分闸输入按钮                               重要3    [按钮  外部中断]
//PD12 A32 接地刀机构合闸位置（信号快)      {状态}
//PD13 A33 接地刀机构分闸位置（信号快)      {状态}
//PD14 A34 接地刀合闸位置   要和A32 配合    {状态}
//PD15 A35 接地刀分闸位置                  {状态}
// HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PB0    A03  底盘车电机  一个触头
// __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
// HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PB1    A04 底盘车电机 另一个触头
// __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

// // HAL_TIM_PWM_Stop();
// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //PB8    A05 接地刀电机 一个触头
// __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //PB9    A06 接地刀电机 另一个触头
// __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //A07 储能电机
// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //A09 分闸线圈
// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //A10 合闸线圈
// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //A11 继电器合闸
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  //PD0 A08 储能电机辅助按钮 要和 A27弹簧未储能按钮配合       重要4   [按钮  外部中断]
  //已捕获PD0 弹簧未储能, 则弹簧储能  需要 捕捉弹簧现在未储能状态PD8 且 远控允许PD9
  if (GPIO_Pin == GPIO_PIN_0)
  {
    pdx[8] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
    if (pdx[8] && (!pdx[9]))
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(4000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_Delay(5000);
    }
    if (pdx[8] && pdx[9])
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(4000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_Delay(5000);
    }
  }
  //PD1 A12 手动分闸按钮  A21 遥控分闸出口                   重要1    [按钮  外部中断]
  //已捕获PD1 手动分闸,直接分闸 且 !远控允许PD9必须为0 且 底盘车工作位置PD5
  //由于紧急分闸 不考虑以下情况:  且 PD13 接地刀机构分闸位置（信号快)  且 PD15 接地刀分闸位置 且 PD3 合闸按钮必须为0
  if (GPIO_Pin == GPIO_PIN_1)
  {
    pdx[5] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
    if (pdx[5] && (!pdx[9]))
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_Delay(5000);
    }
    if (pdx[5] && pdx[9])
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_Delay(5000);
    }
  }
  //PD2 A13 保护分闸按钮                                    重要2    [按钮  外部中断]
  //已捕获PD1 手动分闸,直接分闸 且 远控允许PD9 且 底盘车工作位置PD5
  //由于非紧急分闸 考虑以下情况:  且 PD13 接地刀机构分闸位置（信号快)  且 PD15 接地刀分闸位置 且 PD3 !合闸按钮必须为0
  if (GPIO_Pin == GPIO_PIN_2)
  {
    pdx[5] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
    if (pdx[5] && pdx[9])
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_Delay(5000);
    }
    if (pdx[5] && (!pdx[9]))
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_Delay(5000);
    }
  }

  //PD3 A14合闸按钮                                         重要5    [按钮  外部中断]
  //已捕获PD3 合闸,直接合闸 且 远控允许PD9 且 底盘车工作位置PD5 或 底盘车在试验位置PD4
  //由于非紧急合闸 考虑以下情况:      且 PD15 接地刀分闸位置 且     且 PD2 !保护分闸按钮必须为0 且 PD8!弹簧未储能必为 0
  if (GPIO_Pin == GPIO_PIN_3)
  {
    pdx[4] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
    pdx[5] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
    pdx[15] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);
    pdx[8] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
    if ((pdx[5] || pdx[4]) && pdx[15] && (!pdx[8]) && (!pdx[9]))
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //A10 合闸线圈
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //A10 合闸线圈
      HAL_Delay(5000);
    }
    if ((pdx[5] || pdx[4]) && pdx[15] && (!pdx[8]) && pdx[9])
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); //A10 合闸线圈
      HAL_Delay(1000);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //A10 合闸线圈
      HAL_Delay(5000);
    }
  }
  //PD6 A26底盘车遥出按钮手动                                重要7    [按钮  外部中断]
  //PB7继电器总开关必须使能
  //已捕获PD6 底盘车手动推出  电机反转PB0 = 0 , PB1 = 1 且 !远控允许PD9必为0  且 底盘车工作位置PD5 且 PB7继电器总开关必须使能为1 且  !PB6 断路器必须分闸 且 PD13 接地刀机构分闸位置（信号快)  且 PD15 接地刀分闸位置
  if (GPIO_Pin == GPIO_PIN_6)
  {
    pdx[4] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
    pdx[5] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
    pdx[13] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);

    pbx[7] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    pbx[6] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
    if (pdx[5] && pbx[7] && (!pbx[6]) && pdx[13] && (!pdx[9]))
    {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PB0    A03  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PB1    A04 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
    }
    if ((!pdx[5]) && (!pdx[4]) && pbx[7] && (!pbx[6]) && pdx[13] && (!pdx[9]))
    {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PB0    A03  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PB1    A04 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
    }
    if (pdx[5] && pbx[7] && (!pbx[6]) && pdx[13] && pdx[9])
    {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PB0    A03  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PB1    A04 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
    }
    if ((!pdx[5]) && (!pdx[4]) && pbx[7] && (!pbx[6]) && pdx[13] && pdx[9])
    {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PB0    A03  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PB1    A04 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
    }
  }
  //PD7 A25底盘车遥进按钮手动                                重要6    [按钮  外部中断]
  //已捕获PD7 底盘车手动驶入
  if (GPIO_Pin == GPIO_PIN_7)
  {
    pdx[4] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
    pdx[5] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5);
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);

    pdx[15] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);

    pbx[7] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    pbx[6] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
    if (pdx[5] && pbx[7] && (!pbx[6]) && pdx[15] && (!pdx[9]))
    {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PB0    A03  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PB1    A04 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    }
    if (pdx[5] && pbx[7] && (!pbx[6]) && pdx[15] && pdx[9])
    {
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PB0    A03  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PB1    A04 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    }
  }
  //PD10 A30接地刀合闸输入按钮                               重要8    [按钮  外部中断]
  //PB7继电器总开关必须使能
  //已捕获PD10 接地刀手动合闸  接地刀电机反转PB8 = 1 , PB9 = 0 且 !远控允许PD9必为0  且 底盘车试验位置PD4 且 PB7继电器总开关必须使能为1 且  !PB6 断路器必须分闸 且 PD13 接地刀机构分闸位置（信号快)  且 PD15 接地刀分闸位置
  if (GPIO_Pin == GPIO_PIN_10)
  {
    pdx[4] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
    
    pdx[15] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);

    pbx[7] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    pbx[6] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
    if (pdx[4]  && pbx[7] && (!pbx[6]) && pdx[15] && (!pdx[9]))
    {
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //PB8    A05  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //PB9    A06 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
    }
    if (((!pdx[4])||(!pdx[5]))  && pbx[7] && (!pbx[6]) && pdx[15] && (!pdx[9]))
    {
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //PB8    A05  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //PB9    A06 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
    }
    if (pdx[4]  && pbx[7] && (!pbx[6]) && pdx[15] && pdx[9])
    {
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //PB8    A05  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //PB9    A06 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
    }
    if (((!pdx[4])||(!pdx[5]))  && pbx[7] && (!pbx[6]) && pdx[15] && pdx[9])
    {
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //PB8    A05  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //PB9    A06 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
    }
  }
  //PD11 A31接地刀分闸输入按钮                               重要3    [按钮  外部中断]
  if (GPIO_Pin == GPIO_PIN_11)
  {
    //PB7继电器总开关必须使能
    //已捕获PD10 接地刀手动合闸  接地刀电机反转PB8 = 1 , PB9 = 0 且 !远控允许PD9必为0  且 底盘车试验位置PD4 且 PB7继电器总开关必须使能为1 且  !PB6 断路器必须分闸 且 PD12 接地刀机构合闸位置（信号快)  且 PD14 接地刀合闸位置
    pdx[4] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
    pdx[9] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
   
    pdx[14] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14);

    pbx[7] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    pbx[6] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);

    if (pdx[4] && pbx[7] && (!pbx[6]) && pdx[14] && (!pdx[9]))
    {
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //PB8    A05  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //PB9    A06 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    }
    if (pdx[4] && pbx[7] && (!pbx[6]) && pdx[14] && pdx[9])
    {
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //PB8    A05  底盘车电机  一个触头
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //PB9    A06 底盘车电机 另一个触头
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
                                                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
      // PB0   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      // PB1   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      HAL_Delay(10);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_Delay(5000);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
      HAL_Delay(100);
      //防抱死
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_Delay(3000);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    }
  }
}
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}
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
