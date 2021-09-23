/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define vector_Pin GPIO_PIN_0
#define vector_GPIO_Port GPIOA
#define car1_Pin GPIO_PIN_1
#define car1_GPIO_Port GPIOB
#define car2_Pin GPIO_PIN_2
#define car2_GPIO_Port GPIOB
#define knife1_Pin GPIO_PIN_10
#define knife1_GPIO_Port GPIOB
#define knife2_Pin GPIO_PIN_11
#define knife2_GPIO_Port GPIOB
#define chuneng_Pin GPIO_PIN_4
#define chuneng_GPIO_Port GPIOB
#define fenzha_Pin GPIO_PIN_5
#define fenzha_GPIO_Port GPIOB
#define hezha_Pin GPIO_PIN_6
#define hezha_GPIO_Port GPIOB
#define backup_Pin GPIO_PIN_7
#define backup_GPIO_Port GPIOB
#define onekey_1_Pin GPIO_PIN_8
#define onekey_1_GPIO_Port GPIOB
#define onekey_2_Pin GPIO_PIN_9
#define onekey_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
