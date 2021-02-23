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
#include "stm32f4xx_hal.h"

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
#define ECHO2_Pin GPIO_PIN_9
#define ECHO2_GPIO_Port GPIOI
#define ECHO1_Pin GPIO_PIN_0
#define ECHO1_GPIO_Port GPIOF
#define TRIGGER1_Pin GPIO_PIN_1
#define TRIGGER1_GPIO_Port GPIOF
#define TRIGGER2_Pin GPIO_PIN_10
#define TRIGGER2_GPIO_Port GPIOF

#define LED8_Pin_o GPIO_PIN_8
#define LED8_GPIO_Port_o GPIOG
#define LED7_Pin_o GPIO_PIN_7
#define LED7_GPIO_Port_o GPIOG
#define LED6_Pin_o GPIO_PIN_6
#define LED6_GPIO_Port_o GPIOG
#define LED5_Pin_o GPIO_PIN_5
#define LED5_GPIO_Port_o GPIOG
#define LED4_Pin_o GPIO_PIN_4
#define LED4_GPIO_Port_o GPIOG
#define LED3_Pin_o GPIO_PIN_3
#define LED3_GPIO_Port_o GPIOG
#define LED2_Pin_o GPIO_PIN_2
#define LED2_GPIO_Port_o GPIOG
#define LED1_Pin_o GPIO_PIN_1
#define LED1_GPIO_Port_o GPIOG
#define LED_RED_Pin_o GPIO_PIN_11
#define LED_RED_GPIO_Port_o GPIOE
#define LED_GREEN_Pin_o GPIO_PIN_14
#define LED_GREED_GPIO_Port_o GPIOF

#define TEST_Pin GPIO_PIN_5
#define TEST_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
