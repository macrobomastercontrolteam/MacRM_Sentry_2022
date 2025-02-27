/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOG
#define LED5_Pin GPIO_PIN_3
#define LED5_GPIO_Port GPIOG
#define LED6_Pin GPIO_PIN_4
#define LED6_GPIO_Port GPIOG

// led row output define begin
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
// led row output define end

#define ECHO1_Pin GPIO_PIN_4
#define ECHO1_GPIO_Port GPIOA
#define TRIGGER1_Pin GPIO_PIN_5
#define TRIGGER1_GPIO_Port GPIOA

#define ECHO2_Pin GPIO_PIN_1
#define ECHO2_GPIO_Port GPIOC
#define TRIGGER2_Pin GPIO_PIN_5
#define TRIGGER2_GPIO_Port GPIOC

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
