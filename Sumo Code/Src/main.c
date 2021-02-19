/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "Remote_Control.h"
/* USER CODE END Includes */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define usDelay(uSec)                                                          \
    do                                                                         \
    {                                                                          \
        __HAL_TIM_SET_COUNTER(&htim4, 0); /* set the counter value a 0*/       \
        while (__HAL_TIM_GET_COUNTER(&htim4) < uSec)                           \
            ; /* wait for the counter to reach the us input in the parameter*/ \
    } while (0U)
#define ULTRASONIC_MEASURE_RIGHT()                                                              \
    do                                                                                          \
    {                                                                                           \
        /* START Ultrasonic measure routine */                                                  \
        numTicks = 0;                                                                           \
        HAL_GPIO_WritePin(TRIGGER2_GPIO_Port, TRIGGER2_Pin, GPIO_PIN_RESET);                    \
        usDelay(3);                                                                             \
        /*1. Output 10 usec TRIG (actually 14us)*/                                              \
        HAL_GPIO_WritePin(TRIGGER2_GPIO_Port, TRIGGER2_Pin, GPIO_PIN_SET);                      \
        usDelay(10);                                                                            \
        HAL_GPIO_WritePin(TRIGGER2_GPIO_Port, TRIGGER2_Pin, GPIO_PIN_RESET);                    \
        /*2. Wait for ECHO pin rising edge*/                                                    \
        while (HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == GPIO_PIN_RESET)                  \
            ;                                                                                   \
        /*3. Start measuring ECHO pulse width in usec*/                                         \
        while (HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == GPIO_PIN_SET && numTicks < 5850) \
        { /* within 1 meter range*/                                                             \
            numTicks++;                                                                         \
            usDelay(2);                                                                         \
        };                                                                                      \
        distance_right = (numTicks + 0.0f) * speedOfSound; /* centimeter*/                      \
    } while (0U)
#define ULTRASONIC_MEASURE_LEFT()                                                               \
    do                                                                                          \
    {                                                                                           \
        /* START Ultrasonic measure routine */                                                  \
        numTicks = 0;                                                                           \
        HAL_GPIO_WritePin(TRIGGER1_GPIO_Port, TRIGGER1_Pin, GPIO_PIN_RESET);                    \
        usDelay(3);                                                                             \
        /*1. Output 10 usec TRIG (actually 14us)*/                                              \
        HAL_GPIO_WritePin(TRIGGER1_GPIO_Port, TRIGGER1_Pin, GPIO_PIN_SET);                      \
        usDelay(10);                                                                            \
        HAL_GPIO_WritePin(TRIGGER1_GPIO_Port, TRIGGER1_Pin, GPIO_PIN_RESET);                    \
        /*2. Wait for ECHO pin rising edge*/                                                    \
        while (HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == GPIO_PIN_RESET)                  \
            ;                                                                                   \
        /*3. Start measuring ECHO pulse width in usec*/                                         \
        while (HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == GPIO_PIN_SET && numTicks < 5850) \
        { /* within 1 meter range*/                                                             \
            numTicks++;                                                                         \
            usDelay(2);                                                                         \
        };                                                                                      \
        distance_left = (numTicks + 0.0f) * speedOfSound; /* centimeter*/                       \
    } while (0U)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PID_TypeDef motor_pid[4];
int32_t set_spd = 0;
static int key_sta = 0;
int speed_step_sign = +1;

uint32_t counter = 0;
uint32_t counter1 = 0;
uint32_t counter2 = 0;
//uint16_t sonic = 1;
uint16_t TIM_COUNT[2];

/***** State Machine Variables *****/
uint8_t state = 0;

uint8_t state0_d = 0;
uint8_t state1_d = 0;
uint8_t state2_d = 0;
uint8_t state3_d = 0;
uint8_t state4_d = 0;
uint8_t state5_d = 0;
uint8_t state6_d = 0;
uint8_t state7_d = 0;
uint8_t state8_d = 0;
uint8_t state9_d = 0;

/***** Ultrasonic variables *****/
#define LEFT_SONIC_THRESHOLD 50.0f
#define RIGHT_SONIC_THRESHOLD 50.0f

uint8_t left_detect = 0;
uint8_t right_detect = 0;

const float speedOfSound = 0.0343 / 2;
float distance_left = LEFT_SONIC_THRESHOLD + 1.0f; // in cm
float distance_right = RIGHT_SONIC_THRESHOLD + 1.0f;

/****** Motor Variables ******/
//left wheel - Motor[0] right wheel - Motor[1]
int speed_sign = +1;
//int right_speed_sign = -1;
int32_t wheel_speed_super_slow = 1600;
int32_t wheel_speed_slow = 2000;
int32_t wheel_speed_fast = 8000;
int32_t wheel_speed_medium = 4000;
int32_t wheel_speed_turn = 6000;

uint32_t straight_tick = 2000;
uint32_t fast_90_tick = 250;
uint32_t medium_90_tick = 360;
uint32_t slow_90_tick = 1000;

#define SpeedStep 500

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ledrowoutput(float distance);
void wiggle(uint8_t wtime);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Key_Scan()
{
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
        if (key_sta == 0)
        {
            key_sta = 1;
            set_spd += SpeedStep * speed_step_sign;
            if (set_spd > 8000)
            {
                speed_step_sign = -1;
            }
            if (set_spd <= 0)
            {
                set_spd = 0;
                speed_step_sign = 1;
            }
        }
    }
    else
    {
        key_sta = 0;
    }
}
/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */
    uint32_t numTicks = 0;
    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim4);                          // TIM4 is for usDelay
    my_can_filter_init_recv_all(&hcan1);                 //配置CAN过滤器
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);               //启动CAN接收中断
    HAL_UART_Receive_IT_IDLE(&huart1, UART_Buffer, 100); //启动串口接收

    HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *)TIM_COUNT, 2);

    /*< 初始化PID参数 >*/
    for (int i = 0; i < 4; i++)
    {
        pid_init(&motor_pid[i]);
        motor_pid[i].f_param_init(&motor_pid[i], PID_Speed, 16384, 5000, 10, 0, 8000, 0, 1.5, 0.1, 0);
    }

    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (counter1 % 3 == 0)
        { /********* Ultrasonic detect every 30 ms ********/
            ;
        }
        if (state == 0)
        {
            if (state0_d == 0)
            {
                counter = HAL_GetTick();
                state0_d = 1;
            }
            else if (state0_d == 1)
            {
                ;
            }
        }
        else if (state == 1)
        {
            if (state1_d == 0)
            {
                counter = HAL_GetTick();
                state1_d = 1;
            }
            else if (state1_d == 1)
            {
                ;
            }
        }
        motor_pid[0].f_cal_pid(&motor_pid[0], moto_chassis[0].speed_rpm); //根据设定值进行PID计算。
        // only 1 motor is used to move left & right, other 3 motors outputs are left as 0 here because I don't know how to change the bsp_can file, LOL
        set_moto_current(&hcan1, motor_pid[0].output, 0, 0, 0); //将PID的计算结果通过CAN发送到电机
        HAL_Delay(10);                                          //PID控制频率100HZ
        counter1++;
    }
    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void wiggle(uint8_t wtime)
{
    ;
}

// led output, test function
void ledrowoutput(float distance)
{
    if (distance < 12.5f)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port_o, LED1_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port_o, LED2_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED3_GPIO_Port_o, LED3_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED4_GPIO_Port_o, LED4_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED5_GPIO_Port_o, LED5_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED6_GPIO_Port_o, LED6_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED7_GPIO_Port_o, LED7_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED8_GPIO_Port_o, LED8_Pin_o, GPIO_PIN_SET);
    }
    else if (distance < 25.0f)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port_o, LED1_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port_o, LED2_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port_o, LED3_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED4_GPIO_Port_o, LED4_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED5_GPIO_Port_o, LED5_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED6_GPIO_Port_o, LED6_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED7_GPIO_Port_o, LED7_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED8_GPIO_Port_o, LED8_Pin_o, GPIO_PIN_SET);
    }
    else if (distance < 37.5f)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port_o, LED1_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port_o, LED2_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port_o, LED3_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port_o, LED4_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED5_GPIO_Port_o, LED5_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED6_GPIO_Port_o, LED6_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED7_GPIO_Port_o, LED7_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED8_GPIO_Port_o, LED8_Pin_o, GPIO_PIN_SET);
    }
    else if (distance < 50.0f)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port_o, LED1_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port_o, LED2_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port_o, LED3_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port_o, LED4_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED5_GPIO_Port_o, LED5_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED6_GPIO_Port_o, LED6_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED7_GPIO_Port_o, LED7_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED8_GPIO_Port_o, LED8_Pin_o, GPIO_PIN_SET);
    }
    else if (distance < 62.5f)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port_o, LED1_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port_o, LED2_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port_o, LED3_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port_o, LED4_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED5_GPIO_Port_o, LED5_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port_o, LED6_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED7_GPIO_Port_o, LED7_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED8_GPIO_Port_o, LED8_Pin_o, GPIO_PIN_SET);
    }
    else if (distance < 75.0f)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port_o, LED1_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port_o, LED2_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port_o, LED3_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port_o, LED4_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED5_GPIO_Port_o, LED5_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port_o, LED6_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED7_GPIO_Port_o, LED7_Pin_o, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED8_GPIO_Port_o, LED8_Pin_o, GPIO_PIN_SET);
    }
    else if (distance < 87.5f)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port_o, LED1_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port_o, LED2_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port_o, LED3_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port_o, LED4_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED5_GPIO_Port_o, LED5_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port_o, LED6_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED7_GPIO_Port_o, LED7_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED8_GPIO_Port_o, LED8_Pin_o, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port_o, LED1_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port_o, LED2_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED3_GPIO_Port_o, LED3_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port_o, LED4_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED5_GPIO_Port_o, LED5_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED6_GPIO_Port_o, LED6_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED7_GPIO_Port_o, LED7_Pin_o, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED8_GPIO_Port_o, LED8_Pin_o, GPIO_PIN_RESET);
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    //	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }

    if (htim->Instance == TIM1)
    {
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
