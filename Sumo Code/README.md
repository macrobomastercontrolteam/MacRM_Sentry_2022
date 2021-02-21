# Sentry Ultrasonic Sensor Module
### by MacRobomaster 2020 Control Team

## System Notes
- This code is used on development board A for test reasons
- 上下两个C板：上面C板控制左右运动马达，下面控制云台+拨弹轮  
- 上下拨弹轮synchronize 
- 云台与Chassis用GM6020连接
- 左右移动暂定一个马达M3508

## General Algorithm
1. memorize the previous location (in less frequency)  
2. if both sensors offline, use the last location  
3. If both sensors outputs too short distances, determine how to move using the previous location and measure by the sensors again  
4. if one of the sensors doesn't work, determine using the other sensor and the previous location  

## Pin setup
- Test
    - Echo & Trig 1 (left):   PF0 & PF1  
    - Echo & Trig 2 (right):  PI9 & PF10  
- Modified Sumo Code
    - Echo & Trig 1 (left):   PA4 & PFA5
    - Echo & Trig 2 (right):  PC1 & PC5  

- LED_o 1-8: PG1-8
- LED_RED_o & LED_GREEN_o: PE11 & PF14
- KEY: reset button (PB2)

## Code Setup
- usDelay
    - time (us) parameter must be within range of uint16_t (sugguestion: 1-65500)
    - used TIM4 (prescaler=freq-1 (in MHz), ARR=0xffff-1)
- wiggle() -> wiggle around a wiggling center point

## State Machine
- State 0: going left and wiggle
- State 1: going right and wiggle

## Parameters to be determined
- int speed_sign
- Sound Speed  
- Motor speed  

## Test Code Removal Procedure
- DMA, USART 不知道有没有用就暂时保留
- LED row output
    - In gpio.c, remove  
        >HAL_GPIO_WritePin(GPIOG, LED8_Pin_o|LED7_Pin_o|LED6_Pin_o|LED5_Pin_o, GPIO_PIN_RESET);
    - And
        >// led row output test  
        GPIO_InitStruct.Pin = LED8_Pin_o|LED7_Pin_o|LED6_Pin_o|LED5_Pin_o;  
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  
        GPIO_InitStruct.Pull = GPIO_NOPULL;  
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);  
    - In main.h remove
        >// led row output define begin  
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
