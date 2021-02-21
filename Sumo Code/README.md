# Sentry Ultrasonic Sensor Module
### by MacRobomaster 2020 Control Team

## System Notes
- 暂时用A板
- 左右移动只用一个M3508马达
- 上下两个C板：上面C板控制左右运动马达，下面控制云台+拨弹轮  
- 云台与Chassis用GM6020连接

## General Algorithm
1. Ultrasonic measure twice at the start of each while loop
    1. If both sides are blocked, move towards the middle point and redo measuring
    2. if only one side blocked, go to state 0/1/2/3

## State Machine
- State 0: go left and wiggle randomly for three times around one center point
- State 1: go right and wiggle randomly for three times around one center point
- State 2: go left + enemy attack mode (wiggle twice)
- State 3: go right + enemy attack mode (wiggle twice)  
    At the end of all states check ENEMY_DETECT whether enter attack mode or enter normal mode (State 0 & 1)

## Pin setup
- Test LED Code
    - Echo & Trig 1 (left):   PF0 & PF1  
    - Echo & Trig 2 (right):  PI9 & PF10  
- Modified Sumo Code
    - Echo & Trig 1 (left):   PA4 & PFA5
    - Echo & Trig 2 (right):  PC1 & PC5  
- LED_o 1-8: PG1-8
- LED_RED_o & LED_GREEN_o: PE11 & PF14
- KEY: reset button (PB2)

## Things to be modified
- ENEMY_DETECT - determined from other modules; start attack mode  
    The attack mode could work with computer vision and ballastics to avoid bullets in the future
- int8_t left_speed_sign
- Motor variables
- wiggle_tick
- random_list

## Functions
- usDelay
    - time (us) parameter must be within range of uint16_t (sugguestion: 1-65500)
    - used TIM4 (prescaler=APB1_freq-1 (in MHz), ARR=0xffff-1)

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
