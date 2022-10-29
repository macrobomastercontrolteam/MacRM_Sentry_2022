# Sentry Ultrasonic Sensor Module
### by MacRobomaster 2020 Control Team
## Directories
- Ultrasonic Test by LEDs: test ultrasonic measuring function with the row of LEDs  
- Sentry Ultrasonic Module - Modified Sumo Code: main code

## System Notes
- Two Type A Dev Board: one controls left-right movement, one controls rotation of gimbal
- Left-right movement motor: M2006 P36; corresponding control unit：C610

## States
- State 0 (normal mode): go left + wiggle randomly for three times around one center point
- State 1 (normal mode): go right + wiggle randomly for three times around one center point
- State 2 (attack mode): go left + wiggle twice
- State 3 (attack mode): go right + wiggle twice  
    Each time at the end of all states check ENEMY_DETECT to enter either attack mode or normal mode

## Algorithm
- Ultrasonic measure twice at the start of each while loop
    1. If both sides are blocked, move towards the middle point and redo measuring
    2. if only one side blocked, go to state 0/1/2/3
- Wiggle  
    Take state 0 (to the left) for example

    <img src="doc/Wiggle%20schematic.jpg" alt="Wiggle schematic" width="35%"/>
    
    - Time x<sub>pos</sub> and x<sub>2</sub> are randomly generated, while x<sub>1</sub>=x<sub>pos</sub>/8+100 (this requires x<sub>pos</sub>>114) and last return time is 0.8x<sub>2</sub>
    - full_travel_tick is the max time for a one-way travel in wheel_speed_medium.  
    - In attack states (2 & 3), full_travel is halved and wiggling is executed twice in a one-way travel
    - It's possible that the wiggle state is terminated half-way through because of wrong ultrasonic detection.

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
- wiggle_tick and full_travel_tick
- random_list

## Functions
- usDelay
    - time (us) parameter must be within range of uint16_t (sugguestion: 1-65500)
    - used TIM4 (prescaler=APB1_freq-1 (in MHz), ARR=0xffff-1)
- ULTRASONIC_MEASURE_RIGHT and ULTRASONIC_MEASURE_LEFT
    - measure at most 1 meter

## Test Code Removal Procedure
- DMA, USART 不知道有没有用就暂时保留
- LED row output function additional code
    - In gpio.c,
        >HAL_GPIO_WritePin(GPIOG, LED8_Pin_o|LED7_Pin_o|LED6_Pin_o|LED5_Pin_o, GPIO_PIN_RESET);  

        and  

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
