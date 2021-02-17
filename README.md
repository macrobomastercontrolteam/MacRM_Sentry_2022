# Sentry Ultrasonic Sensor Module
### by MacRobomaster 2020 Control Team

## System Notes
- This code is used on development board A for test reasons
- 上下两个C板：上面C板控制左右运动马达，下面控制云台+拨弹轮  
- 上下拨弹轮synchronize 
- 云台与Chassis用GM6020连接

## General Algorithm
1. memorize the previous location (in less frequency)  
2. if both sensors offline, use the last location  
3. If both sensors outputs too short distances, determine how to move using the previous location and measure by the sensors again  
4. if one of the sensors doesn't work, determine using the other sensor and the previous location  

## Pin setup
- Echo & Trig 1 (left):   PF0 & PF1  
- Echo & Trig 2 (right):  PI9 & PF10  
- LED1-8: PG1-8

## Code Setup
- usDelay
    - us parameter must be within range of uint16_t (sugguest 1-65500)
    - used TIM4 (prescaler=freq-1 (in MHz), ARR=0xffff-1)
- wiggle() -> wiggle around a wiggling center point

## State Machine

## Parameters to be determined on-site
- Sound Speed  
- Motor speed  