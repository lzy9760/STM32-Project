// beep_simple_pwm.c
#include "stm32f10x.h"
#include "delay.h"

#define BEEP_PORT    GPIOB
#define BEEP_PIN     GPIO_Pin_12
#define BEEP_TIM     TIM4

void Beep_Init_Simple(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 配置GPIO
    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
    
    // 配置TIM4
    TIM_TimeBaseStructure.TIM_Period = 499;        // 500-1 = 499
    TIM_TimeBaseStructure.TIM_Prescaler = 719;     // 72MHz/720 = 100kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(BEEP_TIM, &TIM_TimeBaseStructure);
    
    // 配置PWM
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  // 低电平触发
    TIM_OCInitStructure.TIM_Pulse = 250;           // 50%占空比
    
    TIM_OC1Init(BEEP_TIM, &TIM_OCInitStructure);
    
    // 使能预装载
    TIM_OC1PreloadConfig(BEEP_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BEEP_TIM, ENABLE);
    
    // 启动定时器
    TIM_Cmd(BEEP_TIM, ENABLE);
}

void Beep_Test_PWM(void)
{
    Beep_Init_Simple();
    
    // 这个配置会产生 100kHz / 500 = 200Hz 的声音
    
    while(1)
    {
        // 持续发声
        Delay_ms(5000);
        
        // 停止
        TIM_Cmd(BEEP_TIM, DISABLE);
        Delay_ms(2000);
        
        // 重新开始
        TIM_Cmd(BEEP_TIM, ENABLE);
    }
}
