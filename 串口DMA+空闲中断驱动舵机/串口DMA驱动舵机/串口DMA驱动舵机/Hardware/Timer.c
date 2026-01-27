#include "Timer.h"
//Timer.c
u8 Timer1s_Flag = 0; // 1秒定时标志

// TIM3初始化：定时1秒（APB1时钟72MHz）
void TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 1. 开启TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 2. 配置时基参数：72MHz / 7200 = 10kHz计数频率，计数10000次=1秒
    TIM_TimeBaseInitStruct.TIM_Prescaler = 7200 - 1;       // 预分频
    TIM_TimeBaseInitStruct.TIM_Period = 10000 - 1;        // 自动重装值
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    
    // 3. 开启TIM3更新中断
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    
    // 4. 配置NVIC中断优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 5. 启动TIM3
    TIM_Cmd(TIM3, ENABLE);
}

// TIM3中断服务函数：每1秒置位标志
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        Timer1s_Flag = 1; // 置位1秒标志
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断标志
    }
}
