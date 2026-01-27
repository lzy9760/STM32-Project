#include "stm32f10x.h"
#include "Timer.h"
 extern uint16_t num; // 全局变量，记录秒数

void Timer_Init(void)
{
    // 使能定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 选择内部时钟
    TIM_InternalClockConfig(TIM2);
    
    // 配置定时器时基
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;//1ms中断
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;        // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;    // 预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // 清除更新标志
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    
    // 使能更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    // 配置NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

//void TIM2_IRQHandler(void)
//{
//    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//    {
//        num++; // 每秒增加1
//        
//        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//    }
//}
