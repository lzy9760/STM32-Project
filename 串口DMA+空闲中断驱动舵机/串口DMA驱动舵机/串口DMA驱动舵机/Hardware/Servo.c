// 必须先包含Servo.h，才能识别SERVO_MAX_ANGLE/SERVO_MIN_ANGLE宏
#include "Servo.h"
// 补充类型定义
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

// 舵机PWM初始化：TIM2_CH1（PA0），50Hz频率（舵机标准频率）
void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;
    
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 2. 配置PA0为复用推挽输出
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 3. 配置TIM2时基：72MHz / 72 = 1MHz计数频率，ARR=20000→20ms周期（50Hz）
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStruct.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
    
    // 4. 配置TIM2_CH1为PWM模式1
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 1500; // 初始角度90度（对应1.5ms脉宽）
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStruct);
    
    // 5. 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
}

// 设置舵机角度：0度→500（0.5ms），180度→2500（2.5ms）
void Servo_SetAngle(u8 angle)
{
    u16 ccr_val;
    // 限制角度范围
    if(angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    
    // 强制转换为u16避免8位数值溢出
    ccr_val = 500 + ((u16)angle * 2000) / 180;
    TIM_SetCompare1(TIM2, ccr_val);
}
