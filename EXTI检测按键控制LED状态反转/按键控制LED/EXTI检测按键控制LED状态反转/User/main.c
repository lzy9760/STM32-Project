#include "stm32f10x.h"  // STM32F10x系列微控制器的头文件，包含所有外设寄存器定义
#include "Delay.h"       // 延时函数头文件

// 函数声明
void App_LED_Init(void);           // LED初始化函数
void APP_Button_Init(void);        // 按键初始化函数
void EXTI0_IRQHandler(void);       // EXTI0中断服务函数（处理PB0按键）
void EXTI15_10_IRQHandler(void);   // EXTI15-10中断服务函数（处理PB11按键）

/**
  *主函数
  */
int main(void)
{
    // 系统时钟初始化 - 配置系统时钟、HSE、PLL等
    SystemInit();
    
    // 配置NVIC中断优先级分组为第2组
    // 第2组：4位用于抢占优先级，0位用于子优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // 初始化LED和按键
    App_LED_Init();
    APP_Button_Init();
    
    // 初始状态：LED灭（PA1输出高电平）
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
    
    // 主循环
    while(1)
    {
        // 主循环可以添加其他任务
        // 此处使用延时函数，避免空循环占用过多CPU资源
        Delay_ms(100);
    }
}



/**
  * 按键初始化函数
  * 配置PB0和PB11为上拉输入模式，并配置外部中断
  */
void APP_Button_Init(void)
{
    // 1. 开启GPIOB时钟 - 使能GPIOB端口的时钟，才能配置和使用GPIOB
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 定义GPIO初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 配置PB0、PB11为上拉输入模式
    // 上拉输入：引脚默认被拉高，当按键按下时接地变为低电平
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_11;  // 选择PB0和PB11引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;            // 上拉输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // 输入模式下速度设置影响不大，但需要设置
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // 应用配置到GPIOB
    
    // 2. 开启AFIO时钟 - 使能复用功能时钟，用于配置外部中断线映射
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // 3. 配置GPIO引脚连接到EXTI线 - 将PB0和PB11映射到对应的外部中断线
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);   // PB0 -> EXTI0线
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);  // PB11 -> EXTI11线
    
    // 4. 配置EXTI外部中断
    EXTI_InitTypeDef EXTI_InitStructure;
    
    // 设置EXTI公共参数
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;      // 中断模式（非事件模式）
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // 下降沿触发（从高电平变为低电平时触发）
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                // 使能EXTI线
    
    // 配置EXTI0线 (对应PB0引脚)
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;               // 选择EXTI0线
    EXTI_Init(&EXTI_InitStructure);                          // 应用配置
    
    // 配置EXTI11线 (对应PB11引脚)
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;              // 选择EXTI11线
    EXTI_Init(&EXTI_InitStructure);                          // 应用配置
    
    // 5. 配置NVIC嵌套向量中断控制器
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 配置EXTI0中断向量（PB0按键）
    // EXTI0有独立的中断向量：EXTI0_IRQn
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;         // 选择EXTI0中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 设置抢占优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;       // 设置子优先级为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          // 使能该中断通道
    NVIC_Init(&NVIC_InitStructure);                          // 应用配置
    
    // 配置EXTI15_10中断向量（包含PB11按键）
    // EXTI11属于EXTI15_10中断向量组：EXTI15_10_IRQn
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;     // 选择EXTI15-10中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 设置抢占优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;       // 设置子优先级为2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          // 使能该中断通道
    NVIC_Init(&NVIC_InitStructure);                          // 应用配置
}



/**
  *   LED初始化函数
  *  
  * 
  *   配置PA1为推挽输出模式，用于控制LED
  */
void App_LED_Init(void)
{
    // 开启GPIOA时钟 - 使能GPIOA端口的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 定义GPIO初始化结构体
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 配置PA1为推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                // 选择PA1引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         // 推挽输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // 输出速度50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                   // 应用配置到GPIOA
}



/**
  *   EXTI0中断服务函数
  * 
  * 
  *   处理PB0引脚的外部中断，按下PB0时点亮LED
  */
void EXTI0_IRQHandler(void)
{
    // 检查EXTI0线是否产生了中断请求
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // 添加延时去抖动 - 消除机械按键的抖动现象
        Delay_ms(20);
        
        // 再次检测按键状态，确认按键确实按下（非干扰信号）
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0) // 读取PB0引脚状态
        {
            // PB0按键确认按下，点亮LED（PA1输出低电平）
            GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        }
        
        // 清除EXTI0线的中断挂起位 - 必须清除，否则会持续产生中断
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}



/**
  *   EXTI15-10中断服务函数
  * 
  * 
  *   处理PB11引脚的外部中断，按下PB11时熄灭LED
  */
void EXTI15_10_IRQHandler(void)
{
    // 检查EXTI11线是否产生了中断请求
    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        // 添加延时去抖动 - 消除机械按键的抖动现象
        Delay_ms(20);
        
        // 再次检测按键状态，确认按键确实按下（非干扰信号）
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0) // 读取PB11引脚状态
        {
            // PB11按键确认按下，熄灭LED（PA1输出高电平）
            GPIO_SetBits(GPIOA, GPIO_Pin_1);
        }
        
        // 清除EXTI11线的中断挂起位 - 必须清除，否则会持续产生中断
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
}
