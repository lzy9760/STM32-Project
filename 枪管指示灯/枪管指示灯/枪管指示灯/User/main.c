#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"


// LED引脚定义 (A0-A9)
#define LED_PIN_0  GPIO_Pin_0
#define LED_PIN_1  GPIO_Pin_1
#define LED_PIN_2  GPIO_Pin_2
#define LED_PIN_3  GPIO_Pin_3
#define LED_PIN_4  GPIO_Pin_4
#define LED_PIN_5  GPIO_Pin_5
#define LED_PIN_6  GPIO_Pin_6
#define LED_PIN_7  GPIO_Pin_7
#define LED_PIN_8  GPIO_Pin_8
#define LED_PIN_9  GPIO_Pin_9

#define LED_PINS (LED_PIN_0 | LED_PIN_1 | LED_PIN_2 | LED_PIN_3 | LED_PIN_4 | \
                 LED_PIN_5 | LED_PIN_6 | LED_PIN_7 | LED_PIN_8 | LED_PIN_9)

// 按键引脚定义 (B11)
#define KEY_PIN GPIO_Pin_11

// 热量参数
#define HEAT_PER_SHOT 10        // 每发射一发增加10点热量
#define MAX_HEAT 230           // 最大热量值
#define COOLING_RATE 14        // 冷却速率：14/秒

// 全局变量
volatile uint32_t current_heat = 0;      // 当前热量值
volatile uint8_t burst_mode = 0;         // 连发模式标志
volatile uint32_t last_shot_time = 0;    // 上次发射时间
volatile uint8_t key_pressed = 0;        // 按键按下标志
volatile uint32_t key_press_time = 0;    // 按键按下时间

void GPIO_Config(void);
void TIM2_Init(void);
void Update_LEDs(void);
void Single_Shot(void);
void Start_Burst(void);
void Stop_Burst(void);

int main(void)
{
    SystemInit();
    GPIO_Config();
    TIM2_Init();
    OLED_Init();
    
    // 显示初始热量
    OLED_ShowNum(2, 1, current_heat, 3);
    OLED_ShowString(2, 5, "/230");
    
    // 初始更新LED显示
    Update_LEDs();
    
    GPIO_SetBits(GPIOA, LED_PINS);   // 关闭所有LED
    Update_LEDs(); // 恢复初始状态
    
    while(1)
    {
        // 检测按键状态
        if(GPIO_ReadInputDataBit(GPIOB, KEY_PIN) == 0) // 按键按下
        {
            Delay_ms(10); // 简单消抖
            if(GPIO_ReadInputDataBit(GPIOB, KEY_PIN) == 0) // 确认按下
            {
                if(!key_pressed) // 首次按下
                {
                    key_pressed = 1;
                    key_press_time = 0;
                    Single_Shot(); // 短按单发
                }
                else
                {
                    key_press_time++;
                    // 长按判断（约200ms后进入连发模式）
                    if(key_press_time > 20 && !burst_mode) // 10*20ms = 200ms
                    {
                        Start_Burst();
                    }
                }
            }
        }
        else // 按键释放
        {
            if(key_pressed)
            {
                Stop_Burst();
                key_pressed = 0;
            }
        }
        
        Delay_ms(20);
    }
}

// GPIO配置
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 开启GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置LED引脚 (A0-A9) 为推挽输出
    GPIO_InitStructure.GPIO_Pin = LED_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置按键引脚 (B11) 为上拉输入
    GPIO_InitStructure.GPIO_Pin = KEY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始关闭所有LED
    GPIO_SetBits(GPIOA, LED_PINS);
}

// 定时器2初始化
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 定时器配置：20ms中断
    TIM_TimeBaseStructure.TIM_Period = 2000 - 1; // 20ms = 2000 * 0.01ms
    TIM_TimeBaseStructure.TIM_Prescaler = 720 - 1; // 72MHz/720 = 100kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM2, ENABLE);
}

// 单发射击
void Single_Shot(void)
{
    if(current_heat + HEAT_PER_SHOT <= MAX_HEAT)
    {
        current_heat += HEAT_PER_SHOT;
        Update_LEDs();
        // 更新OLED显示
        OLED_ShowNum(2, 1, current_heat, 3);
    }
}

// 开始连发
void Start_Burst(void)
{
    burst_mode = 1;
    last_shot_time = 0;
}

// 停止连发
void Stop_Burst(void)
{
    burst_mode = 0;
}

// 更新LED显示
void Update_LEDs(void)
{
    // 计算需要点亮的LED数量（热量百分比）
    uint8_t leds_to_light = (current_heat * 10 + MAX_HEAT - 1) / MAX_HEAT;
    
    // 确保不超过10个LED
    if(leds_to_light > 10) 
        leds_to_light = 10;
    
    // 单独控制每个LED引脚
    if(leds_to_light >= 1) GPIO_ResetBits(GPIOA, LED_PIN_0); else GPIO_SetBits(GPIOA, LED_PIN_0);
    if(leds_to_light >= 2) GPIO_ResetBits(GPIOA, LED_PIN_1); else GPIO_SetBits(GPIOA, LED_PIN_1);
    if(leds_to_light >= 3) GPIO_ResetBits(GPIOA, LED_PIN_2); else GPIO_SetBits(GPIOA, LED_PIN_2);
    if(leds_to_light >= 4) GPIO_ResetBits(GPIOA, LED_PIN_3); else GPIO_SetBits(GPIOA, LED_PIN_3);
    if(leds_to_light >= 5) GPIO_ResetBits(GPIOA, LED_PIN_4); else GPIO_SetBits(GPIOA, LED_PIN_4);
    if(leds_to_light >= 6) GPIO_ResetBits(GPIOA, LED_PIN_5); else GPIO_SetBits(GPIOA, LED_PIN_5);
    if(leds_to_light >= 7) GPIO_ResetBits(GPIOA, LED_PIN_6); else GPIO_SetBits(GPIOA, LED_PIN_6);
    if(leds_to_light >= 8) GPIO_ResetBits(GPIOA, LED_PIN_7); else GPIO_SetBits(GPIOA, LED_PIN_7);
    if(leds_to_light >= 9) GPIO_ResetBits(GPIOA, LED_PIN_8); else GPIO_SetBits(GPIOA, LED_PIN_8);
    if(leds_to_light >= 10) GPIO_ResetBits(GPIOA, LED_PIN_9); else GPIO_SetBits(GPIOA, LED_PIN_9);
}

// 冷却处理
void Cooling_Process(void)
{
    static uint32_t cooling_counter = 0;
    
    cooling_counter++;
    if(cooling_counter >= 50) // 50 * 20ms = 1000ms
    {
        cooling_counter = 0;
        if(current_heat > 0)
        {
            if(current_heat > COOLING_RATE)
                current_heat -= COOLING_RATE;
            else
                current_heat = 0;
            
            Update_LEDs();
            // 更新OLED显示
            OLED_ShowNum(2, 1, current_heat, 3);
        }
    }
}

// 定时器2中断服务函数
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // 冷却处理
        Cooling_Process();
        
        // 连发处理
        if(burst_mode)
        {
            last_shot_time++;
            if(last_shot_time >= 1) // 每20ms发射一次
            {
                last_shot_time = 0;
                Single_Shot();
            }
        }
    }
}

