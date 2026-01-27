#include "LED.h"
#include "Delay.h"
#include "stm32f10x.h"

// LED端口定义（PA0-PA7，全部为普通IO，软件模拟呼吸）
static GPIO_TypeDef* LED_PORT[LED_NUM] = {
    GPIOA,GPIOA,GPIOA,GPIOA,
    GPIOA,GPIOA,GPIOA,GPIOA
};

static uint16_t LED_PIN[LED_NUM] = {
    GPIO_Pin_0,GPIO_Pin_1,GPIO_Pin_2,GPIO_Pin_3,
    GPIO_Pin_4,GPIO_Pin_5,GPIO_Pin_6,GPIO_Pin_7
};

void LED_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    // 初始化所有LED引脚为普通推挽输出（去掉硬件PWM）
    for(int i=0;i<LED_NUM;i++)
    {
        gpio.GPIO_Pin = LED_PIN[i];
        GPIO_Init(LED_PORT[i], &gpio);
    }

    // 所有LED初始灭
    LED_All_Off();
}

// 软件模拟LED呼吸亮度
// index：LED索引；brightness：亮度0-255（0=灭，255=最亮）
void LED_Breath_Control(uint8_t index, uint8_t brightness)
{
    if(index >= LED_NUM) return; // 防止越界

    // 呼吸周期：2ms（亮的时长=brightness*8us，灭的时长=(255-brightness)*8us）
    if(brightness > 0)
    {
        GPIO_SetBits(LED_PORT[index], LED_PIN[index]);
        Delay_us(brightness * 8); // 亮的时长随亮度增加
    }
    
    if(brightness < 255)
    {
        GPIO_ResetBits(LED_PORT[index], LED_PIN[index]);
        Delay_us((255 - brightness) * 8); // 灭的时长随亮度减少
    }
}

void LED_Set(uint8_t index, uint8_t state)
{
    if(index >= LED_NUM) return;
    
    if(state)
        GPIO_SetBits(LED_PORT[index], LED_PIN[index]);
    else
        GPIO_ResetBits(LED_PORT[index], LED_PIN[index]);
}

void LED_All_Off(void)
{
    for(int i=0;i<LED_NUM;i++)
        GPIO_ResetBits(LED_PORT[i], LED_PIN[i]);
}
