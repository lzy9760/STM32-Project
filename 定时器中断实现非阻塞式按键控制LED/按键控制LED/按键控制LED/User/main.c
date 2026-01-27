#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "LED.h"
#include "KEY.h"
#include "Timer.h"
#include "OLED.h"

uint8_t KeyNum;
uint8_t LED1Mode;
uint8_t LED2Mode;
uint16_t num = 0;
void Key_Tick(void);
uint8_t Key_GetNum(void);
void LED_Tick(void);
void LED1_SetMode(uint8_t Mode);
void LED2_SetMode(uint8_t Mode);


int main()
{
	LED_Init();
	KEY_Init();
	Timer_Init();
	OLED_Init();
	
	OLED_ShowString(1,1,"i:");
	OLED_ShowString(2,1,"LED1Mode:");
	OLED_ShowString(3,1,"LED2Mode:");
	
	while (1)
	{
		
		
		KeyNum = Key_GetNum();
		if(KeyNum == 1)
		{
			//FlashFlag = !FlashFlag;
			LED1Mode++;//标志位
			LED1Mode %= 5;
			LED1_SetMode(LED1Mode);
		}
		
		
		if(KeyNum == 2)
		{
			//FlashFlag = !FlashFlag;
			LED2Mode++;//标志位
			LED2Mode %= 5;
			LED2_SetMode(LED2Mode);
		}
//		if(FlashFlag)//SetMode函数是非阻塞的
//		{
//			LED1_SetMode(1);//开始闪烁
//		}
//		else
//		{
//			LED1_SetMode(0);//停止闪烁
//		}
	
		OLED_ShowNum(1,3,num++,5);
		OLED_ShowNum(2,10,LED1Mode,1);
		OLED_ShowNum(3,10,LED2Mode,1);
	}
	
}


void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        Key_Tick();
        LED_Tick();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
