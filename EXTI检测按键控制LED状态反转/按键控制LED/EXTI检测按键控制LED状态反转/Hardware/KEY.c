#include "stm32f10x.h"                  // Device header
#include "Delay.h"
void KEY_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
	
	uint8_t KEY_Getnum(void)//相当于unsigned char类型
	{
		uint8_t KEYNUM = 0;
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 0)
		{
			Delay_ms(50);//取消按下前摇
			while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 0)
			Delay_ms(50);//取消松手后摇
			
			KEYNUM = 1;
		}
		
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 0)
		{
			Delay_ms(50);//取消按下前摇
			while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 0)
			Delay_ms(50);//取消松手后摇
			
			KEYNUM = 2;
		}
		
		return KEYNUM;
	}

