#include "stm32f10x.h"                  // Device header
#include "Delay.h"
uint8_t Key_Num;

void KEY_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}
	
	uint8_t Key_GetNum(void)
	{
		uint8_t Temp;
		if(Key_Num)
		{
			Temp = Key_Num;
			Key_Num = 0;//为什么要置为0？
			return Temp;
		}
		return 0;
	}

uint8_t KEY_GetState(void)//相当于unsigned char类型
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0)
		{
			return 1;
		}
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 0)
		{
			return 2;
		}
		return 0;
	}

	void Key_Tick(void)
	{
		static uint8_t Count;
		static uint8_t CurState,PrevState;
		Count++;
		if(Count >= 20)//定时中断，每隔20ms读取一次当前引脚值和上次引脚值
		{
			Count = 0;
			
			PrevState = CurState;
			CurState = KEY_GetState();
			
			if(CurState == 0 && PrevState != 0)
			{
				Key_Num = PrevState;
			}
		}
	}
