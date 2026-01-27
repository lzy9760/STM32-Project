#include "stm32f10x.h"                  // Device header
uint16_t LED1_Count;//定义计次变量
uint8_t LED1_Mode;//全局变量，指定LED1的模式
uint16_t LED2_Count;
uint8_t LED2_Mode;
void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7);
}



void LED1_ON(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
}

void LED1_Off(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
}

//void LED1_Turn(void)
//{
//	if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_6) == 0)
//	{
//		GPIO_SetBits(GPIOA, GPIO_Pin_6);//如果输出0，那么置为1
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOA, GPIO_Pin_6);//否则置为0
//	}
//}


void LED2_ON(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);
}

void LED2_Off(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
}


//void LED2_Turn(void)
//{
//	if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7) == 0)
//	{
//		GPIO_SetBits(GPIOA, GPIO_Pin_7);//如果输出0，那么置为1
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOA, GPIO_Pin_7);//否则置为0
//	}
//}


//LED1
void LED1_SetMode(uint8_t Mode)
{
	LED1_Mode = Mode;
}
void LED2_SetMode(uint8_t Mode)
{
	LED2_Mode = Mode;
}

void LED_Tick(void)
{
	if(LED1_Mode == 0)
	{
		LED1_Off();
	}
	if(LED1_Mode == 1)
	{
		LED1_ON();
	}
	else if(LED1_Mode == 2)
	{
		LED1_Count++;
	if(LED1_Count >= 999)//或写成LED1_Count %= 1000,也可防止自增越界
	{
		LED1_Count = 0;
		
	}
	if(LED1_Count < 500)//定时器中断完成LED慢闪
		{
			LED1_ON();
		}
		else
		{
			LED1_Off();
		}
	}
	else if(LED1_Mode == 3)
	{
		LED1_Count++;
	if(LED1_Count >= 99)//或写成LED1_Count %= 1000,也可防止自增越界
	{
		LED1_Count = 0;
		
	}
	if(LED1_Count < 50)//定时器中断完成LED快闪，快闪周期值要从1000变到100
		{
			LED1_ON();
		}
		else
		{
			LED1_Off();
		}
	}
		else if(LED1_Mode == 4)
	{
		LED1_Count++;
	if(LED1_Count >= 999)//或写成LED1_Count %= 1000,也可防止自增越界
	{
		LED1_Count = 0;
		
	}
	if(LED1_Count < 100)//定时器中断完成LED点闪
		{
			LED1_ON();
		}
		else
		{
			LED1_Off();
		}
	}
	
	
	
	//LED2
	if(LED2_Mode == 0)
	{
		LED2_Off();
	}
	if(LED2_Mode == 1)
	{
		LED2_ON();
	}
	else if(LED2_Mode == 2)
	{
		LED2_Count++;
	if(LED2_Count >= 999)//或写成LED1_Count %= 1000,也可防止自增越界
	{
		LED2_Count = 0;
		
	}
	if(LED2_Count < 500)//定时器中断完成LED慢闪
		{
			LED2_ON();
		}
		else
		{
			LED2_Off();
		}
	}
	else if(LED2_Mode == 3)
	{
		LED2_Count++;
	if(LED2_Count >= 99)//或写成LED1_Count %= 1000,也可防止自增越界
	{
		LED2_Count = 0;
		
	}
	if(LED2_Count < 50)//定时器中断完成LED快闪，快闪周期值要从1000变到100
		{
			LED2_ON();
		}
		else
		{
			LED2_Off();
		}
	}
		else if(LED2_Mode == 4)
	{
		LED2_Count++;
	if(LED2_Count >= 999)//或写成LED1_Count %= 1000,也可防止自增越界
	{
		LED2_Count = 0;
		
	}
	if(LED2_Count < 100)//定时器中断完成LED点闪
		{
			LED2_ON();
		}
		else
		{
			LED2_Off();
		}
	}
}
