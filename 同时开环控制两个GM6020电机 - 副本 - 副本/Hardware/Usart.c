#include "stm32f10x.h"                  // Device header
#include "stdio.h"
#include "stdarg.h"

//Usart.c
//Step1:开启时钟，把USART和GPIO的时钟打开
//Step2:初始化GPIO，把TX配置成复用输出，RX配置成输入
//Step3:配置USART，直接使用一个结构体
//Step4:如果只需要发送的功能，直接开启USART，初始化结束，如果需要接收的功能，可能还要配置中断(在开启USART前加上ITConfig和NVIC的代码)
//Step5:初始化完成之后，如果要发送或接收数据，调用一个库函数即可，获取发送和接收的状态，调用获取标志位的函数


void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//开启USART1的时钟，USART1是APB2的外设，其他都是APB1的外设
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//开启GPIOA的时钟
	
	//初始化引脚，把GPIOA的PA9配置为复用推挽输出模式，供USART1的TX使用
	GPIO_InitTypeDef nb;
	nb.GPIO_Mode = GPIO_Mode_AF_PP;//TX引脚由USART外设控制，选择复用输出推挽模式
  nb.GPIO_Pin = GPIO_Pin_9;
	nb.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&nb);
	
	
	//初始化USART1
	USART_InitTypeDef nbb;
	nbb.USART_BaudRate = 115200;//波特率，这里写一个数值，下面Init函数内部会自动算好115200对应的分配系数，然后写到BRR寄存器中
	nbb.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制，不使用流控，选择None
	nbb.USART_Mode = USART_Mode_Tx;//串口模式
	nbb.USART_Parity = USART_Parity_No;//校验位，选择无校验
	nbb.USART_StopBits = USART_StopBits_1;//停止位，选择1位停止位
	nbb.USART_WordLength = USART_WordLength_8b;//字长，不需要校验，字长选择8位
	USART_Init(USART1,&nbb);
	USART_Cmd(USART1,ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);//Byte变量写入TDR发送数据寄存器，还要等待一下，等TDR数据转移到移位寄存器，否则会产生数据覆盖
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);//TXE是发送数据寄存器为空的意思
	//如果发送数据寄存器非空，就一直等待，否则才跳出while循环
}

void Serial_SendArray(uint8_t *Array,uint16_t Length)
{
	uint16_t i;
	for(i= 0 ;i<Length;i++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char*String)
{
	uint8_t i;
	for(i = 0; String[i] != '\0'/*这里也可以直接写0*/;i++)
	{
		Serial_SendByte(String[i]);
	}
	
	//或者也可以用while循环代替for循环
	//while(*String != '\0')
	//{
		//Serial_SendByte(*String++);
	//}
}

uint32_t Serial_Pow(uint16_t X,uint16_t Y)
{
	uint32_t ret = 1;
	while(Y--)
	{
		ret *= X;
	}
	return ret;
}

void Serial_SendNumber(uint32_t Num,uint8_t Length)
{
	uint8_t i;
	for(i = 0;i<Length;i++)
	{
		Serial_SendByte(Num / Serial_Pow(10,Length-1-i) % 10 + '0');//注意最终要以字符的形式显示，还要加上偏移量'0'或0x30
	}
}

int fputc(int ch,FILE *f)//这是fputc函数的原型
{
     Serial_SendByte(ch);	//把fputc重定向到串口，从而将printf函数移植到串口,fputc函数是printf函数的底层
	   return ch;
}

//封装printf函数(可变参数格式的封装)
void Serial_Printf(char *format,...)//前面一个参数用来接收格式化字符串，后面一个参数用来接收后面的可变参数列表
{
	char string[100];
	va_list arg;
	va_start(arg,format);//从format位置开始接收参数列表，放在arg后面
	vsprintf(string,format,arg);//要改成vsprinf，因为sprintf只能接收直接写的参数，封装格式要用vsprintf
	va_end(arg);//释放参数表
	Serial_SendString(string);//把string发送出去
}


//tip:如果要发送中文，要么双方都选择UTF-8，并且加上--no-multibyte-chars参数，要么不加参数，都使用GB2312编码格式
