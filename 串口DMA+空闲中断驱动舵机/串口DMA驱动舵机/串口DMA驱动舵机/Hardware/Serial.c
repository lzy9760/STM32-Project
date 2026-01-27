#include "Serial.h"
#include <stdarg.h>
#include "string.h"
// 补充类型定义（如果main中已全局定义，这里可省略）
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

u8 USART1_RX_BUF[USART1_RX_BUF_SIZE];
u16 USART1_RX_LEN = 0;
u8 USART1_RX_Flag = 0;
u16 USART1_RX_Count = 0; // 接收次数统计

// 串口初始化：USART1 + DMA1_Channel5
void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    // 2. 配置GPIO：PA9(TX)复用推挽，PA10(RX)浮空输入 
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 3. 配置USART1
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);
    
    // 4. 配置DMA1_Channel5（USART1_RX）
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
    DMA_InitStruct.DMA_MemoryBaseAddr = (u32)USART1_RX_BUF;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize = USART1_RX_BUF_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStruct);
    DMA_Cmd(DMA1_Channel5, ENABLE);
    
    // 5. 开启USART1空闲中断
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    
    // 6. 配置NVIC
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    // 7. 开启USART1
    USART_Cmd(USART1, ENABLE);
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

// 重定向printf到USART1
int fputc(int ch, FILE* f)
{
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (u8)ch);
    return ch;
}

// 串口打印函数（支持格式化输出）
void Serial_Printf(const char* fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    for(u16 i=0; buf[i]!='\0'; i++)
    {
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, buf[i]);
    }
}

// USART1中断服务函数（处理空闲中断）
void USART1_IRQHandler(void)
{
    u32 temp;
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        // 清除空闲中断标志
        temp = USART1->SR;
        temp = USART1->DR;
        (void) temp; 
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
        
        // 关闭DMA，计算接收长度
        DMA_Cmd(DMA1_Channel5, DISABLE);
        USART1_RX_LEN = USART1_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);
        
        // 【修复结束符】先清空缓冲区末尾，再添加
        memset(USART1_RX_BUF + USART1_RX_LEN, 0, USART1_RX_BUF_SIZE - USART1_RX_LEN);
        // 确保结束符存在（即使接收长度为0，也给空字符串）
        USART1_RX_BUF[USART1_RX_BUF_SIZE - 1] = '\0';
        
        // 置位接收标志，接收次数+1
        USART1_RX_Flag = 1;
        USART1_RX_Count++;
        
        // 重置DMA
        DMA_SetCurrDataCounter(DMA1_Channel5, USART1_RX_BUF_SIZE);
        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
}


////封装printf函数(可变参数格式的封装)
//void Serial_Printf(char *format,...)//前面一个参数用来接收格式化字符串，后面一个参数用来接收后面的可变参数列表
//{
//	char string[100];
//	va_list arg;
//	va_start(arg,format);//从format位置开始接收参数列表，放在arg后面
//	vsprintf(string,format,arg);//要改成vsprinf，因为sprintf只能接收直接写的参数，封装格式要用vsprintf
//	va_end(arg);//释放参数表
//	Serial_SendString(string);//把string发送出去
//}


////tip:如果要发送中文，要么双方都选择UTF-8，并且加上--no-multibyte-chars参数，要么不加参数，都使用GB2312编码格式


////中断接收和变量的封装
//uint8_t Serial_GetRxFlag(void)
//{
//	if(Serial_RxFlag == 1)
//	{
//		Serial_RxFlag = 0;
//		return 1;
//	}
//	return 0;
//}

//uint8_t Serial_GetRxData(void)
//{
//	return Serial_RxData;
//}


//void USART1_IRQHandler(void)
//{
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)//先判断标志位
//	{
//		Serial_RxData = USART_ReceiveData(USART1);//读DR寄存器
//		Serial_RxFlag = 1;
//		
//		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//手动清除标志位
//	}
//}









