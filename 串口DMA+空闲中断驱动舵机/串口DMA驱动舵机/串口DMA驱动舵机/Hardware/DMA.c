#include "DMA.h"
#include "Serial.h"
#include "stm32f10x.h"


void DMA_RX_Init(void)
{
DMA_InitTypeDef DMA_InitStruct;


RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


DMA_DeInit(DMA1_Channel5);
DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)Serial_RxBuf;
DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
DMA_InitStruct.DMA_BufferSize = SERIAL_RX_BUF_SIZE;
DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
DMA_InitStruct.DMA_Priority = DMA_Priority_High;
DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
DMA_Init(DMA1_Channel5, &DMA_InitStruct);


USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);//使能USART1的DMA请求
DMA_Cmd(DMA1_Channel5, ENABLE);
}
