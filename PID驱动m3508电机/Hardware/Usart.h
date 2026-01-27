#ifndef __USART_H
#define __USART_H

void USART1_SendMonitor(void);
void USART1_SendFloat(float f);
void USART1_Init(uint32_t baud);
void USART1_IRQHandler(void);

#endif
