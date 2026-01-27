#ifndef __USART_H
#define __USART_H
//Usart.h
#include <stdio.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);
void Serial_SendString(char*String);
uint32_t Serial_Pow(uint16_t X,uint16_t Y);
void Serial_SendNumber(uint32_t Num,uint8_t Length);
void Serial_Printf(char *format,...);

#endif
