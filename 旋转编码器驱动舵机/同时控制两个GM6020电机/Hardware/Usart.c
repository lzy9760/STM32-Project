#include "Usart.h"

static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

void USART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

void USART_SendFloat(float data)
{
    uint8_t *p = (uint8_t *)&data;
    for (uint8_t i = 0; i < 4; i++)
    {
        USART_SendData(USART1, p[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

void USART_SendVOFA_Tail(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        USART_SendData(USART1, vofa_tail[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}


