#include "stm32f10x.h"                  // Device header
#include "Monitor.h"
#include "stdio.h"

void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}


void USART1_SendFloat(float f)
{
    char buf[16];
    sprintf(buf, "%.2f", f);
    for (int i = 0; buf[i]; i++)
    {
        USART_SendData(USART1, buf[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

void USART1_SendMonitor(void)
{
    USART_SendData(USART1, '$');
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

    USART1_SendFloat(monitor.kp);      USART_SendData(USART1, ',');
    USART1_SendFloat(monitor.ki);      USART_SendData(USART1, ',');
    USART1_SendFloat(monitor.kd);      USART_SendData(USART1, ',');
    USART1_SendFloat(monitor.target_speed); USART_SendData(USART1, ',');
    USART1_SendFloat(monitor.actual_speed); USART_SendData(USART1, ',');
    USART1_SendFloat(monitor.pid_out);

    USART_SendData(USART1, '\n');
}


char rx_buf[32];
uint8_t rx_idx = 0;

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        char ch = USART_ReceiveData(USART1);

        if (ch == '\n')
        {
            rx_buf[rx_idx] = 0;

            if (sscanf(rx_buf, "kp=%f", &monitor.kp));
            else if (sscanf(rx_buf, "ki=%f", &monitor.ki));
            else if (sscanf(rx_buf, "kd=%f", &monitor.kd));
            else if (sscanf(rx_buf, "target=%f", &monitor.target_speed));

            rx_idx = 0;
        }
        else
        {
            rx_buf[rx_idx++] = ch;
        }
    }
}


