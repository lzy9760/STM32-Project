//// LED.c
//#include "LED.h"

//void LED_Init(void)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//}

//void LED_Toggle(void)
//{
//    GPIOC->ODR ^= GPIO_Pin_13;
//}

//void Delay_Simple(uint32_t t)
//{
//    while (t--);
//}
