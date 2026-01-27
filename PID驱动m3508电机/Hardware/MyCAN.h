//MyCAN.h
#ifndef __CAN_H
#define __CAN_H

#include "stm32f10x.h"

void CAN1_Init(void);
void CAN1_Send_Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
