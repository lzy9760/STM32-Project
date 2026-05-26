#ifndef __KEY_H
#define __KEY_H

#include "main.h"

void Key_Init(void);
uint8_t Key_GetNum(void);
uint8_t Key_GetState(void);
void Key_Tick(void);

#endif
