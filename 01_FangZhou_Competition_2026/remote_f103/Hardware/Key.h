#ifndef __KEY_H
#define __KEY_H

#include "main.h"

#define KEY_LIFT_UP_MASK        0x01U
#define KEY_LIFT_DOWN_MASK      0x02U
#define KEY_SERVO_75_MASK       0x04U
#define KEY_SERVO_25_MASK       0x08U

void Key_Init(void);
uint8_t Key_GetNum(void);
uint8_t Key_GetState(void);
uint8_t Key_GetLiftButtons(void);
void Key_Tick(void);

#endif
