#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h"

void MyI2C_Init(void);
void MyI2C_WriteReg(uint8_t dev, uint8_t reg, uint8_t data);
void MyI2C_ReadRegs(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len);

#endif
