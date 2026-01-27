#ifndef __MYI2C_H
#define __MYI2C_H
//MyI2C.h
#include "stm32f10x.h"

void I2C_Init_Config(void);
void I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
void I2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len);

#endif

