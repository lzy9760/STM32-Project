#ifndef __NRF24L01_H
#define __NRF24L01_H

#include <stdint.h>
#include "NRF24L01_Define.h"

/*外部可调用全局数组***********/

extern uint8_t NRF24L01_TxAddress[];
extern uint8_t NRF24L01_TxPacket[];

extern uint8_t NRF24L01_RxAddress[];
extern uint8_t NRF24L01_RxPacket[];

/***********外部可调用全局数组*/


/*函数声明*********************/

/*指令实现*/
uint8_t NRF24L01_ReadReg(uint8_t RegAddress);
void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);
void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Data);
void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);
void NRF24L01_ReadRxPayload(uint8_t *DataArray, uint8_t Count);
void NRF24L01_WriteTxPayload(uint8_t *DataArray, uint8_t Count);
void NRF24L01_FlushTx(void);
void NRF24L01_FlushRx(void);
uint8_t NRF24L01_ReadStatus(void);

/*功能函数*/
void NRF24L01_PowerDown(void);
void NRF24L01_StandbyI(void);
void NRF24L01_Rx(void);
void NRF24L01_Tx(void);

void NRF24L01_Init(void);
uint8_t NRF24L01_Send(void);
uint8_t NRF24L01_Receive(void);
uint8_t NRF24L01_ReadAckPayload(void);
uint8_t NRF24L01_SendPayload(uint8_t *Address, uint8_t *DataArray, uint8_t Count);
extern uint8_t NRF24L01_PoseTxAddress[5];
extern uint8_t NRF24L01_PoseRxAddress[5];
void NRF24L01_UpdateRxAddress(void);

/*********************函数声明*/


#endif


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
