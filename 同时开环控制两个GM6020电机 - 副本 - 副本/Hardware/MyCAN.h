#ifndef __MYCAN_H
#define __MYCAN_H

#include "stm32f10x.h"

/* 电压范围 */
#define VOLTAGE_MAX        25000
#define VOLTAGE_MIN        -25000

/* 电机状态结构体 */
typedef struct {
    uint16_t mechanical_angle;
    int16_t  speed_rpm;
    float    actual_current;
    uint8_t  temperature;
    uint8_t  is_connected;
} GM6020_StatusTypeDef;

extern GM6020_StatusTypeDef gm6020_status[8];
extern float gm6020_total_angle[8];

/* 函数声明 */
void GM6020_CAN_Init(void);
void GM6020_Set_Voltage(uint8_t id, int32_t voltage);      // 新增
void GM6020_Send_All_Voltage(void);                         // 新增
void GM6020_Send_Voltage_ByID(uint8_t id, int32_t voltage); // 保留兼容
void GM6020_Parse_Feedback(CanRxMsg* rx_msg);

#endif

