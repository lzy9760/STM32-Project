#ifndef __MYCAN_H
#define __MYCAN_H
//MyCAN.h
#include "stm32f10x.h"

/* ==== 配置区 ==== */
#define GM6020_MOTOR_ID    1       /* 电机ID，根据拨码开关设置 */
//MyCAN.h
/* CAN协议相关定义 */
#define CAN_CMD_VOLTAGE   0x1FF      /* 控制ID1-4的帧标识符 */
#define CAN_CMD_VOLTAGE_5_7    0x2FF      /* 控制ID5-7的帧标识符 */
#define CAN_FEEDBACK_BASE      0x204      /* 反馈帧基址 */

/* 电压范围 */
#define VOLTAGE_MAX        25000
#define VOLTAGE_MIN        -25000

/* 电机状态结构体 */
typedef struct {
    uint16_t mechanical_angle;      /* 机械角度 0~8191 */
    int16_t  speed_rpm;             /* 转速 rpm */
    float    actual_current;        /* 实际电流 A */
    uint8_t  temperature;           /* 温度 °C */
    uint8_t  is_connected;          /* 连接状态 */
} GM6020_StatusTypeDef;

/* 全局变量声明 */
extern GM6020_StatusTypeDef gm6020_status;
extern float gm6020_total_angle;
extern uint16_t flag;
/* 函数声明 */
void GM6020_CAN_Init(void);
void GM6020_Send_Voltage(int32_t voltage);
void GM6020_Send_Voltage_Multi(uint8_t id, int32_t voltage);
void GM6020_Parse_Feedback(CanRxMsg* rx_msg);

#endif /* __MYCAN_H */

