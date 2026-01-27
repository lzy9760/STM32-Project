#ifndef __MyCAN_H
#define __MyCAN_H

#include "stm32f10x.h"
#include <stdint.h>

// 电机信息结构体（与原代码一致，存储GM6020反馈数据）
typedef struct {
    uint16_t rotor_angle;    // 机械角度（0~8191，对应文档协议）
    int16_t  rotor_speed;    // 转速（单位：rpm，有正负）
    int16_t  torque_current; // 实际转矩电流（有正负）
    uint8_t  temp;           // 电机温度（单位：℃）
} moto_info_t;

// 全局变量声明（与原代码一致）
extern moto_info_t motor_yaw_info;
extern uint16_t can_cnt;

// 函数声明
void can_filter_init(void);                  // CAN过滤器+外设初始化（含GPIO/中断）
void set_GM6020_motor_voltage(int16_t v1);   // GM6020电压控制指令发送
void CAN1_RX0_IRQHandler(void);              // CAN1 RX_FIFO0中断服务函数（解析反馈）

#endif // __BSP_CAN_H