#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "at32f403a_407_board.h"
#include "can.h"
#include "pid.h"

/* ========== 电机类型 ========== */
typedef enum {
    MOTOR_TYPE_GM6020,  
    MOTOR_TYPE_M3508,   
} MotorType;

/* ========== 电机结构体 ========== */
typedef struct Motor{
    
    MotorType   type;       // 电机类型
    can_type   *hcan;       // 挂在哪路 CAN 上
    MotorIndex  index;      // 在 CAN 帧中的位置 0~3

    uint16_t    angle;      // 机械角度 0~8191
    int16_t     rpm;        // 转速
    int16_t     current;    // 实际电流
    uint8_t     temp;       // 温度
    uint8_t     feedback_valid;

    float     target;     // 目标值

    pid_ctrl pid_speed;   // 转速环 PID
    pid_ctrl pid_pos;    // 位置环 PID
} Motor;
extern Motor motors[8];







void Motor_Init(Motor *motor, MotorType type, can_type *hcan, MotorIndex index);
void Motor_Nb_Receive(Motor *motor, uint8_t data[8]);

#endif
