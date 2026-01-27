#ifndef __GM6020_H
#define __GM6020_H
//GM6020.h
#include "stm32f10x.h"

/* 控制模式 */
typedef enum {
    CONTROL_MODE_SPEED = 0,     // 速度环模式
    CONTROL_MODE_ANGLE = 1      // 角度环模式（串级PID）
} ControlMode_e;

/* 函数声明 */
void Motor_Control_Init(void);
void Motor_Control_1ms(void);
void Motor_SetTargetAngle(float angle_deg);
void Motor_SetTargetSpeed(float speed_rpm);
void Motor_SetMode(ControlMode_e mode);

/* 外部变量 */
extern float target_angle;
extern float target_speed;

#endif
