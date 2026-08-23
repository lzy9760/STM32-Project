#ifndef __CAN_H__
#define __CAN_H__

#include "at32f403a_407_board.h"

/* ========== 控制帧 ID（一发四）========== */
#define CAN_CONTROL_ID_GM6020    0x1FF
#define CAN_CONTROL_ID_M3508     0x200

/* ========== 反馈帧 ID 基址 ========== */
#define CAN_FEEDBACK_ID_GM6020   0x205  // 0x205 + motor_id (0~3)
#define CAN_FEEDBACK_ID_M3508    0x201  // 0x201 + motor_id (0~3)

/* ========== 电机编号 ========== */
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3
} MotorIndex;

/* ========== 函数声明 ========== */
void CAN_Init(void);
void CAN_Nb_Send(can_type *can, uint32_t id, int16_t m0, int16_t m1, int16_t m2, int16_t m3);
void Motor_CAN_Nb_Receive(can_type *can_x, can_rx_message_type *rx_msg);
void CAN_Nb_Receive(uint8_t data[8], uint16_t *angle, int16_t *rpm, int16_t *current, uint8_t *temp);
#endif
