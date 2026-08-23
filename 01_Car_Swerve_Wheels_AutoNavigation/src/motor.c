#include "motor.h"


void Motor_Init(Motor *motor, MotorType type, can_type *hcan, MotorIndex index)
{
    motor->type    = type;
    motor->hcan    = hcan;
    motor->index   = index;

    motor->angle   = 0;
    motor->rpm     = 0;
    motor->current = 0;
    motor->temp    = 0;
    motor->feedback_valid = 0U;
    motor->target  = 0;
}


void Motor_Nb_Receive(Motor *motor, uint8_t data[8])
{
    CAN_Nb_Receive(data, &motor->angle, &motor->rpm, &motor->current, &motor->temp);
    motor->feedback_valid = 1U;
}
