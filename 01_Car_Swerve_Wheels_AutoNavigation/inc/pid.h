#ifndef __PID_H__
#define __PID_H__

#include "at32f403a_407_board.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float error;
    float last_error;
    float last_last_error;

    float output;
    float output_max;
    float output_min;
    float integral_max;
    float integral_min;

} pid_ctrl;

typedef struct Motor Motor;

void pid_init(pid_ctrl *pid, float kp, float ki, float kd, float output_max, float output_min, float integral_max, float integral_min);
void pid_all_init(void);
int16_t GM6020_cascade_control(Motor *motor, float target_angle); 
float pid_speed_calculate(pid_ctrl *pid, float target, float current);
float pid_pos_calculate(pid_ctrl *pid, float target, float current);


#endif
