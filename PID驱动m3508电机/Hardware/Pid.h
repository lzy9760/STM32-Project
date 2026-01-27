
//Pid.h
#ifndef __PID_H
#define __PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float ref;

    float err;
    float err_last;

    float integral;

    float out;
    float out_max;
    float integral_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_max, float integral_max);

float PID_Calc(PID_t *pid, float ref, float set);

#endif
