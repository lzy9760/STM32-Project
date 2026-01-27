#ifndef __PID_H
#define __PID_H
//pid.h
typedef struct
{
    float kp;
    float ki;
    float kd;

    float target;
    float feedback;

    float err;
    float err_last;

    float integral;
    float output;

    float out_max;
    float integral_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_max, float integral_max);
float PID_Calc(PID_t *pid, float target, float feedback);
void PID_Reset(PID_t *pid); 

#endif
