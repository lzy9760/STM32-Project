#include "pid.h"
#include "math.h"
//pid.c
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_max, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_max = out_max;
    pid->integral_max = integral_max;
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;
}

void PID_Reset(PID_t *pid)
{
    pid->err = 0;
    pid->err_last = 0;
    pid->integral = 0;
    pid->output = 0;
}

float PID_Calc(PID_t *pid, float target, float feedback)
{
    pid->target = target;
    pid->feedback = feedback;
    pid->err = target - feedback;

    /* 积分分离：误差较小时才积分 */
    if (fabsf(pid->err) < pid->out_max * 0.5f)
    {
        pid->integral += pid->err;
    }

    /* 积分限幅 */
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    /* PID计算 */
    pid->output = pid->kp * pid->err +
                  pid->ki * pid->integral +
                  pid->kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;

    /* 输出限幅 */
    if (pid->output > pid->out_max)
        pid->output = pid->out_max;
    if (pid->output < -pid->out_max)
        pid->output = -pid->out_max;

    return pid->output;
}
