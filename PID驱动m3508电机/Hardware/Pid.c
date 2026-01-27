#include "Pid.h"
//Pid.c
void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_max, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_max = out_max;
    pid->integral_max = integral_max;

    pid->integral = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->out = 0;
}

float PID_Calc(PID_t *pid, float ref, float set)  // 注意：形参写反了，应该是(set, ref)
{
    // 1. 把传入的“目标值”和“实际值”存到PID结构体里
    pid->ref = ref;    // 实际反馈值（比如电机实际转速2800rpm）
    pid->set = set;    // 目标值（比如电机目标转速3000rpm）

    // 2. 计算当前偏差：目标值 - 实际值（偏差为正，说明实际值不够，需要加大输出）
    pid->err = set - ref;  // 比如3000-2800=200rpm，偏差200

    // 3. 积分项：累积历史偏差（消除静态误差，比如电机低速时的小偏差）
    pid->integral += pid->err;  // 把当前偏差加到积分里（越久没到目标，积分越大）
    // 积分限幅：防止积分累积太多（积分饱和），导致电机突然冲速
    if (pid->integral > pid->integral_max)  pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    // 4. 微分项：计算偏差的变化率（看偏差是变大还是变小，抑制震荡）
    float derivative = pid->err - pid->err_last;  // 比如这次偏差200，上次150，变化率50

    // 5. 核心：PID公式计算最终输出（比例+积分+微分）
    pid->out = pid->kp * pid->err        // 比例项：偏差200 × kp=8 → 1600
             + pid->ki * pid->integral   // 积分项：累积偏差 × ki=0.1 → 补偿小偏差
             + pid->kd * derivative;     // 微分项：变化率50 × kd=0.05 → 抑制冲速

    // 6. 输出限幅：防止输出超过电机/电调的承受范围（比如最大电流8000）
    if (pid->out > pid->out_max)  pid->out = pid->out_max;
    if (pid->out < -pid->out_max) pid->out = -pid->out_max;

    // 7. 保存当前偏差为“上一次偏差”（给下次计算微分用）
    pid->err_last = pid->err;

    // 8. 返回最终输出值（比如电流指令，传给CAN发送函数控制电机）
    return pid->out;
}
