#include "pid.h"
#include "motor.h"

#define pid_speed_GM6020_kp 60.0f
#define pid_speed_GM6020_ki 0.6f
#define pid_speed_GM6020_kd 0.0f

#define pid_speed_M3508_kp 6.8f
#define pid_speed_M3508_ki 0.2f
#define pid_speed_M3508_kd 0.0f

#define pid_pos_kp 40.0f
#define pid_pos_ki 0.0f
#define pid_pos_kd 0.0f

void pid_init(pid_ctrl *pid, float kp, float ki, float kd, float output_max, float output_min, float integral_max, float integral_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;

    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;

    pid->output = 0.0f;
}

void pid_all_init(void)
{
    /*GM6020*/
    pid_init(&motors[0].pid_speed, pid_speed_GM6020_kp, pid_speed_GM6020_ki, pid_speed_GM6020_kd, 15000.0f, -15000.0f, 20000.0f, -20000.0f);
    pid_init(&motors[0].pid_pos, pid_pos_kp, pid_pos_ki, pid_pos_kd, 1200.0f, -1200.0f, 20000.0f, -20000.0f);
    pid_init(&motors[1].pid_speed, pid_speed_GM6020_kp, pid_speed_GM6020_ki, pid_speed_GM6020_kd, 15000.0f, -15000.0f, 20000.0f, -20000.0f);
    pid_init(&motors[1].pid_pos, pid_pos_kp, pid_pos_ki, pid_pos_kd, 1200.0f, -1200.0f, 20000.0f, -20000.0f);
    pid_init(&motors[2].pid_speed, pid_speed_GM6020_kp, pid_speed_GM6020_ki, pid_speed_GM6020_kd, 15000.0f, -15000.0f, 20000.0f, -20000.0f);
    pid_init(&motors[2].pid_pos, pid_pos_kp, pid_pos_ki, pid_pos_kd, 1200.0f, -1200.0f, 20000.0f, -20000.0f);
    pid_init(&motors[3].pid_speed, pid_speed_GM6020_kp, pid_speed_GM6020_ki, pid_speed_GM6020_kd, 15000.0f, -15000.0f, 20000.0f, -20000.0f);
    pid_init(&motors[3].pid_pos, pid_pos_kp, pid_pos_ki, pid_pos_kd, 1200.0f, -1200.0f, 20000.0f, -20000.0f);

    /*M3508*/
    pid_init(&motors[4].pid_speed, pid_speed_M3508_kp, pid_speed_M3508_ki, pid_speed_M3508_kd, 3000.0f, -3000.0f, 20000.0f, -20000.0f);
    pid_init(&motors[5].pid_speed, pid_speed_M3508_kp, pid_speed_M3508_ki, pid_speed_M3508_kd, 3000.0f, -3000.0f, 20000.0f, -20000.0f);
    pid_init(&motors[6].pid_speed, pid_speed_M3508_kp, pid_speed_M3508_ki, pid_speed_M3508_kd, 3000.0f, -3000.0f, 20000.0f, -20000.0f);
    pid_init(&motors[7].pid_speed, pid_speed_M3508_kp, pid_speed_M3508_ki, pid_speed_M3508_kd, 3000.0f, -3000.0f, 20000.0f, -20000.0f);
}


float pid_speed_calculate(pid_ctrl *pid, float target, float current)
{
    pid->error = target - current;

    // 增量式pid
    pid->output += (pid->kp * (pid->error- pid->last_error))  + (pid->ki * pid->error) + (pid->kd * (pid->error - 2 * pid->last_error + pid->last_last_error));

    // 输出限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;

    // 更新误差历史
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

float pid_pos_calculate(pid_ctrl *pid, float target, float current)
{
    pid->error = target - current;

    if(pid->error > 180.0f)
    {
        pid->error -= 360.0f;
    }
    else if(pid->error < -180.0f)
    {
        pid->error += 360.0f;
    }

    pid->integral += pid->error;

    if(pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < pid->integral_min)
        pid->integral = pid->integral_min;

    //位置pid
    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * (pid->error - pid->last_error));
    
    // 输出限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;


    // 更新误差历史
    pid->last_error = pid->error;

    return pid->output;
}

int16_t GM6020_cascade_control(Motor *motor, float target_angle)
{
    float actual_angle = motor->angle / 8192.0f * 360.0f;
    float target_rpm;
    float output_voltage;

        // 位置环计算目标转速
    target_rpm = pid_pos_calculate(&motor->pid_pos, target_angle, actual_angle);

    // 转速环计算输出电压
    output_voltage = pid_speed_calculate(&motor->pid_speed, target_rpm, motor->rpm);

    return (int16_t)output_voltage;
}
