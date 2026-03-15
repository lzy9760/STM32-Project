#include "module_pid.h"

static float Module_PID_Clamp(float value, float min_value, float max_value)
{//限制PID输出在指定范围内
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

void Module_PID_Init(ModulePid_t *pid,
                     float kp,
                     float ki,
                     float kd,
                     float out_limit,
                     float integral_limit)
{//初始化PID控制器
  if (pid == 0)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;

  pid->out_limit = (out_limit > 0.0f) ? out_limit : 1.0f;
  pid->integral_limit = (integral_limit > 0.0f) ? integral_limit : 1.0f;
}

void Module_PID_Reset(ModulePid_t *pid)
{//重置PID控制器
  if (pid == 0)
  {
    return;
  }

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}

float Module_PID_Update(ModulePid_t *pid, float error, float dt_s)
{//更新PID控制器输出
  float derivative;
  float integral_next;
  float output_unsat;
  float output;

  if (pid == 0 || dt_s <= 0.0f)
  {
    return 0.0f;
  }

  derivative = (error - pid->prev_error) / dt_s; //计算导数项
  pid->prev_error = error; //更新前一个误差项

  integral_next = pid->integral + error * dt_s; //计算积分项
  integral_next = Module_PID_Clamp(integral_next,
                                   -pid->integral_limit,
                                   pid->integral_limit); //限制积分项在指定范围内

  output_unsat = pid->kp * error + pid->ki * integral_next + pid->kd * derivative; //计算未限制输出

  /* Basic anti-windup: freeze integration while saturating in the same direction. */
  if ((output_unsat > pid->out_limit && error > 0.0f) ||
      (output_unsat < -pid->out_limit && error < 0.0f))
  {
    integral_next = pid->integral;//如果输出饱和，冻结积分项
    output_unsat = pid->kp * error + pid->ki * integral_next + pid->kd * derivative; //计算未限制输出
  }

  pid->integral = integral_next; //更新积分项
  output = Module_PID_Clamp(output_unsat, -pid->out_limit, pid->out_limit);

  return output;
}
