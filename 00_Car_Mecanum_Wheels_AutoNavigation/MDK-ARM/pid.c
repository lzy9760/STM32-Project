#include "pid.h"

#include "app_encoder.h"
#include "app_motor.h"

#define PID_TARGET_SPEED_MAX     100.0f
#define PID_BASE_PWM_PERCENT     100.0f
#define PID_DEFAULT_KP           400.0f
#define PID_DEFAULT_KI           5.0f
#define PID_DEFAULT_KD           0.0f
#define PID_SUM_ERROR_MAX        300.0f

typedef struct
{
  float kp;
  float ki;
  float kd;
  int32_t target_pwm;
  int32_t now_speed;
  int32_t last_count;
  int32_t output_pwm;
  float sum_error;
  float last_error;
} PidMotor_t;

static PidMotor_t g_pid_motors[PID_MOTOR_COUNT];//PID电机结构体数组
static uint32_t g_pid_last_tick;

static float Pid_ClampFloat(float value, float min_value, float max_value)
{
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

static int32_t Pid_ClampPwm(int32_t value)
{
  if (value > PID_PWM_MAX)
  {
    return PID_PWM_MAX;
  }
  if (value < -PID_PWM_MAX)
  {
    return -PID_PWM_MAX;
  }

  return value;
}

static void Pid_ClearOne(uint8_t motor_id)
{
  g_pid_motors[motor_id].sum_error = 0.0f;
  g_pid_motors[motor_id].last_error = 0.0f;
  g_pid_motors[motor_id].now_speed = 0;
  g_pid_motors[motor_id].output_pwm = 0;
  g_pid_motors[motor_id].last_count = Encoder_GetCount(motor_id);
}

void Pid_InitAll(void)
{
  uint8_t motor_id;

  Encoder_ResetAll();
  for (motor_id = 0U; motor_id < PID_MOTOR_COUNT; motor_id++)
  {
    g_pid_motors[motor_id].kp = PID_DEFAULT_KP;
    g_pid_motors[motor_id].ki = PID_DEFAULT_KI;
    g_pid_motors[motor_id].kd = PID_DEFAULT_KD;
    g_pid_motors[motor_id].target_pwm = 0;
    Pid_ClearOne(motor_id);
  }
  g_pid_last_tick = HAL_GetTick();
}

void Pid_ClearAll(void)
{
  uint8_t motor_id;

  for (motor_id = 0U; motor_id < PID_MOTOR_COUNT; motor_id++)
  {
    Pid_ClearOne(motor_id);
  }
}

void Pid_SetTarget(uint8_t motor_id, int32_t target_pwm)
{
  if (motor_id >= PID_MOTOR_COUNT)
  {
    return;
  }

  target_pwm = Pid_ClampPwm(target_pwm);
  g_pid_motors[motor_id].target_pwm = target_pwm;
  if (target_pwm == 0)
  {
    Pid_ClearOne(motor_id);
  }
}

void Pid_SetTargetAll(int32_t front_left,
                      int32_t front_right,
                      int32_t rear_left,
                      int32_t rear_right)
{
  Pid_SetTarget(0U, front_left);
  Pid_SetTarget(1U, front_right);
  Pid_SetTarget(2U, rear_left);
  Pid_SetTarget(3U, rear_right);
}

void Pid_StopAll(void)
{
  uint8_t motor_id;

  for (motor_id = 0U; motor_id < PID_MOTOR_COUNT; motor_id++)
  {
    g_pid_motors[motor_id].target_pwm = 0;
    Pid_ClearOne(motor_id);
  }
  Motor_StopAll();
}

void Pid_UpdateAll(void)
{
  uint8_t motor_id;
  uint32_t now_tick;

  now_tick = HAL_GetTick();
  if ((now_tick - g_pid_last_tick) < PID_RUN_PERIOD_MS)
  {
    return;
  }
  g_pid_last_tick = now_tick;

  Encoder_Scan();
  for (motor_id = 0U; motor_id < PID_MOTOR_COUNT; motor_id++)
  {
    PidMotor_t *pid;
    int32_t now_count;
    float target_speed;
    float error;
    float change_error;
    float base_pwm;
    float output;

    pid = &g_pid_motors[motor_id];
    now_count = Encoder_GetCount(motor_id);
    pid->now_speed = now_count - pid->last_count;//计算当前速度，通过编码器计数差得到
    pid->last_count = now_count;

    if (pid->target_pwm == 0)
    {
      Pid_ClearOne(motor_id);
      Motor_SetOutput(motor_id, 0);
      continue;
    }

    target_speed = ((float)pid->target_pwm * PID_TARGET_SPEED_MAX) / (float)PID_PWM_MAX;
    error = target_speed - (float)pid->now_speed;
    pid->sum_error = Pid_ClampFloat(pid->sum_error + error, -PID_SUM_ERROR_MAX, PID_SUM_ERROR_MAX);
    change_error = error - pid->last_error;
    pid->last_error = error;

    base_pwm = ((float)pid->target_pwm * PID_BASE_PWM_PERCENT) / 100.0f;
    output = base_pwm +
             (pid->kp * error) +
             (pid->ki * pid->sum_error) +
             (pid->kd * change_error);

    if (output >= 0.0f)
    {
      pid->output_pwm = Pid_ClampPwm((int32_t)(output + 0.5f));
    }
    else
    {
      pid->output_pwm = Pid_ClampPwm((int32_t)(output - 0.5f));
    }

    Motor_SetOutput(motor_id, pid->output_pwm);
  }
}

int32_t Pid_GetTarget(uint8_t motor_id)
{
  if (motor_id >= PID_MOTOR_COUNT)
  {
    return 0;
  }

  return g_pid_motors[motor_id].target_pwm;
}

int32_t Pid_GetSpeed(uint8_t motor_id)
{
  if (motor_id >= PID_MOTOR_COUNT)
  {
    return 0;
  }

  return g_pid_motors[motor_id].now_speed;
}

int32_t Pid_GetOutput(uint8_t motor_id)
{
  if (motor_id >= PID_MOTOR_COUNT)
  {
    return 0;
  }

  return g_pid_motors[motor_id].output_pwm;
}
