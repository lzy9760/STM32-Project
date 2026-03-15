#ifndef MODULE_PID_H
#define MODULE_PID_H

typedef struct
{
  float kp;
  float ki;
  float kd;

  float integral;
  float prev_error;

  float out_limit;
  float integral_limit;
} ModulePid_t;

void Module_PID_Init(ModulePid_t *pid,
                     float kp,
                     float ki,
                     float kd,
                     float out_limit,
                     float integral_limit);
void Module_PID_Reset(ModulePid_t *pid);
float Module_PID_Update(ModulePid_t *pid, float error, float dt_s);

#endif /* MODULE_PID_H */
