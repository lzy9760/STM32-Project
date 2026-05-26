#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

#define PID_MOTOR_COUNT        4U
#define PID_RUN_PERIOD_MS      10U
#define PID_PWM_MAX            65535

void Pid_InitAll(void);
void Pid_ClearAll(void);
void Pid_SetTarget(uint8_t motor_id, int32_t target_pwm);
void Pid_SetTargetAll(int32_t front_left,
                      int32_t front_right,
                      int32_t rear_left,
                      int32_t rear_right);
void Pid_StopAll(void);
void Pid_UpdateAll(void);
int32_t Pid_GetTarget(uint8_t motor_id);
int32_t Pid_GetSpeed(uint8_t motor_id);
int32_t Pid_GetOutput(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
