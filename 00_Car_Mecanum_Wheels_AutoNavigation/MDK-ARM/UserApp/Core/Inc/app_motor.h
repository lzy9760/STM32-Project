#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

void Motor_StartAllPwm(void);
void Motor_SetOutput(uint8_t motor_id, int32_t pwm);
void Motor_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MOTOR_H */
