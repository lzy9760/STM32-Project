#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define MOTOR_COUNT             4U

void MotorDriver_Init(void);
void MotorDriver_SetDuty(int16_t duty_percent);
void MotorDriver_Stop(void);
int16_t MotorDriver_GetDuty(void);
void Motor_SetOutput(uint8_t motor_id, int16_t duty_percent);
void Motor_StopAll(void);
int16_t Motor_GetDuty(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
