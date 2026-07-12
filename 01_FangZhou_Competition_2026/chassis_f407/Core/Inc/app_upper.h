#ifndef APP_UPPER_H
#define APP_UPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define APP_UPPER_FLAG_LIFT_UP        0x01U
#define APP_UPPER_FLAG_LIFT_DOWN      0x02U
#define APP_UPPER_FLAG_SERVO_75       0x04U
#define APP_UPPER_FLAG_SERVO_25       0x08U
#define APP_UPPER_FLAG_MASK           (APP_UPPER_FLAG_LIFT_UP | APP_UPPER_FLAG_LIFT_DOWN | \
                                       APP_UPPER_FLAG_SERVO_75 | APP_UPPER_FLAG_SERVO_25)

void AppUpper_Init(void);
void AppUpper_Task(uint8_t remote_flags);
void AppUpper_TIM7_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UPPER_H */
