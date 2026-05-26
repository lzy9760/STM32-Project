#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

void App_Init(void);
void App_Task(void);
uint8_t App_StartScheduler(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
