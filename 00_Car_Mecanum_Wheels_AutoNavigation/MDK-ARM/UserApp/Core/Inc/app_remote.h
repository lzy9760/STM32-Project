#ifndef __APP_REMOTE_H
#define __APP_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

void Remote_Init(void);
void Remote_ControlTask(void);
uint8_t Remote_HasManualCommand(void);
uint8_t Remote_IsOnline(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_REMOTE_H */
