#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "bsp_remote.h"

void APP_Chassis_ControlStep(const BSP_RemoteState_t *remote_state, float dt_s);

#endif /* APP_CHASSIS_H */
