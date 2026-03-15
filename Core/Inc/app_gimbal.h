#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#include "bsp_imu.h"
#include "bsp_remote.h"
#include "bsp_vision_link.h"
#include <stdint.h>

void APP_Gimbal_ControlStep(const BSP_RemoteState_t *remote_state,
                            const BSP_VisionTarget_t *vision_target,
                            uint8_t has_target,
                            const BSP_ImuSample_t *imu_sample,
                            float dt_s);

#endif /* APP_GIMBAL_H */
