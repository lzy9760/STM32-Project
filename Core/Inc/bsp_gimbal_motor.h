#ifndef BSP_GIMBAL_MOTOR_H
#define BSP_GIMBAL_MOTOR_H

#include <stdint.h>

typedef struct
{
  float yaw_deg;
  float pitch_deg;
  float yaw_rate_dps;
  float pitch_rate_dps;
  uint8_t limit_triggered;
} BSP_GimbalFeedback_t;

void BSP_GimbalMotor_Init(void);
void BSP_GimbalMotor_SetSoftLimit(float yaw_min_deg,
                                  float yaw_max_deg,
                                  float pitch_min_deg,
                                  float pitch_max_deg);
void BSP_GimbalMotor_CommandCurrent(float yaw_current_cmd, float pitch_current_cmd);
void BSP_GimbalMotor_SetCanTxEnable(uint8_t enable);
void BSP_GimbalMotor_Update(float dt_s);
void BSP_GimbalMotor_GetFeedback(BSP_GimbalFeedback_t *feedback);
void BSP_GimbalMotor_GetCanCurrentCmd(int16_t *yaw_cmd, int16_t *pitch_cmd);
void BSP_GimbalMotor_OnCanFeedback(uint16_t std_id, const uint8_t *data, uint8_t dlc);
uint8_t BSP_GimbalMotor_IsLimited(void);

#endif /* BSP_GIMBAL_MOTOR_H */

