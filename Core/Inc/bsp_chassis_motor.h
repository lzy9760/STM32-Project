#ifndef BSP_CHASSIS_MOTOR_H
#define BSP_CHASSIS_MOTOR_H

#include <stdint.h>

typedef enum
{
  BSP_CHASSIS_WHEEL_MOTOR0 = 0,
  BSP_CHASSIS_WHEEL_MOTOR1 = 1,
  BSP_CHASSIS_WHEEL_MOTOR2 = 2,
  BSP_CHASSIS_WHEEL_MOTOR3 = 3,

  /* Legacy aliases kept for compatibility. */
  BSP_CHASSIS_WHEEL_FRONT = BSP_CHASSIS_WHEEL_MOTOR0,
  BSP_CHASSIS_WHEEL_RIGHT = BSP_CHASSIS_WHEEL_MOTOR1,
  BSP_CHASSIS_WHEEL_REAR  = BSP_CHASSIS_WHEEL_MOTOR2,
  BSP_CHASSIS_WHEEL_LEFT  = BSP_CHASSIS_WHEEL_MOTOR3,

  BSP_CHASSIS_WHEEL_COUNT = 4
} BSP_ChassisWheelId_t;

typedef struct
{
  float wheel_rpm[BSP_CHASSIS_WHEEL_COUNT];
} BSP_ChassisFeedback_t;

void BSP_ChassisMotor_Init(float wheel_radius_m, float chassis_radius_m);

/* Motor direction sign, use +1/-1 according to wiring. */
void BSP_ChassisMotor_SetWheelDirection(int8_t motor0_dir,
                                        int8_t motor1_dir,
                                        int8_t motor2_dir,
                                        int8_t motor3_dir);

/*
 * Set wheel projection angles in degrees, measured from +X axis with CCW positive.
 * Wheel traction direction is t = [-sin(theta), cos(theta)].
 *
 * Common presets:
 * 1) Cross-layout omni chassis (front/right/rear/left wheel arrangement):
 *    motor0=0, motor1=270, motor2=180, motor3=90.
 * 2) Square-layout omni chassis (45 deg modules):
 *    motor0=45, motor1=135, motor2=225, motor3=315.
 */
void BSP_ChassisMotor_SetWheelProjectAngleDeg(float motor0_deg,
                                              float motor1_deg,
                                              float motor2_deg,
                                              float motor3_deg);

/* Force motor command output enable/disable. disable=soft power-off command at BSP layer. */
void BSP_ChassisMotor_SetOutputEnable(uint8_t enable);

/* CAN current frame output enable/disable. */
void BSP_ChassisMotor_SetCanTxEnable(uint8_t enable);

/* Absolute rpm limit for each wheel command. */
void BSP_ChassisMotor_SetWheelRpmLimit(float rpm_limit_abs);

/*
 * Estimated chassis power in watts (for control-layer guard fallback when referee data is absent).
 * This is only a proxy, not referee-measured power.
 */
float BSP_ChassisMotor_GetEstimatedPowerW(void);

void BSP_ChassisMotor_SetCommand(float vx_mps, float vy_mps, float wz_radps);
void BSP_ChassisMotor_Update(float dt_s);
void BSP_ChassisMotor_GetFeedback(BSP_ChassisFeedback_t *feedback);
void BSP_ChassisMotor_GetTargetRpm(float target_rpm[BSP_CHASSIS_WHEEL_COUNT]);
void BSP_ChassisMotor_GetCanCurrentCmd(int16_t out_current[BSP_CHASSIS_WHEEL_COUNT]);
void BSP_ChassisMotor_OnCanFeedback(uint16_t std_id, const uint8_t *data, uint8_t dlc);

#endif /* BSP_CHASSIS_MOTOR_H */

