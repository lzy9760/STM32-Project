#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

/* Chassis geometry */
#define ROBOT_CHASSIS_WHEEL_RADIUS_M 0.0815f
#define ROBOT_CHASSIS_RADIUS_M       0.2125f

/* Cross-layout omni: motor0/front, motor1/right, motor2/rear, motor3/left */
#define ROBOT_CHASSIS_MOTOR0_DIR       (+1)
#define ROBOT_CHASSIS_MOTOR1_DIR       (+1)
#define ROBOT_CHASSIS_MOTOR2_DIR       (+1)
#define ROBOT_CHASSIS_MOTOR3_DIR       (+1)
#define ROBOT_CHASSIS_MOTOR0_THETA_DEG 0.0f
#define ROBOT_CHASSIS_MOTOR1_THETA_DEG 270.0f
#define ROBOT_CHASSIS_MOTOR2_THETA_DEG 180.0f
#define ROBOT_CHASSIS_MOTOR3_THETA_DEG 90.0f

/* Gimbal software limits */
#define ROBOT_GIMBAL_YAW_MIN_DEG   (-145.0f)
#define ROBOT_GIMBAL_YAW_MAX_DEG   (145.0f)
#define ROBOT_GIMBAL_PITCH_MIN_DEG (-20.0f)
#define ROBOT_GIMBAL_PITCH_MAX_DEG (30.0f)

/* CAN output default enable */
#define ROBOT_CAN_CHASSIS_TX_ENABLE 1U
#define ROBOT_CAN_GIMBAL_TX_ENABLE  1U

/* CAN feedback IDs (DJI motor default mapping) */
#define ROBOT_CAN_ID_CHASSIS_MOTOR0_FB 0x201U
#define ROBOT_CAN_ID_CHASSIS_MOTOR1_FB 0x202U
#define ROBOT_CAN_ID_CHASSIS_MOTOR2_FB 0x203U
#define ROBOT_CAN_ID_CHASSIS_MOTOR3_FB 0x204U
#define ROBOT_CAN_ID_GIMBAL_YAW_FB     0x205U
#define ROBOT_CAN_ID_GIMBAL_PITCH_FB   0x206U

/* Gimbal encoder parameters */
#define ROBOT_GIMBAL_ENCODER_RESOLUTION 8192.0f//云台编码器分辨率，用于将编码器计数转换为角度
#define ROBOT_GIMBAL_YAW_ECD_OFFSET     0U//云台YAW轴编码器偏移量，用于校准YAW轴角度
#define ROBOT_GIMBAL_PITCH_ECD_OFFSET   0U//云台PITCH轴编码器偏移量，用于校准PITCH轴角度
#define ROBOT_GIMBAL_YAW_GEAR_RATIO     1.0f//云台YAW轴齿轮比，用于计算YAW轴角度
#define ROBOT_GIMBAL_PITCH_GEAR_RATIO   1.0f//云台PITCH轴齿轮比，用于计算PITCH轴角度
#define ROBOT_GIMBAL_FB_TIMEOUT_MS      60U//云台反馈超时时间，单位为毫秒
#define ROBOT_CHASSIS_FB_TIMEOUT_MS     60U//底盘反馈超时时间，单位为毫秒

#endif /* ROBOT_PARAM_H */
