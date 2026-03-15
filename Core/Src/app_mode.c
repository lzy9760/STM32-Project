#include "app_mode.h"

#include "bsp_board.h"
#include "bsp_can.h"
#include "bsp_chassis_motor.h"
#include "bsp_clock.h"
#include "bsp_gimbal_motor.h"
#include "bsp_imu.h"
#include "bsp_remote.h"
#include "bsp_vision_link.h"
#include "robot_param.h"

#include <string.h>

static AppRuntime_t s_runtime;//存储系统的所有运行时状态

static void APP_Mode_CanRxDispatch(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
  BSP_ChassisMotor_OnCanFeedback(std_id, data, dlc);//处理底盘电机的CAN反馈
  BSP_GimbalMotor_OnCanFeedback(std_id, data, dlc);//处理云台电机的CAN反馈
}

void APP_Mode_Bringup(void)//初始化应用模式
{
  memset(&s_runtime, 0, sizeof(s_runtime));//清空应用运行时状态结构体

  BSP_Clock_Init();//初始化时钟
  BSP_Board_InitFromManualV26();//初始化板级硬件

  s_runtime.can_online = (BSP_CAN_Init() == HAL_OK) ? 1U : 0U;//初始化CAN
  if (s_runtime.can_online != 0U)//如果CAN初始化成功
  {
    BSP_CAN_RegisterRxCallback(APP_Mode_CanRxDispatch);//注册CAN接收回调函数
  }

  BSP_Imu_Init();
  BSP_Remote_Init();
  BSP_Vision_Init();

  BSP_GimbalMotor_Init();
  BSP_GimbalMotor_SetCanTxEnable((uint8_t)(s_runtime.can_online && (ROBOT_CAN_GIMBAL_TX_ENABLE != 0U)));//设置CAN发送使能状态
  BSP_GimbalMotor_SetSoftLimit(ROBOT_GIMBAL_YAW_MIN_DEG,//设置云台软限位（yaw轴）
                               ROBOT_GIMBAL_YAW_MAX_DEG,//设置云台软限位（yaw轴）
                               ROBOT_GIMBAL_PITCH_MIN_DEG,//设置云台软限位（pitch轴）
                               ROBOT_GIMBAL_PITCH_MAX_DEG);//设置云台软限位（pitch轴）

  BSP_ChassisMotor_Init(ROBOT_CHASSIS_WHEEL_RADIUS_M, ROBOT_CHASSIS_RADIUS_M);//初始化底盘电机
  BSP_ChassisMotor_SetCanTxEnable((uint8_t)(s_runtime.can_online && (ROBOT_CAN_CHASSIS_TX_ENABLE != 0U)));
  BSP_ChassisMotor_SetWheelDirection(ROBOT_CHASSIS_MOTOR0_DIR,//设置底盘电机0的轮向
                                     ROBOT_CHASSIS_MOTOR1_DIR,//设置底盘电机1的轮向
                                     ROBOT_CHASSIS_MOTOR2_DIR,//设置底盘电机2的轮向
                                     ROBOT_CHASSIS_MOTOR3_DIR);//设置底盘电机3的轮向
  BSP_ChassisMotor_SetWheelProjectAngleDeg(ROBOT_CHASSIS_MOTOR0_THETA_DEG,//设置底盘电机0的轮向投影角度（度）
                                           ROBOT_CHASSIS_MOTOR1_THETA_DEG,//设置底盘电机1的轮向投影角度（度）
                                           ROBOT_CHASSIS_MOTOR2_THETA_DEG,//设置底盘电机2的轮向投影角度（度）
                                           ROBOT_CHASSIS_MOTOR3_THETA_DEG);//设置底盘电机3的轮向投影角度（度）

  Module_Attitude_Init(&s_runtime.attitude);
  Module_Gimbal_Init(&s_runtime.gimbal_ctrl);
  Module_Chassis_Init(&s_runtime.chassis_ctrl);
  Module_Chassis_SetRulePreset(&s_runtime.chassis_ctrl, MODULE_CHASSIS_RULE_3V3_INFANTRY);
  Module_Chassis_SetPowerMode(&s_runtime.chassis_ctrl, MODULE_CHASSIS_POWER_MODE_HP_FIRST);
}

AppRuntime_t *APP_Mode_Runtime(void)//获取应用运行时状态指针
{
  return &s_runtime;//获取应用运行时状态指针
}

uint8_t APP_Mode_IsControlEnabled(void)//检查控制是否已启用
{
  return (s_runtime.force_safe == 0U) ? 1U : 0U;//检查是否强制安全模式未启用
}

void APP_Mode_SetForceSafe(uint8_t force_safe)// 设置强制安全模式
{
  s_runtime.force_safe = (force_safe != 0U) ? 1U : 0U;
}

void APP_Mode_SetOnlineFlags(uint8_t imu_online, uint8_t vision_online, uint8_t gimbal_limited)
{//设置在线标志位，更新各模块在线状态
  s_runtime.imu_online = (imu_online != 0U) ? 1U : 0U;//设置IMU在线标志位
  s_runtime.vision_online = (vision_online != 0U) ? 1U : 0U;//设置视觉系统在线标志位
  s_runtime.gimbal_limited = (gimbal_limited != 0U) ? 1U : 0U;//设置云台是否受限标志位
}

void APP_Mode_IncControlOverrun(void)//增加控制循环溢出次数
{
  s_runtime.ctrl_loop_overrun++;
}

void APP_RemoteRxFrameHook(const uint8_t *frame, uint16_t len)//处理遥控器接收帧
{//处理遥控器接收到的完整帧，解码并更新遥控器状态，当USB/串口接收到完整帧时调用
  BSP_Remote_DecodeFrame(frame, len);
}

void APP_RemoteRxByteHook(uint8_t byte)//处理遥控器接收字节
{
  BSP_Remote_FeedByte(byte);//将接收到的字节添加到遥控器状态机中，用于解码和处理遥控器命令
}

void APP_VisionRxBytesHook(const uint8_t *data, uint16_t len)//处理视觉接收字节
{
  BSP_Vision_FeedRx(data, len);//将接收到的字节添加到视觉系统状态机中，用于解码和处理视觉数据
}
