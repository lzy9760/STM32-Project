#include "bsp_chassis_motor.h"

#include <math.h>

#define BSP_DEG2RAD      0.01745329252f
#define BSP_RADPS_TO_RPM 9.54929658f

/*
 * Omni-wheel model using projection angles:
 * wheel_i_radps = (-sin(theta_i) * vx + cos(theta_i) * vy + wz * R) / r
 *
 * theta_i: wheel projection angle, measured from +X axis, CCW positive.
 * t_i = [-sin(theta_i), cos(theta_i)] is the wheel traction direction.
 */
static struct
{
  float wheel_radius_m;//轮子半径，单位：米
  float chassis_radius_m;//底盘半径，单位：米

  float vx_cmd_mps;//x轴速度指令，单位：米/秒
  float vy_cmd_mps;//y轴速度指令，单位：米/秒
  float wz_cmd_radps;//z轴角速度指令，单位：弧度/秒

  int8_t wheel_dir[BSP_CHASSIS_WHEEL_COUNT];//轮子方向，1为正转，-1为反转
  float wheel_theta_rad[BSP_CHASSIS_WHEEL_COUNT];//轮子投影角度，单位：弧度
  float wheel_theta_sin[BSP_CHASSIS_WHEEL_COUNT];//轮子投影角度的正弦值
  float wheel_theta_cos[BSP_CHASSIS_WHEEL_COUNT];//轮子投影角度的余弦值

  uint8_t output_enable;//输出使能开关，1为使能，0为禁用
  float wheel_rpm_limit_abs;//轮速绝对值上限，单位：转/分钟
  float power_proxy_coeff_w_per_rpm;//功率估计系数，单位：瓦特/转/分钟
  float power_proxy_w;//估计功率，单位：瓦特

  float wheel_rpm[BSP_CHASSIS_WHEEL_COUNT];//当前轮子转速，单位：转/分钟
  float wheel_target_rpm[BSP_CHASSIS_WHEEL_COUNT];//目标轮子转速，单位：转/分钟
} s_chassis;

static float BSP_Chassis_Abs(float value)
{//辅助函数，返回value的绝对值
  return (value >= 0.0f) ? value : -value;
}

static float BSP_Chassis_Clamp(float value, float min_value, float max_value)
{//辅助函数，将value限制在[min_value, max_value]范围内
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float BSP_Chassis_WrapDeg(float deg)
{//辅助函数，将deg归一化到[0, 360)范围内
  while (deg >= 360.0f)
  {
    deg -= 360.0f;
  }

  while (deg < 0.0f)
  {
    deg += 360.0f;
  }

  return deg;
}

static int8_t BSP_Chassis_NormalizeDir(int8_t dir)
{//辅助函数，将dir归一化到{1, -1}范围内
  return (dir >= 0) ? 1 : -1;
}

static void BSP_Chassis_SetOneAngle(uint8_t id, float deg)
{//辅助函数，设置轮子id的投影角度为deg
  float rad;

  if (id >= BSP_CHASSIS_WHEEL_COUNT)
  {
    return;
  }

  deg = BSP_Chassis_WrapDeg(deg);
  rad = deg * BSP_DEG2RAD;

  s_chassis.wheel_theta_rad[id] = rad;
  s_chassis.wheel_theta_sin[id] = sinf(rad);
  s_chassis.wheel_theta_cos[id] = cosf(rad);
}

void BSP_ChassisMotor_Init(float wheel_radius_m, float chassis_radius_m)
{//初始化底盘电机
  uint8_t i;

  s_chassis.wheel_radius_m = (wheel_radius_m > 0.001f) ? wheel_radius_m : 0.0815f;
  s_chassis.chassis_radius_m = (chassis_radius_m > 0.01f) ? chassis_radius_m : 0.2125f;
  s_chassis.vx_cmd_mps = 0.0f;
  s_chassis.vy_cmd_mps = 0.0f;
  s_chassis.wz_cmd_radps = 0.0f;

  s_chassis.output_enable = 1U;
  s_chassis.wheel_rpm_limit_abs = 6000.0f;
  s_chassis.power_proxy_coeff_w_per_rpm = 0.01f;
  s_chassis.power_proxy_w = 0.0f;

  for (i = 0U; i < BSP_CHASSIS_WHEEL_COUNT; i++)
  {
    s_chassis.wheel_dir[i] = 1;
    s_chassis.wheel_rpm[i] = 0.0f;
    s_chassis.wheel_target_rpm[i] = 0.0f;
  }

  /* Default to cross-layout omni chassis: front/right/rear/left = 0/270/180/90 deg. */
  BSP_ChassisMotor_SetWheelProjectAngleDeg(0.0f, 270.0f, 180.0f, 90.0f);
}

void BSP_ChassisMotor_SetWheelDirection(int8_t motor0_dir,
                                        int8_t motor1_dir,
                                        int8_t motor2_dir,
                                        int8_t motor3_dir)
{//设置轮子方向
  s_chassis.wheel_dir[BSP_CHASSIS_WHEEL_MOTOR0] = BSP_Chassis_NormalizeDir(motor0_dir);
  s_chassis.wheel_dir[BSP_CHASSIS_WHEEL_MOTOR1] = BSP_Chassis_NormalizeDir(motor1_dir);
  s_chassis.wheel_dir[BSP_CHASSIS_WHEEL_MOTOR2] = BSP_Chassis_NormalizeDir(motor2_dir);
  s_chassis.wheel_dir[BSP_CHASSIS_WHEEL_MOTOR3] = BSP_Chassis_NormalizeDir(motor3_dir);
}

void BSP_ChassisMotor_SetWheelProjectAngleDeg(float motor0_deg,
                                              float motor1_deg,
                                              float motor2_deg,
                                              float motor3_deg)
{//设置轮子投影角度
  BSP_Chassis_SetOneAngle(BSP_CHASSIS_WHEEL_MOTOR0, motor0_deg);
  BSP_Chassis_SetOneAngle(BSP_CHASSIS_WHEEL_MOTOR1, motor1_deg);
  BSP_Chassis_SetOneAngle(BSP_CHASSIS_WHEEL_MOTOR2, motor2_deg);
  BSP_Chassis_SetOneAngle(BSP_CHASSIS_WHEEL_MOTOR3, motor3_deg);
}

void BSP_ChassisMotor_SetOutputEnable(uint8_t enable)
{//设置输出使能开关
  s_chassis.output_enable = (enable != 0U) ? 1U : 0U;
}

void BSP_ChassisMotor_SetWheelRpmLimit(float rpm_limit_abs)
{//设置轮速绝对值上限
  if (rpm_limit_abs < 500.0f)
  {
    rpm_limit_abs = 500.0f;
  }
  if (rpm_limit_abs > 12000.0f)
  {
    rpm_limit_abs = 12000.0f;
  }

  s_chassis.wheel_rpm_limit_abs = rpm_limit_abs;
}

float BSP_ChassisMotor_GetEstimatedPowerW(void)
{//获取估计功率
  return s_chassis.power_proxy_w;
}

void BSP_ChassisMotor_SetCommand(float vx_mps, float vy_mps, float wz_radps)
{//设置底盘运动指令
  s_chassis.vx_cmd_mps = BSP_Chassis_Clamp(vx_mps, -3.0f, 3.0f);
  s_chassis.vy_cmd_mps = BSP_Chassis_Clamp(vy_mps, -3.0f, 3.0f);
  s_chassis.wz_cmd_radps = BSP_Chassis_Clamp(wz_radps, -6.5f, 6.5f);
}

void BSP_ChassisMotor_Update(float dt_s)
{//更新底盘电机状态（核心函数），根据指令和时间步dt_s计算轮速目标并更新轮速
  const float response = 18.0f;
  float wheel_radps_target[BSP_CHASSIS_WHEEL_COUNT];
  float v_project;
  float target_rpm;
  float rpm_sum_abs;
  uint8_t i;

  if (dt_s <= 0.0f)
  {
    return;
  }

  rpm_sum_abs = 0.0f;

  for (i = 0U; i < BSP_CHASSIS_WHEEL_COUNT; i++)
  {
    if (s_chassis.output_enable == 0U)
    {
      s_chassis.wheel_target_rpm[i] = 0.0f;
    }
    else
    {
      v_project = -s_chassis.wheel_theta_sin[i] * s_chassis.vx_cmd_mps
                +  s_chassis.wheel_theta_cos[i] * s_chassis.vy_cmd_mps;

      wheel_radps_target[i] = (v_project + s_chassis.chassis_radius_m * s_chassis.wz_cmd_radps)
                            / s_chassis.wheel_radius_m;

      target_rpm = wheel_radps_target[i] * BSP_RADPS_TO_RPM * (float)s_chassis.wheel_dir[i];
      target_rpm = BSP_Chassis_Clamp(target_rpm,
                                     -s_chassis.wheel_rpm_limit_abs,
                                     s_chassis.wheel_rpm_limit_abs);
      s_chassis.wheel_target_rpm[i] = target_rpm;
    }

    rpm_sum_abs += BSP_Chassis_Abs(s_chassis.wheel_target_rpm[i]);
    s_chassis.wheel_rpm[i] += (s_chassis.wheel_target_rpm[i] - s_chassis.wheel_rpm[i]) * response * dt_s;
  }

  s_chassis.power_proxy_w = rpm_sum_abs * s_chassis.power_proxy_coeff_w_per_rpm;
}

void BSP_ChassisMotor_GetFeedback(BSP_ChassisFeedback_t *feedback)
{
  uint8_t i;

  if (feedback == 0)
  {
    return;
  }

  for (i = 0U; i < BSP_CHASSIS_WHEEL_COUNT; i++)
  {
    feedback->wheel_rpm[i] = s_chassis.wheel_rpm[i];
  }
}

void BSP_ChassisMotor_GetTargetRpm(float target_rpm[BSP_CHASSIS_WHEEL_COUNT])
{
  uint8_t i;

  if (target_rpm == 0)
  {
    return;
  }

  for (i = 0U; i < BSP_CHASSIS_WHEEL_COUNT; i++)
  {
    target_rpm[i] = s_chassis.wheel_target_rpm[i];
  }
}
