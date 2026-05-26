#include "app_odometry.h"

#include "app_encoder.h"
#include "app_mpu6050.h"

#include <math.h>

#define ODOM_WHEEL_DIAMETER_MM        65.0f
#define ODOM_ENCODER_COUNTS_PER_REV   4320.0f
#define ODOM_FL_GAIN                  1.000f
#define ODOM_FR_GAIN                  1.000f
#define ODOM_RL_GAIN                  1.000f
#define ODOM_RR_GAIN                  1.000f
#define ODOM_FORWARD_SCALE            0.985f
#define ODOM_STRAFE_SCALE             0.960f
#define ODOM_PI                       3.14159265f

static Odometry_Pose_t g_pose;
static int32_t g_last_count[ENCODER_COUNT];

static float Odom_CosDeg(float deg)
{
  return cosf((deg * ODOM_PI) / 180.0f);
}

static float Odom_SinDeg(float deg)
{
  return sinf((deg * ODOM_PI) / 180.0f);
}

static float Odom_WrapDeg(float deg)
{
  while (deg > 180.0f)
  {
    deg -= 360.0f;
  }
  while (deg < -180.0f)
  {
    deg += 360.0f;
  }

  return deg;
}

static float Odom_CountToDistanceMm(int32_t count_delta)
{//将编码器计数转换为距离（单位：毫米）
  return ((float)count_delta * ODOM_PI * ODOM_WHEEL_DIAMETER_MM)/
         ODOM_ENCODER_COUNTS_PER_REV;
}

void Odometry_Init(void)
{
  uint8_t i;

  g_pose.x_mm = 0.0f;
  g_pose.y_mm = 0.0f;
  g_pose.yaw_deg = MPU6050_GetYaw();

  for (i = 0U; i < ENCODER_COUNT; i++)
  {
    g_last_count[i] = Encoder_GetCount(i);
  }
}

void Odometry_Reset(float x_mm, float y_mm, float yaw_deg)
{
  uint8_t i;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  g_pose.x_mm = x_mm;
  g_pose.y_mm = y_mm;
  g_pose.yaw_deg = yaw_deg;
  for (i = 0U; i < ENCODER_COUNT; i++)
  {
    g_last_count[i] = Encoder_GetCount(i);
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Odometry_Update(void)
{//通过编码器计数和陀螺仪数据更新里程计位姿
  int32_t count[ENCODER_COUNT];
  float d_fl;
  float d_fr;
  float d_rl;
  float d_rr;
  float body_forward_mm;
  float body_strafe_mm;
  float last_yaw_deg;
  float yaw_deg;
  float yaw_mid_deg;
  float yaw_delta_deg;
  float cos_yaw;
  float sin_yaw;
  uint8_t i;
  uint32_t primask;

  for (i = 0U; i < ENCODER_COUNT; i++)
  {
    count[i] = Encoder_GetCount(i);
  }

  d_fl = Odom_CountToDistanceMm(count[ENCODER_U1] - g_last_count[ENCODER_U1]) * ODOM_FL_GAIN;//计算每个轮子的行驶距离，通过编码器计数差转换得到
  d_fr = Odom_CountToDistanceMm(count[ENCODER_U2] - g_last_count[ENCODER_U2]) * ODOM_FR_GAIN;
  d_rl = Odom_CountToDistanceMm(count[ENCODER_U3] - g_last_count[ENCODER_U3]) * ODOM_RL_GAIN;
  d_rr = Odom_CountToDistanceMm(count[ENCODER_U4] - g_last_count[ENCODER_U4]) * ODOM_RR_GAIN;

  for (i = 0U; i < ENCODER_COUNT; i++)
  {
    g_last_count[i] = count[i];
  }

  yaw_deg = MPU6050_GetYaw();
  body_forward_mm = ((d_fl + d_fr + d_rl + d_rr) * 0.25f) * ODOM_FORWARD_SCALE;
  body_strafe_mm = ((-d_fl + d_fr + d_rl - d_rr) * 0.25f) * ODOM_STRAFE_SCALE;

  primask = __get_PRIMASK();
  __disable_irq();
  last_yaw_deg = g_pose.yaw_deg;
  if (primask == 0U)
  {
    __enable_irq();
  }

  yaw_delta_deg = Odom_WrapDeg(yaw_deg - last_yaw_deg);
  yaw_mid_deg = Odom_WrapDeg(last_yaw_deg + (yaw_delta_deg * 0.5f));
  cos_yaw = Odom_CosDeg(yaw_mid_deg);
  sin_yaw = Odom_SinDeg(yaw_mid_deg);

  primask = __get_PRIMASK();
  __disable_irq();
  g_pose.x_mm += (body_forward_mm * cos_yaw) - (body_strafe_mm * sin_yaw);
  g_pose.y_mm += (body_forward_mm * sin_yaw) + (body_strafe_mm * cos_yaw);
  g_pose.yaw_deg = yaw_deg;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

Odometry_Pose_t Odometry_GetPose(void)
{
  Odometry_Pose_t pose;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  pose = g_pose;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return pose;
}
