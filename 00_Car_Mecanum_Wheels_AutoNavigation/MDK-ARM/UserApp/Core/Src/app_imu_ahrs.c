#include "app_imu_ahrs.h"

#include <math.h>
#include <string.h>

#define IMU_AHRS_DEG2RAD              0.017453292519943295f
#define IMU_AHRS_RAD2DEG              57.29577951308232f

#define IMU_AHRS_KP_STATIC            4.0f
#define IMU_AHRS_KP_DYNAMIC           0.01f
#define IMU_AHRS_KI_STATIC            0.002f
#define IMU_AHRS_KI_DYNAMIC           0.0f
#define IMU_AHRS_INTEGRAL_MAX         0.05f
#define IMU_AHRS_INTEGRAL_MIN        -0.05f
#define IMU_AHRS_YAW_BIAS_MAX         0.1f
#define IMU_AHRS_YAW_BIAS_KP_SLOW     0.015f
#define IMU_AHRS_YAW_BIAS_KP_FAST     0.02f

static float ImuAhrs_Clamp(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

void ImuAhrs_MahonyInit(ImuAhrs_Mahony_t *ahrs)
{
  if (ahrs == NULL)
  {
    return;
  }

  memset(ahrs, 0, sizeof(ImuAhrs_Mahony_t));
  ahrs->q0 = 1.0f;
}

void ImuAhrs_MahonyInitWithAccel(ImuAhrs_Mahony_t *ahrs, float ax_g, float ay_g, float az_g)
{
  float norm;
  float roll;
  float pitch;
  float cr;
  float sr;
  float cp;
  float sp;

  if (ahrs == NULL)
  {
    return;
  }

  ImuAhrs_MahonyInit(ahrs);

  norm = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
  if (norm <= 0.0001f)
  {
    return;
  }

  ax_g /= norm;
  ay_g /= norm;
  az_g /= norm;

  roll = atan2f(ay_g, az_g);
  pitch = -asinf(ImuAhrs_Clamp(ax_g, -1.0f, 1.0f));

  cr = cosf(roll * 0.5f);
  sr = sinf(roll * 0.5f);
  cp = cosf(pitch * 0.5f);
  sp = sinf(pitch * 0.5f);

  ahrs->q0 = cr * cp;
  ahrs->q1 = sr * cp;
  ahrs->q2 = cr * sp;
  ahrs->q3 = -sr * sp;
}

void ImuAhrs_MahonyUpdate(ImuAhrs_Mahony_t *ahrs,
                          float ax_g, float ay_g, float az_g,
                          float gx_dps, float gy_dps, float gz_dps,
                          uint8_t is_static,
                          float dt_s)
{
  float kp;
  float ki;
  float gx;
  float gy;
  float gz;
  float norm;
  float halfvx;
  float halfvy;
  float halfvz;
  float halfex;
  float halfey;
  float halfez;
  float q0;
  float q1;
  float q2;
  float q3;
  float half_gx;
  float half_gy;
  float half_gz;
  float half_dt;
  float k1_q0;
  float k1_q1;
  float k1_q2;
  float k1_q3;
  float q_mid0;
  float q_mid1;
  float q_mid2;
  float q_mid3;
  float k2_q0;
  float k2_q1;
  float k2_q2;
  float k2_q3;

  if ((ahrs == NULL) || (dt_s <= 0.0f))
  {
    return;
  }

  kp = (is_static != 0U) ? IMU_AHRS_KP_STATIC : IMU_AHRS_KP_DYNAMIC;
  ki = (is_static != 0U) ? IMU_AHRS_KI_STATIC : IMU_AHRS_KI_DYNAMIC;

  gx = gx_dps * IMU_AHRS_DEG2RAD;
  gy = gy_dps * IMU_AHRS_DEG2RAD;
  gz = gz_dps * IMU_AHRS_DEG2RAD;

  if (is_static != 0U)
  {
    float gz_corrected = gz - ahrs->yaw_bias_rad_s;
    float yaw_kp = (fabsf(gz_corrected) > 0.005f) ?
                   IMU_AHRS_YAW_BIAS_KP_FAST :
                   IMU_AHRS_YAW_BIAS_KP_SLOW;

    ahrs->yaw_bias_rad_s += yaw_kp * gz_corrected * dt_s;
    ahrs->yaw_bias_rad_s = ImuAhrs_Clamp(ahrs->yaw_bias_rad_s,
                                         -IMU_AHRS_YAW_BIAS_MAX,
                                         IMU_AHRS_YAW_BIAS_MAX);
  }
  gz -= ahrs->yaw_bias_rad_s;

  norm = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
  if (norm <= 0.0001f)
  {
    return;
  }
  ax_g /= norm;
  ay_g /= norm;
  az_g /= norm;

  halfvx = ahrs->q1 * ahrs->q3 - ahrs->q0 * ahrs->q2;
  halfvy = ahrs->q0 * ahrs->q1 + ahrs->q2 * ahrs->q3;
  halfvz = ahrs->q0 * ahrs->q0 - 0.5f + ahrs->q3 * ahrs->q3;

  halfex = ay_g * halfvz - az_g * halfvy;
  halfey = az_g * halfvx - ax_g * halfvz;
  halfez = ax_g * halfvy - ay_g * halfvx;

  if (ki > 0.0f)
  {
    ahrs->ex_int += ki * halfex * dt_s;
    ahrs->ey_int += ki * halfey * dt_s;
    ahrs->ez_int += ki * halfez * dt_s;

    ahrs->ex_int = ImuAhrs_Clamp(ahrs->ex_int, IMU_AHRS_INTEGRAL_MIN, IMU_AHRS_INTEGRAL_MAX);
    ahrs->ey_int = ImuAhrs_Clamp(ahrs->ey_int, IMU_AHRS_INTEGRAL_MIN, IMU_AHRS_INTEGRAL_MAX);
    ahrs->ez_int = ImuAhrs_Clamp(ahrs->ez_int, IMU_AHRS_INTEGRAL_MIN, IMU_AHRS_INTEGRAL_MAX);

    gx += ahrs->ex_int;
    gy += ahrs->ey_int;
    gz += ahrs->ez_int;
  }
  else
  {
    ahrs->ex_int = 0.0f;
    ahrs->ey_int = 0.0f;
    ahrs->ez_int = 0.0f;
  }

  gx += kp * halfex;
  gy += kp * halfey;
  gz += kp * halfez;

  q0 = ahrs->q0;
  q1 = ahrs->q1;
  q2 = ahrs->q2;
  q3 = ahrs->q3;
  half_gx = 0.5f * gx;
  half_gy = 0.5f * gy;
  half_gz = 0.5f * gz;

  k1_q0 = -q1 * half_gx - q2 * half_gy - q3 * half_gz;
  k1_q1 =  q0 * half_gx + q2 * half_gz - q3 * half_gy;
  k1_q2 =  q0 * half_gy - q1 * half_gz + q3 * half_gx;
  k1_q3 =  q0 * half_gz + q1 * half_gy - q2 * half_gx;

  half_dt = 0.5f * dt_s;
  q_mid0 = q0 + k1_q0 * half_dt;
  q_mid1 = q1 + k1_q1 * half_dt;
  q_mid2 = q2 + k1_q2 * half_dt;
  q_mid3 = q3 + k1_q3 * half_dt;

  norm = sqrtf((q_mid0 * q_mid0) + (q_mid1 * q_mid1) +
               (q_mid2 * q_mid2) + (q_mid3 * q_mid3));
  if (norm <= 0.0001f)
  {
    return;
  }
  q_mid0 /= norm;
  q_mid1 /= norm;
  q_mid2 /= norm;
  q_mid3 /= norm;

  k2_q0 = -q_mid1 * half_gx - q_mid2 * half_gy - q_mid3 * half_gz;
  k2_q1 =  q_mid0 * half_gx + q_mid2 * half_gz - q_mid3 * half_gy;
  k2_q2 =  q_mid0 * half_gy - q_mid1 * half_gz + q_mid3 * half_gx;
  k2_q3 =  q_mid0 * half_gz + q_mid1 * half_gy - q_mid2 * half_gx;

  ahrs->q0 = q0 + k2_q0 * dt_s;
  ahrs->q1 = q1 + k2_q1 * dt_s;
  ahrs->q2 = q2 + k2_q2 * dt_s;
  ahrs->q3 = q3 + k2_q3 * dt_s;

  norm = sqrtf((ahrs->q0 * ahrs->q0) + (ahrs->q1 * ahrs->q1) +
               (ahrs->q2 * ahrs->q2) + (ahrs->q3 * ahrs->q3));
  if (norm > 0.0001f)
  {
    ahrs->q0 /= norm;
    ahrs->q1 /= norm;
    ahrs->q2 /= norm;
    ahrs->q3 /= norm;
  }
}

void ImuAhrs_MahonyGetEulerDeg(const ImuAhrs_Mahony_t *ahrs,
                               float *roll_deg,
                               float *pitch_deg,
                               float *yaw_deg)
{
  float roll_rad;
  float pitch_sin;
  float pitch_rad;
  float yaw_rad;

  if (ahrs == NULL)
  {
    return;
  }

  roll_rad = atan2f(2.0f * ((ahrs->q0 * ahrs->q1) + (ahrs->q2 * ahrs->q3)),
                    1.0f - 2.0f * ((ahrs->q1 * ahrs->q1) + (ahrs->q2 * ahrs->q2)));
  pitch_sin = 2.0f * ((ahrs->q0 * ahrs->q2) - (ahrs->q3 * ahrs->q1));
  pitch_rad = asinf(ImuAhrs_Clamp(pitch_sin, -1.0f, 1.0f));
  yaw_rad = atan2f(2.0f * ((ahrs->q0 * ahrs->q3) + (ahrs->q1 * ahrs->q2)),
                   1.0f - 2.0f * ((ahrs->q2 * ahrs->q2) + (ahrs->q3 * ahrs->q3)));

  if (roll_deg != NULL)
  {
    *roll_deg = roll_rad * IMU_AHRS_RAD2DEG;
  }
  if (pitch_deg != NULL)
  {
    *pitch_deg = pitch_rad * IMU_AHRS_RAD2DEG;
  }
  if (yaw_deg != NULL)
  {
    *yaw_deg = yaw_rad * IMU_AHRS_RAD2DEG;
  }
}
