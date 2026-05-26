#ifndef __APP_IMU_AHRS_H
#define __APP_IMU_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
  float ex_int;
  float ey_int;
  float ez_int;
  float yaw_bias_rad_s;
} ImuAhrs_Mahony_t;

void ImuAhrs_MahonyInit(ImuAhrs_Mahony_t *ahrs);
void ImuAhrs_MahonyInitWithAccel(ImuAhrs_Mahony_t *ahrs, float ax_g, float ay_g, float az_g);
void ImuAhrs_MahonyUpdate(ImuAhrs_Mahony_t *ahrs,
                          float ax_g, float ay_g, float az_g,
                          float gx_dps, float gy_dps, float gz_dps,
                          uint8_t is_static,
                          float dt_s);
void ImuAhrs_MahonyGetEulerDeg(const ImuAhrs_Mahony_t *ahrs,
                               float *roll_deg,
                               float *pitch_deg,
                               float *yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* __APP_IMU_AHRS_H */
