#ifndef __APP_IMU_FILTER_H
#define __APP_IMU_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

#define IMU_FILTER_WINDOW_SIZE  10U

typedef struct
{
  float x_hat;
  float x_pre;
  float p;
  float p_pre;
  float k;
  float q;
  float r;
  float innovation;
  float innovation_var;
  float innovation_window[IMU_FILTER_WINDOW_SIZE];
  uint8_t window_idx;
  uint8_t window_full;
} ImuFilter_Akf_t;

typedef struct
{
  ImuFilter_Akf_t akf;
  float bias_dps;
  float last_filtered_dps;
  uint16_t stable_count;
  uint8_t calibrated;
} ImuFilter_GyroAxis_t;

void ImuFilter_GyroInit(ImuFilter_GyroAxis_t *filter, float initial_bias_dps, float measure_var);
float ImuFilter_GyroUpdate(ImuFilter_GyroAxis_t *filter, float raw_dps, uint8_t allow_static_bias);

#ifdef __cplusplus
}
#endif

#endif /* __APP_IMU_FILTER_H */
