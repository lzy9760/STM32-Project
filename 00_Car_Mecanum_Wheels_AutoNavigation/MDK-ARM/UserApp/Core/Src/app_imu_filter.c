#include "app_imu_filter.h"

#include <math.h>
#include <string.h>

#define IMU_FILTER_DEFAULT_P              1.0f
#define IMU_FILTER_DEFAULT_Q              1.0e-5f
#define IMU_FILTER_MIN_Q                  1.0e-8f
#define IMU_FILTER_MIN_R                  1.0e-6f
#define IMU_FILTER_STABILITY_THRESHOLD    0.01f
#define IMU_FILTER_STABILITY_COUNT        100U
#define IMU_FILTER_STATIC_RATE_DPS        2.0f
#define IMU_FILTER_BIAS_ALPHA             0.002f

static float ImuFilter_Abs(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static void ImuFilter_AkfInit(ImuFilter_Akf_t *akf, float x0, float p0, float q0, float r0)
{
  memset(akf, 0, sizeof(ImuFilter_Akf_t));
  akf->x_hat = x0;
  akf->p = p0;
  akf->q = q0;
  akf->r = (r0 > IMU_FILTER_MIN_R) ? r0 : IMU_FILTER_MIN_R;
}

static float ImuFilter_AkfUpdate(ImuFilter_Akf_t *akf, float z)
{
  float denominator;
  uint8_t i;

  akf->x_pre = akf->x_hat;
  akf->p_pre = akf->p + akf->q;

  denominator = akf->p_pre + akf->r;
  if (ImuFilter_Abs(denominator) < 1.0e-10f)
  {
    akf->k = 0.0f;
  }
  else
  {
    akf->k = akf->p_pre / denominator;
  }

  akf->innovation = z - akf->x_pre;
  akf->innovation_window[akf->window_idx] = akf->innovation;
  akf->window_idx++;
  if (akf->window_idx >= IMU_FILTER_WINDOW_SIZE)
  {
    akf->window_idx = 0U;
    akf->window_full = 1U;
  }

  if (akf->window_full != 0U)
  {
    akf->innovation_var = 0.0f;
    for (i = 0U; i < IMU_FILTER_WINDOW_SIZE; i++)
    {
      akf->innovation_var += akf->innovation_window[i] * akf->innovation_window[i];
    }
    akf->innovation_var /= (float)IMU_FILTER_WINDOW_SIZE;
    akf->q = akf->k * akf->k * akf->innovation_var;
    if (akf->q < IMU_FILTER_MIN_Q)
    {
      akf->q = IMU_FILTER_MIN_Q;
    }
  }

  akf->x_hat = akf->x_pre + akf->k * akf->innovation;
  akf->p = (1.0f - akf->k) * akf->p_pre;
  if (akf->p < 0.0f)
  {
    akf->p = IMU_FILTER_MIN_Q;
  }

  return akf->x_hat;
}

void ImuFilter_GyroInit(ImuFilter_GyroAxis_t *filter, float initial_bias_dps, float measure_var)
{
  if (filter == NULL)
  {
    return;
  }

  memset(filter, 0, sizeof(ImuFilter_GyroAxis_t));
  filter->bias_dps = initial_bias_dps;
  ImuFilter_AkfInit(&filter->akf,
                    0.0f,
                    IMU_FILTER_DEFAULT_P,
                    IMU_FILTER_DEFAULT_Q,
                    measure_var);
}

float ImuFilter_GyroUpdate(ImuFilter_GyroAxis_t *filter, float raw_dps, uint8_t allow_static_bias)
{
  float corrected_dps;
  float filtered_dps;
  float delta;

  if (filter == NULL)
  {
    return raw_dps;
  }

  corrected_dps = raw_dps - filter->bias_dps;
  filtered_dps = ImuFilter_AkfUpdate(&filter->akf, corrected_dps);

  if (allow_static_bias != 0U)
  {
    delta = ImuFilter_Abs(filtered_dps - filter->last_filtered_dps);
    if ((delta < IMU_FILTER_STABILITY_THRESHOLD) &&
        (ImuFilter_Abs(filtered_dps) < IMU_FILTER_STATIC_RATE_DPS))
    {
      if (filter->stable_count < IMU_FILTER_STABILITY_COUNT)
      {
        filter->stable_count++;
      }
      else
      {
        filter->calibrated = 1U;
      }
    }
    else
    {
      filter->stable_count = 0U;
    }

    if ((filter->calibrated != 0U) || (filter->stable_count >= IMU_FILTER_STABILITY_COUNT))
    {
      filter->bias_dps += filtered_dps * IMU_FILTER_BIAS_ALPHA;
      filtered_dps = 0.0f;
    }
  }
  else
  {
    filter->stable_count = 0U;
  }

  filter->last_filtered_dps = filtered_dps;
  return filtered_dps;
}
