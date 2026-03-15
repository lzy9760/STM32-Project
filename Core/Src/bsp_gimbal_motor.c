#include "bsp_gimbal_motor.h"

#include "bsp_can.h"
#include "robot_param.h"

static struct
{
  float yaw_deg;
  float pitch_deg;
  float yaw_rate_dps;
  float pitch_rate_dps;

  float yaw_current_cmd;
  float pitch_current_cmd;

  float yaw_min_deg;
  float yaw_max_deg;
  float pitch_min_deg;
  float pitch_max_deg;

  uint8_t limit_triggered;
  uint8_t can_tx_enable;
  int16_t yaw_can_cmd;
  int16_t pitch_can_cmd;

  uint8_t yaw_fb_valid;
  uint8_t pitch_fb_valid;
  uint16_t yaw_last_ecd;
  uint16_t pitch_last_ecd;
  uint16_t yaw_base_ecd;
  uint16_t pitch_base_ecd;
  int32_t yaw_turn;
  int32_t pitch_turn;
  uint32_t yaw_fb_ms;
  uint32_t pitch_fb_ms;
} s_gimbal;

static float BSP_Gimbal_Clamp(float value, float min_value, float max_value)
{
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

static int16_t BSP_Gimbal_FloatToCurrent(float current_cmd)
{
  if (current_cmd > 16384.0f)
  {
    current_cmd = 16384.0f;
  }
  if (current_cmd < -16384.0f)
  {
    current_cmd = -16384.0f;
  }

  return (int16_t)current_cmd;
}

static void BSP_Gimbal_UpdateAxisFromCan(float *angle_deg,
                                         float *rate_dps,
                                         uint8_t *fb_valid,
                                         uint16_t *last_ecd,
                                         uint16_t *base_ecd,
                                         int32_t *turn_cnt,
                                         uint32_t *last_fb_ms,
                                         uint16_t ecd,
                                         int16_t rpm,
                                         uint16_t user_offset,
                                         float gear_ratio)
{
  int32_t ecd_res;
  int32_t delta;
  int32_t total_ecd;

  if (angle_deg == 0 || rate_dps == 0 || fb_valid == 0 || last_ecd == 0 ||
      base_ecd == 0 || turn_cnt == 0 || last_fb_ms == 0)
  {
    return;
  }

  if (gear_ratio < 0.01f)
  {
    gear_ratio = 1.0f;
  }

  ecd_res = (int32_t)ROBOT_GIMBAL_ENCODER_RESOLUTION;
  if (ecd_res <= 1)
  {
    ecd_res = 8192;
  }

  if (*fb_valid == 0U)
  {
    *turn_cnt = 0;
    *last_ecd = ecd;
    *base_ecd = (uint16_t)(((uint32_t)ecd + (uint32_t)user_offset) % (uint32_t)ecd_res);
  }
  else
  {
    delta = (int32_t)ecd - (int32_t)(*last_ecd);
    if (delta > (ecd_res / 2))
    {
      (*turn_cnt)--;
    }
    else if (delta < -(ecd_res / 2))
    {
      (*turn_cnt)++;
    }

    *last_ecd = ecd;
  }

  total_ecd = (*turn_cnt) * ecd_res + (int32_t)ecd - (int32_t)(*base_ecd);
  *angle_deg = ((float)total_ecd * 360.0f) / ((float)ecd_res * gear_ratio);
  *rate_dps = ((float)rpm * 6.0f) / gear_ratio;

  *fb_valid = 1U;
  *last_fb_ms = HAL_GetTick();
}

void BSP_GimbalMotor_Init(void)
{
  s_gimbal.yaw_deg = 0.0f;
  s_gimbal.pitch_deg = 0.0f;
  s_gimbal.yaw_rate_dps = 0.0f;
  s_gimbal.pitch_rate_dps = 0.0f;
  s_gimbal.yaw_current_cmd = 0.0f;
  s_gimbal.pitch_current_cmd = 0.0f;
  s_gimbal.yaw_min_deg = -150.0f;
  s_gimbal.yaw_max_deg = 150.0f;
  s_gimbal.pitch_min_deg = -25.0f;
  s_gimbal.pitch_max_deg = 35.0f;
  s_gimbal.limit_triggered = 0U;
  s_gimbal.can_tx_enable = 1U;
  s_gimbal.yaw_can_cmd = 0;
  s_gimbal.pitch_can_cmd = 0;

  s_gimbal.yaw_fb_valid = 0U;
  s_gimbal.pitch_fb_valid = 0U;
  s_gimbal.yaw_last_ecd = 0U;
  s_gimbal.pitch_last_ecd = 0U;
  s_gimbal.yaw_base_ecd = 0U;
  s_gimbal.pitch_base_ecd = 0U;
  s_gimbal.yaw_turn = 0;
  s_gimbal.pitch_turn = 0;
  s_gimbal.yaw_fb_ms = 0U;
  s_gimbal.pitch_fb_ms = 0U;
}

void BSP_GimbalMotor_SetSoftLimit(float yaw_min_deg,
                                  float yaw_max_deg,
                                  float pitch_min_deg,
                                  float pitch_max_deg)
{
  if (yaw_min_deg < yaw_max_deg)
  {
    s_gimbal.yaw_min_deg = yaw_min_deg;
    s_gimbal.yaw_max_deg = yaw_max_deg;
  }

  if (pitch_min_deg < pitch_max_deg)
  {
    s_gimbal.pitch_min_deg = pitch_min_deg;
    s_gimbal.pitch_max_deg = pitch_max_deg;
  }
}

void BSP_GimbalMotor_CommandCurrent(float yaw_current_cmd, float pitch_current_cmd)
{
  s_gimbal.yaw_current_cmd = BSP_Gimbal_Clamp(yaw_current_cmd, -8.0f, 8.0f);
  s_gimbal.pitch_current_cmd = BSP_Gimbal_Clamp(pitch_current_cmd, -8.0f, 8.0f);
}

void BSP_GimbalMotor_SetCanTxEnable(uint8_t enable)
{
  s_gimbal.can_tx_enable = (enable != 0U) ? 1U : 0U;
}

void BSP_GimbalMotor_Update(float dt_s)
{
  const float current_to_accel = 140.0f;
  const float damping = 9.0f;
  const float can_current_scale = 2000.0f;

  float yaw_cmd;
  float pitch_cmd;
  uint32_t now;

  if (dt_s <= 0.0f)
  {
    return;
  }

  now = HAL_GetTick();
  if (s_gimbal.yaw_fb_valid != 0U && (now - s_gimbal.yaw_fb_ms) > ROBOT_GIMBAL_FB_TIMEOUT_MS)
  {
    s_gimbal.yaw_fb_valid = 0U;
  }
  if (s_gimbal.pitch_fb_valid != 0U && (now - s_gimbal.pitch_fb_ms) > ROBOT_GIMBAL_FB_TIMEOUT_MS)
  {
    s_gimbal.pitch_fb_valid = 0U;
  }

  s_gimbal.limit_triggered = 0U;
  yaw_cmd = s_gimbal.yaw_current_cmd;
  pitch_cmd = s_gimbal.pitch_current_cmd;

  if ((s_gimbal.yaw_deg >= s_gimbal.yaw_max_deg && yaw_cmd > 0.0f) ||
      (s_gimbal.yaw_deg <= s_gimbal.yaw_min_deg && yaw_cmd < 0.0f))
  {
    yaw_cmd = 0.0f;
    s_gimbal.limit_triggered = 1U;
  }

  if ((s_gimbal.pitch_deg >= s_gimbal.pitch_max_deg && pitch_cmd > 0.0f) ||
      (s_gimbal.pitch_deg <= s_gimbal.pitch_min_deg && pitch_cmd < 0.0f))
  {
    pitch_cmd = 0.0f;
    s_gimbal.limit_triggered = 1U;
  }

  if (s_gimbal.yaw_fb_valid == 0U)
  {
    float yaw_accel;

    yaw_accel = yaw_cmd * current_to_accel - s_gimbal.yaw_rate_dps * damping;
    s_gimbal.yaw_rate_dps += yaw_accel * dt_s;
    s_gimbal.yaw_deg += s_gimbal.yaw_rate_dps * dt_s;

    if (s_gimbal.yaw_deg >= s_gimbal.yaw_max_deg)
    {
      s_gimbal.yaw_deg = s_gimbal.yaw_max_deg;
      if (s_gimbal.yaw_rate_dps > 0.0f)
      {
        s_gimbal.yaw_rate_dps = 0.0f;
        s_gimbal.limit_triggered = 1U;
      }
    }
    else if (s_gimbal.yaw_deg <= s_gimbal.yaw_min_deg)
    {
      s_gimbal.yaw_deg = s_gimbal.yaw_min_deg;
      if (s_gimbal.yaw_rate_dps < 0.0f)
      {
        s_gimbal.yaw_rate_dps = 0.0f;
        s_gimbal.limit_triggered = 1U;
      }
    }
  }

  if (s_gimbal.pitch_fb_valid == 0U)
  {
    float pitch_accel;

    pitch_accel = pitch_cmd * current_to_accel - s_gimbal.pitch_rate_dps * damping;
    s_gimbal.pitch_rate_dps += pitch_accel * dt_s;
    s_gimbal.pitch_deg += s_gimbal.pitch_rate_dps * dt_s;

    if (s_gimbal.pitch_deg >= s_gimbal.pitch_max_deg)
    {
      s_gimbal.pitch_deg = s_gimbal.pitch_max_deg;
      if (s_gimbal.pitch_rate_dps > 0.0f)
      {
        s_gimbal.pitch_rate_dps = 0.0f;
        s_gimbal.limit_triggered = 1U;
      }
    }
    else if (s_gimbal.pitch_deg <= s_gimbal.pitch_min_deg)
    {
      s_gimbal.pitch_deg = s_gimbal.pitch_min_deg;
      if (s_gimbal.pitch_rate_dps < 0.0f)
      {
        s_gimbal.pitch_rate_dps = 0.0f;
        s_gimbal.limit_triggered = 1U;
      }
    }
  }

  s_gimbal.yaw_can_cmd = BSP_Gimbal_FloatToCurrent(yaw_cmd * can_current_scale);
  s_gimbal.pitch_can_cmd = BSP_Gimbal_FloatToCurrent(pitch_cmd * can_current_scale);

  if (s_gimbal.can_tx_enable != 0U)
  {
    (void)BSP_CAN_SendCurrentGroup2(s_gimbal.yaw_can_cmd, s_gimbal.pitch_can_cmd, 0, 0);
  }
}

void BSP_GimbalMotor_GetFeedback(BSP_GimbalFeedback_t *feedback)
{
  if (feedback == 0)
  {
    return;
  }

  feedback->yaw_deg = s_gimbal.yaw_deg;
  feedback->pitch_deg = s_gimbal.pitch_deg;
  feedback->yaw_rate_dps = s_gimbal.yaw_rate_dps;
  feedback->pitch_rate_dps = s_gimbal.pitch_rate_dps;
  feedback->limit_triggered = s_gimbal.limit_triggered;
}

void BSP_GimbalMotor_GetCanCurrentCmd(int16_t *yaw_cmd, int16_t *pitch_cmd)
{
  if (yaw_cmd != 0)
  {
    *yaw_cmd = s_gimbal.yaw_can_cmd;
  }

  if (pitch_cmd != 0)
  {
    *pitch_cmd = s_gimbal.pitch_can_cmd;
  }
}

void BSP_GimbalMotor_OnCanFeedback(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
  uint16_t ecd;
  int16_t rpm;

  if (data == 0 || dlc < 4U)
  {
    return;
  }

  ecd = (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
  rpm = (int16_t)(((uint16_t)data[2] << 8U) | (uint16_t)data[3]);

  if (std_id == ROBOT_CAN_ID_GIMBAL_YAW_FB)
  {
    BSP_Gimbal_UpdateAxisFromCan(&s_gimbal.yaw_deg,
                                 &s_gimbal.yaw_rate_dps,
                                 &s_gimbal.yaw_fb_valid,
                                 &s_gimbal.yaw_last_ecd,
                                 &s_gimbal.yaw_base_ecd,
                                 &s_gimbal.yaw_turn,
                                 &s_gimbal.yaw_fb_ms,
                                 ecd,
                                 rpm,
                                 ROBOT_GIMBAL_YAW_ECD_OFFSET,
                                 ROBOT_GIMBAL_YAW_GEAR_RATIO);
  }
  else if (std_id == ROBOT_CAN_ID_GIMBAL_PITCH_FB)
  {
    BSP_Gimbal_UpdateAxisFromCan(&s_gimbal.pitch_deg,
                                 &s_gimbal.pitch_rate_dps,
                                 &s_gimbal.pitch_fb_valid,
                                 &s_gimbal.pitch_last_ecd,
                                 &s_gimbal.pitch_base_ecd,
                                 &s_gimbal.pitch_turn,
                                 &s_gimbal.pitch_fb_ms,
                                 ecd,
                                 rpm,
                                 ROBOT_GIMBAL_PITCH_ECD_OFFSET,
                                 ROBOT_GIMBAL_PITCH_GEAR_RATIO);
  }
}

uint8_t BSP_GimbalMotor_IsLimited(void)
{
  return s_gimbal.limit_triggered;
}
