#include "app_remote.h"

#include "app_chassis.h"
#include "app_mpu6050.h"
#include "app_odometry.h"
#include "app_nrf24l01.h"
#include "app_nrf_remote.h"
#include "pid.h"

#include "main.h"

#define RC_FORWARD_SIGN         1
#define RC_STRAFE_SIGN          -1
#define RC_YAW_SIGN             1
#define RC_LOST_TIMEOUT_MS      500U
#define RC_YAW_AXIS_DEADZONE    18
#define RC_YAW_DEADZONE         30
#define RC_YAW_RELEASE_DEADZONE 15
#define RC_YAW_CONFIRM_COUNT    3U
#define RC_MOVE_DEADZONE        10
#define RC_MOVE_CONFIRM_COUNT   3U
#define RC_STOP_CONFIRM_MS      80U

static float Remote_ClampFloat(float value, float min_value, float max_value)
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

static float Remote_NormalizeAxis(int16_t value)
{
  float axis;
  float deadzone;

  axis = Remote_ClampFloat((float)value, -100.0f, 100.0f);
  deadzone = (float)RC_MOVE_DEADZONE;
  if ((axis > -deadzone) && (axis < deadzone))
  {
    return 0.0f;
  }

  if (axis > 0.0f)
  {
    axis = (axis - deadzone) / (100.0f - deadzone);
  }
  else
  {
    axis = (axis + deadzone) / (100.0f - deadzone);
  }

  return Remote_ClampFloat(axis, -1.0f, 1.0f);
}

static int16_t Remote_ApplyDeadzone(int16_t value, int16_t deadzone)
{
  if ((value > -deadzone) && (value < deadzone))
  {
    return 0;
  }

  return value;
}

typedef struct
{
  volatile int16_t lh;
  volatile int16_t lv;
  volatile int16_t rh;
  volatile int16_t rv;
  volatile uint8_t online;
  volatile uint32_t last_rx_tick;
} RemoteControl_t;

static RemoteControl_t g_remote_control;
static uint8_t g_move_confirm_count;
static uint8_t g_yaw_confirm_count;
static uint32_t g_remote_last_motion_tick;
static uint32_t g_remote_zero_start_tick;

static uint8_t ProcessNrfPacket(void)
{
  NrfRemote_Data_t nrf_data;

  if (NrfRemote_Read(&nrf_data) == 0U)
  {
    return 0U;
  }

  g_remote_control.lh = nrf_data.left_x;
  g_remote_control.lv = nrf_data.left_y;
  g_remote_control.rh = nrf_data.right_x;
  g_remote_control.rv = nrf_data.right_y;
  g_remote_control.online = 1U;
  g_remote_control.last_rx_tick = HAL_GetTick();

  return 1U;
}

static void Remote_ZeroCommand(uint8_t keep_online)
{
  g_remote_control.lh = 0;
  g_remote_control.lv = 0;
  g_remote_control.rh = 0;
  g_remote_control.rv = 0;

  if (keep_online != 0U)
  {
    g_remote_control.online = 1U;
    g_remote_control.last_rx_tick = HAL_GetTick();
  }
  else
  {
    g_remote_control.online = 0U;
    g_remote_control.last_rx_tick = 0U;
  }
  Pid_StopAll();
  g_move_confirm_count = 0U;
  g_yaw_confirm_count = 0U;
  g_remote_last_motion_tick = 0U;
  g_remote_zero_start_tick = 0U;
}

void Remote_Init(void)
{
  Remote_ZeroCommand(0U);
  NrfRemote_Init();
  g_remote_last_motion_tick = 0U;
  g_remote_zero_start_tick = 0U;
}

uint8_t Remote_HasManualCommand(void)
{
  int16_t yaw_sample;
  int16_t lh_cmd;
  int16_t lv_cmd;
  int16_t rh_cmd;
  int16_t rv_cmd;

  (void)ProcessNrfPacket();
  if (g_remote_control.online == 0U)
  {
    return 0U;
  }

  if ((HAL_GetTick() - g_remote_control.last_rx_tick) > RC_LOST_TIMEOUT_MS)
  {
    Remote_ZeroCommand(0U);
    return 0U;
  }

  lh_cmd = Remote_ApplyDeadzone(g_remote_control.lh, RC_MOVE_DEADZONE);
  lv_cmd = Remote_ApplyDeadzone(g_remote_control.lv, RC_MOVE_DEADZONE);
  rv_cmd = Remote_ApplyDeadzone(g_remote_control.rv, RC_YAW_AXIS_DEADZONE);
  rh_cmd = Remote_ApplyDeadzone(g_remote_control.rh, RC_YAW_AXIS_DEADZONE);
  yaw_sample = rv_cmd + rh_cmd;

  return (((lh_cmd != 0) ||
           (lv_cmd != 0) ||
           (yaw_sample > RC_YAW_DEADZONE) ||
           (yaw_sample < -RC_YAW_DEADZONE)) ? 1U : 0U);
}

uint8_t Remote_IsOnline(void)
{
  if (g_remote_control.online == 0U)
  {
    return 0U;
  }

  if ((HAL_GetTick() - g_remote_control.last_rx_tick) > RC_LOST_TIMEOUT_MS)
  {
    return 0U;
  }

  return 1U;
}

void Remote_ControlTask(void)
{
  uint32_t now;
  int16_t yaw_raw;
  int16_t yaw_sample;
  int16_t yaw_corrected;
  int16_t lh_cmd;
  int16_t lv_cmd;
  int16_t rh_cmd;
  int16_t rv_cmd;
  uint8_t moving_sample;
  uint8_t moving_cmd;
  uint8_t rx_ok;

  MPU6050_Update();
  Odometry_Update();
  rx_ok = ProcessNrfPacket();

  if (g_remote_control.online == 0U)
  {
    Pid_StopAll();
    return;
  }

  now = HAL_GetTick();
  if ((now - g_remote_control.last_rx_tick) > RC_LOST_TIMEOUT_MS)
  {
    Remote_ZeroCommand(0U);
    return;
  }

  /*
   * Control mapping:
   *   lh (left  horizontal) -> forward / backward
   *   lv (left  vertical)   -> lateral strafe (left / right, sign-adjusted)
   *   rv (right vertical)   -> turning while moving (car-like steering)
   *   rh (right horizontal) -> self-spin (pure rotation in place)
   *
   * rv and rh are summed into the single yaw parameter.  A separate yaw
   * deadzone filters out residual stick-centre offsets from BOTH axes
   * that would otherwise cause the chassis to turn when only forward /
   * strafe is commanded.
   */
  lh_cmd = Remote_ApplyDeadzone(g_remote_control.lh, RC_MOVE_DEADZONE);
  lv_cmd = Remote_ApplyDeadzone(g_remote_control.lv, RC_MOVE_DEADZONE);
  rv_cmd = Remote_ApplyDeadzone(g_remote_control.rv, RC_YAW_AXIS_DEADZONE);
  rh_cmd = Remote_ApplyDeadzone(g_remote_control.rh, RC_YAW_AXIS_DEADZONE);
  yaw_sample = rv_cmd + rh_cmd;
  if (yaw_sample > -RC_YAW_RELEASE_DEADZONE && yaw_sample < RC_YAW_RELEASE_DEADZONE)
  {
    g_yaw_confirm_count = 0U;
    yaw_raw = 0;
  }
  else if (yaw_sample > RC_YAW_DEADZONE || yaw_sample < -RC_YAW_DEADZONE)
  {
    if (g_yaw_confirm_count < RC_YAW_CONFIRM_COUNT)
    {
      g_yaw_confirm_count++;
    }
    yaw_raw = (g_yaw_confirm_count >= RC_YAW_CONFIRM_COUNT) ? yaw_sample : 0;
  }
  else
  {
    yaw_raw = 0;
  }

  /* Tell MPU6050 whether the car is really commanded to move. Use the same
     input deadzone as chassis translation so stick-center noise does not
     engage heading hold while parked. */
  moving_sample = ((lh_cmd != 0) || (lv_cmd != 0)) ? 1U : 0U;
  if (moving_sample != 0U)
  {
    if (g_move_confirm_count < RC_MOVE_CONFIRM_COUNT)
    {
      g_move_confirm_count++;
    }
    moving_cmd = (g_move_confirm_count >= RC_MOVE_CONFIRM_COUNT) ? 1U : 0U;
  }
  else
  {
    g_move_confirm_count = 0U;
    moving_cmd = 0U;
  }
  MPU6050_NotifyMoving(moving_cmd);

  if ((moving_cmd != 0U) || (yaw_raw != 0))
  {
    g_remote_last_motion_tick = now;
    g_remote_zero_start_tick = 0U;
  }

  if ((moving_cmd == 0U) && (yaw_raw == 0))
  {
    if (rx_ok == 0U)
    {
      return;
    }

    if (g_remote_zero_start_tick == 0U)
    {
      g_remote_zero_start_tick = now;
    }

    if ((g_remote_last_motion_tick == 0U) ||
        ((now - g_remote_zero_start_tick) >= RC_STOP_CONFIRM_MS))
    {
      Pid_StopAll();
    }
    return;
  }

  yaw_corrected = MPU6050_GetHeadingCorrection((int16_t)(RC_YAW_SIGN * yaw_raw));

  if (yaw_raw == 0)
  {
    Chassis_SetNormalizedMotion(Remote_NormalizeAxis((int16_t)(RC_FORWARD_SIGN * lh_cmd)),
                                Remote_NormalizeAxis((int16_t)(RC_STRAFE_SIGN * lv_cmd)),
                                Remote_ClampFloat((float)yaw_corrected / 100.0f, -0.45f, 0.45f));
  }
  else
  {
    Chassis_SetMotion((int16_t)(RC_FORWARD_SIGN * lh_cmd),
                      (int16_t)(RC_STRAFE_SIGN * lv_cmd),
                      yaw_corrected);
  }
}
