#include "app_chassis_remote.h"

#include "app_nrf24l01.h"
#include "motor_driver.h"

#define REMOTE_PACKET_LV_INDEX      0U
#define REMOTE_PACKET_LH_INDEX      1U
#define REMOTE_PACKET_RV_INDEX      2U
#define REMOTE_PACKET_RH_INDEX      3U
#define REMOTE_PACKET_BUTTON_INDEX  4U
#define REMOTE_BUTTON_VALID_MASK    0x0FU
#define REMOTE_BUTTON_LIFT_MASK     0x03U
#define REMOTE_BUTTON_SERVO_MASK    0x0CU

#define REMOTE_LINK_TIMEOUT_MS      300U
#define REMOTE_MAX_DUTY_PERCENT     80
#define REMOTE_AXIS_DEADZONE        20
#define REMOTE_YAW_AXIS_DEADZONE    25
#define REMOTE_YAW_DEADZONE         30
#define REMOTE_AXIS_ASSIST_MIN      18
#define REMOTE_AXIS_ASSIST_NUM      18
#define REMOTE_AXIS_ASSIST_DEN      10
#define REMOTE_START_DEADBAND_PERCENT 30
#define REMOTE_START_CONFIRM_PACKETS  4U
#define REMOTE_WHEEL_FILTER_SHIFT     2

#define REMOTE_FORWARD_SIGN         1
#define REMOTE_STRAFE_SIGN          1
#define REMOTE_YAW_SIGN             -1

#define MOTOR0_SIGN                 -1
#define MOTOR1_SIGN                 1
#define MOTOR2_SIGN                 1
#define MOTOR3_SIGN                 1

static ChassisRemoteStatus_t s_status;
static uint32_t s_last_rx_tick;
static uint8_t s_stopped;
static uint8_t s_start_confirm_count;
static int16_t s_filtered_wheel[MOTOR_COUNT];

static int16_t RemoteAbs(int16_t value)
{
  return (value < 0) ? (int16_t)-value : value;
}

static int16_t RemoteAxisToPercentWithDeadzone(uint8_t value, int16_t deadzone)
{
  int16_t axis;

  axis = (int16_t)value - 128;
  if (axis >= 0)
  {
    axis = (int16_t)(((int32_t)axis * 100L) / 127L);
  }
  else
  {
    axis = (int16_t)(((int32_t)axis * 100L) / 128L);
  }

  if (RemoteAbs(axis) <= deadzone)
  {
    return 0;
  }

  return axis;
}

static int16_t RemoteAxisToPercent(uint8_t value)
{
  return RemoteAxisToPercentWithDeadzone(value, REMOTE_AXIS_DEADZONE);
}

static int16_t RemoteScaleDuty(int16_t value)
{
  return (int16_t)(((int32_t)value * REMOTE_MAX_DUTY_PERCENT) / 100L);
}

static uint8_t RemoteNormalizeButtonFlags(uint8_t flags)
{
  flags &= REMOTE_BUTTON_VALID_MASK;

  if ((flags & REMOTE_BUTTON_LIFT_MASK) == REMOTE_BUTTON_LIFT_MASK)
  {
    flags &= (uint8_t)~REMOTE_BUTTON_LIFT_MASK;
  }
  if ((flags & REMOTE_BUTTON_SERVO_MASK) == REMOTE_BUTTON_SERVO_MASK)
  {
    flags &= (uint8_t)~REMOTE_BUTTON_SERVO_MASK;
  }

  return flags;
}

static void RemoteApplyTranslationAxisAssist(int16_t *forward, int16_t *strafe)
{
  int16_t forward_abs;
  int16_t strafe_abs;

  forward_abs = RemoteAbs(*forward);
  strafe_abs = RemoteAbs(*strafe);

  if ((strafe_abs > REMOTE_AXIS_ASSIST_MIN) &&
      (((int32_t)strafe_abs * REMOTE_AXIS_ASSIST_DEN) >
       ((int32_t)forward_abs * REMOTE_AXIS_ASSIST_NUM)))
  {
    *forward = 0;
  }
  else if ((forward_abs > REMOTE_AXIS_ASSIST_MIN) &&
           (((int32_t)forward_abs * REMOTE_AXIS_ASSIST_DEN) >
            ((int32_t)strafe_abs * REMOTE_AXIS_ASSIST_NUM)))
  {
    *strafe = 0;
  }
}

static void RemoteClearWheelCommand(int16_t wheel[4])
{
  uint8_t i;

  for (i = 0U; i < MOTOR_COUNT; i++)
  {
    wheel[i] = 0;
  }
}

static int16_t RemoteFilterWheelValue(uint8_t motor_id, int16_t target)
{
  int32_t filtered;

  if ((s_stopped != 0U) || (target == 0))
  {
    s_filtered_wheel[motor_id] = target;
    return target;
  }

  filtered = (int32_t)s_filtered_wheel[motor_id];
  filtered += (((int32_t)target - filtered) / (1L << REMOTE_WHEEL_FILTER_SHIFT));
  s_filtered_wheel[motor_id] = (int16_t)filtered;
  return s_filtered_wheel[motor_id];
}

static void RemoteFilterWheelCommand(int16_t wheel[4])
{
  uint8_t i;

  for (i = 0U; i < MOTOR_COUNT; i++)
  {
    wheel[i] = RemoteFilterWheelValue(i, wheel[i]);
  }
}

static uint8_t RemoteSuppressStartJitter(int16_t wheel[4])
{
  int16_t max_abs;
  uint8_t i;

  max_abs = 0;
  for (i = 0U; i < MOTOR_COUNT; i++)
  {
    if (RemoteAbs(wheel[i]) > max_abs)
    {
      max_abs = RemoteAbs(wheel[i]);
    }
  }

  if (max_abs < REMOTE_START_DEADBAND_PERCENT)
  {
    RemoteClearWheelCommand(wheel);
    s_start_confirm_count = 0U;
    return 0U;
  }

  if (s_stopped != 0U)
  {
    if (s_start_confirm_count < REMOTE_START_CONFIRM_PACKETS)
    {
      s_start_confirm_count++;
      RemoteClearWheelCommand(wheel);
      return 0U;
    }
  }

  s_start_confirm_count = REMOTE_START_CONFIRM_PACKETS;
  return 1U;
}

static void RemoteApplyStop(void)
{
  uint8_t i;

  Motor_StopAll();
  for (i = 0U; i < MOTOR_COUNT; i++)
  {
    s_status.motor_duty[i] = 0;
    s_filtered_wheel[i] = 0;
  }
  s_stopped = 1U;
  s_start_confirm_count = 0U;
}

static void RemoteApplyPacket(const uint8_t packet[APP_NRF24L01_PAYLOAD_WIDTH])
{
  int16_t forward;
  int16_t strafe;
  int16_t yaw;
  int16_t yaw_sample;
  int16_t wheel[4];
  int16_t max_abs;
  uint8_t motion_active;
  uint8_t i;

  forward = (int16_t)(REMOTE_FORWARD_SIGN * RemoteAxisToPercent(packet[REMOTE_PACKET_LV_INDEX]));
  strafe = (int16_t)(REMOTE_STRAFE_SIGN * RemoteAxisToPercent(packet[REMOTE_PACKET_LH_INDEX]));
  yaw_sample = RemoteAxisToPercentWithDeadzone(packet[REMOTE_PACKET_RV_INDEX], REMOTE_YAW_AXIS_DEADZONE);
  yaw = (RemoteAbs(yaw_sample) > REMOTE_YAW_DEADZONE) ? (int16_t)(REMOTE_YAW_SIGN * yaw_sample) : 0;

  RemoteApplyTranslationAxisAssist(&forward, &strafe);

  wheel[0] = (int16_t)(forward - strafe + yaw);
  wheel[1] = (int16_t)(forward + strafe + yaw);
  wheel[2] = (int16_t)(forward + strafe - yaw);
  wheel[3] = (int16_t)(forward - strafe - yaw);

  max_abs = 1;
  for (i = 0U; i < MOTOR_COUNT; i++)
  {
    if (RemoteAbs(wheel[i]) > max_abs)
    {
      max_abs = RemoteAbs(wheel[i]);
    }
  }

  if (max_abs > 100)
  {
    for (i = 0U; i < MOTOR_COUNT; i++)
    {
      wheel[i] = (int16_t)(((int32_t)wheel[i] * 100L) / max_abs);
    }
  }

  wheel[0] = (int16_t)(MOTOR0_SIGN * RemoteScaleDuty(wheel[0]));
  wheel[1] = (int16_t)(MOTOR1_SIGN * RemoteScaleDuty(wheel[1]));
  wheel[2] = (int16_t)(MOTOR2_SIGN * RemoteScaleDuty(wheel[2]));
  wheel[3] = (int16_t)(MOTOR3_SIGN * RemoteScaleDuty(wheel[3]));
  motion_active = RemoteSuppressStartJitter(wheel);
  if (motion_active != 0U)
  {
    RemoteFilterWheelCommand(wheel);
  }

  for (i = 0U; i < MOTOR_COUNT; i++)
  {
    Motor_SetOutput(i, wheel[i]);
    s_status.motor_duty[i] = Motor_GetDuty(i);
  }

  s_stopped = (motion_active == 0U) ? 1U : 0U;
}

void ChassisRemote_Init(void)
{
  uint8_t i;

  MotorDriver_Init();
  s_status.nrf_ok = AppNrf24l01_Init();
  s_status.link_active = 0U;
  s_status.rx_count = 0U;
  s_status.error_count = 0U;
  s_status.lost_count = 0U;
  for (i = 0U; i < sizeof(s_status.packet); i++)
  {
    s_status.packet[i] = 128U;
  }
  RemoteApplyStop();
  s_last_rx_tick = HAL_GetTick();
}

void ChassisRemote_Task(void)
{
  uint8_t packet[APP_NRF24L01_PAYLOAD_WIDTH];
  AppNrf24l01RxResult_t result;

  result = AppNrf24l01_Receive(packet);
  if (result == APP_NRF24L01_RX_OK)
  {
    uint8_t i;

    packet[REMOTE_PACKET_BUTTON_INDEX] = RemoteNormalizeButtonFlags(packet[REMOTE_PACKET_BUTTON_INDEX]);
    for (i = 0U; i < APP_NRF24L01_PAYLOAD_WIDTH; i++)
    {
      s_status.packet[i] = packet[i];
    }
    s_status.rx_count++;
    s_status.link_active = 1U;
    s_status.nrf_ok = AppNrf24l01_IsConnected();
    s_last_rx_tick = HAL_GetTick();
    RemoteApplyPacket(packet);
  }
  else if (result == APP_NRF24L01_RX_ERROR)
  {
    s_status.error_count++;
    s_status.nrf_ok = AppNrf24l01_IsConnected();
  }

  if ((HAL_GetTick() - s_last_rx_tick) >= REMOTE_LINK_TIMEOUT_MS)
  {
    if (s_status.link_active != 0U)
    {
      s_status.lost_count++;
    }
    s_status.link_active = 0U;
    s_status.packet[REMOTE_PACKET_BUTTON_INDEX] = 0U;
    if (s_stopped == 0U)
    {
      RemoteApplyStop();
    }
  }
}

void ChassisRemote_GetStatus(ChassisRemoteStatus_t *status)
{
  if (status != 0)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *status = s_status;
    if (primask == 0U)
    {
      __enable_irq();
    }
  }
}

const char *ChassisRemote_GetStateName(void)
{
  ChassisRemoteStatus_t status;

  ChassisRemote_GetStatus(&status);
  if (status.nrf_ok == 0U)
  {
    return "NRF ERR";
  }
  if (status.link_active == 0U)
  {
    return "NRF WAIT";
  }
  return "NRF RUN";
}
