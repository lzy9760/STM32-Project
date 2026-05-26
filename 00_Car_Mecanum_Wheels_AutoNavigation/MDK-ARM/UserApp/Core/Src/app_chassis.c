#include "app_chassis.h"
#include "pid.h"

#define RC_INPUT_MAX            100
#define RC_DEADZONE             8
#define RC_TRANSLATION_EXPO     0.20f
#define RC_ROTATION_EXPO        0.20f
#define RC_AXIS_ASSIST_MIN      0.18f
#define RC_AXIS_ASSIST_RATIO    1.80f
#define CHASSIS_REMOTE_PWM_MAX  65535U

/*
 * Per-wheel speed calibration gains.
 *
 * Even with closed-loop PID, identical target speeds on all four wheels
 * can produce a crooked trajectory because every motor has slightly
 * different characteristics (winding resistance, gearbox friction,
 * wheel diameter, etc.).
 *
 * These gains are applied AFTER mecanum kinematics so they do not break
 * the inverse-kinematics model.  Tune them one wheel at a time:
 *   - If the chassis veers RIGHT when going FORWARD, the left-side
 *     wheels are too fast  ->  decrease FL_GAIN and / or RL_GAIN.
 *   - If the chassis veers LEFT  when going FORWARD, the right-side
 *     wheels are too fast  ->  decrease FR_GAIN and / or RR_GAIN.
 *   - If the chassis rotates while STRAFING, tune front-vs-rear gains.
 *
 * Typical range: 0.85f .. 1.15f.  1.00f = no adjustment.
 */
#define CHASSIS_FL_GAIN  1.00f
#define CHASSIS_FR_GAIN  0.97f
#define CHASSIS_RL_GAIN  1.00f
#define CHASSIS_RR_GAIN  0.97f

static float ClampFloat(float value, float min_value, float max_value)
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

static float AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float ApplyExpo(float value, float expo)
{
  return ((1.0f - expo) * value) + (expo * value * value * value);//指数曲线
}

static void ApplyTranslationAxisAssist(float *forward_value, float *strafe_value)
{
  float forward_abs;
  float strafe_abs;

  forward_abs = AbsFloat(*forward_value);
  strafe_abs = AbsFloat(*strafe_value);

  
  if ((strafe_abs > RC_AXIS_ASSIST_MIN) &&
      (strafe_abs > (forward_abs * RC_AXIS_ASSIST_RATIO)))
  {
    *forward_value = 0.0f;
  }
  else if ((forward_abs > RC_AXIS_ASSIST_MIN) &&
           (forward_abs > (strafe_abs * RC_AXIS_ASSIST_RATIO)))
  {
    *strafe_value = 0.0f;
  }
}

static float NormalizeJoystick(int16_t value)
{
  float x;

  x = ClampFloat((float)value, -RC_INPUT_MAX, RC_INPUT_MAX);
  if ((x > -(float)RC_DEADZONE) && (x < (float)RC_DEADZONE))
  {
    return 0.0f;
  }

  if (x > 0.0f)
  {
    x = (x - (float)RC_DEADZONE) / (float)(RC_INPUT_MAX - RC_DEADZONE);
  }
  else
  {
    x = (x + (float)RC_DEADZONE) / (float)(RC_INPUT_MAX - RC_DEADZONE);
  }

  return ClampFloat(x, -1.0f, 1.0f);
}

static int32_t ScaleNormalizedToPwm(float value)
{
  float limited;

  limited = ClampFloat(value, -1.0f, 1.0f);
  if (limited >= 0.0f)
  {
    return (int32_t)((limited * (float)CHASSIS_REMOTE_PWM_MAX) + 0.5f);
  }

  return (int32_t)((limited * (float)CHASSIS_REMOTE_PWM_MAX) - 0.5f);
}

void Chassis_SetNormalizedMotion(float forward, float strafe, float yaw)
{
  float vx;
  float vy;
  float kw;
  float front_left;
  float front_right;
  float rear_left;
  float rear_right;
  float max_magnitude;

  vx = ClampFloat(forward, -1.0f, 1.0f);
  vy = ClampFloat(strafe, -1.0f, 1.0f);
  kw = ClampFloat(yaw, -1.0f, 1.0f);

  front_left = vx - vy + kw;
  front_right = vx + vy - kw;
  rear_left = vx + vy + kw;
  rear_right = vx - vy - kw;

  max_magnitude = AbsFloat(front_left);
  if (AbsFloat(front_right) > max_magnitude)
  {
    max_magnitude = AbsFloat(front_right);
  }
  if (AbsFloat(rear_left) > max_magnitude)
  {
    max_magnitude = AbsFloat(rear_left);
  }
  if (AbsFloat(rear_right) > max_magnitude)
  {
    max_magnitude = AbsFloat(rear_right);
  }
  if (max_magnitude < 1.0f)
  {
    max_magnitude = 1.0f;
  }

  Pid_SetTargetAll(ScaleNormalizedToPwm((front_left / max_magnitude) * CHASSIS_FL_GAIN),
                   ScaleNormalizedToPwm((front_right / max_magnitude) * CHASSIS_FR_GAIN),
                   ScaleNormalizedToPwm((rear_left / max_magnitude) * CHASSIS_RL_GAIN),
                   ScaleNormalizedToPwm((rear_right / max_magnitude) * CHASSIS_RR_GAIN));
}

void Chassis_SetMotion(int16_t forward, int16_t strafe, int16_t yaw)
{
  float vx;
  float vy;
  float kw;

  vx = ApplyExpo(NormalizeJoystick(forward), RC_TRANSLATION_EXPO);
  vy = ApplyExpo(NormalizeJoystick(strafe), RC_TRANSLATION_EXPO);
  kw = ApplyExpo(NormalizeJoystick(yaw), RC_ROTATION_EXPO);
  ApplyTranslationAxisAssist(&vx, &vy);

  Chassis_SetNormalizedMotion(vx, vy, kw);
}
