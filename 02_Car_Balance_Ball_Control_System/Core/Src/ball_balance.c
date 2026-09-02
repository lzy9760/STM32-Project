#include "ball_balance.h"

#include "ball_angle.h"
#include "ball_config.h"
#include "ball_stepper.h"
#include "ball_vision.h"

#define BALL_PI 3.1415926f

typedef struct
{
  float integral;
  float last_error;
  float derivative;
  float integral_limit;
  float derivative_filter_hz;
  uint8_t initialized;
} BallPid_t;

volatile BallDebug_t g_ball_debug;
volatile float g_ball_position_kp = BALL_POSITION_KP_DEFAULT;
volatile float g_ball_position_ki = BALL_POSITION_KI_DEFAULT;
volatile float g_ball_position_kd = BALL_POSITION_KD_DEFAULT;
volatile float g_ball_breakaway_boost_adc =
  BALL_BREAKAWAY_BOOST_ADC_DEFAULT;
volatile float g_ball_final_kp = BALL_FINAL_KP_DEFAULT;
volatile float g_ball_final_ki = BALL_FINAL_KI_DEFAULT;
volatile float g_ball_final_kd = BALL_FINAL_KD_DEFAULT;
volatile float g_ball_final_angle_limit_adc =
  BALL_FINAL_ANGLE_LIMIT_ADC;
volatile float g_ball_final_hold_angle_limit_adc =
  BALL_FINAL_HOLD_ANGLE_LIMIT_ADC;
volatile float g_ball_final_capture_lookahead_s =
  BALL_FINAL_CAPTURE_LOOKAHEAD_S;
volatile float g_ball_angle_kp = BALL_ANGLE_KP_DEFAULT;
volatile float g_ball_angle_ki = BALL_ANGLE_KI_DEFAULT;
volatile float g_ball_angle_kd = BALL_ANGLE_KD_DEFAULT;

static BallPid_t s_position_pid;
static BallPid_t s_angle_pid;
static uint32_t s_init_tick;
static uint32_t s_state_tick;
static uint32_t s_sequence_tick;
static uint32_t s_stable_ms;
static uint8_t s_position_divider;
static float s_target_position_mm;
static float s_requested_position_mm;
static float s_target_angle_adc;
static uint16_t s_direction_baseline_adc;
static volatile uint8_t s_control_mode;

static float LimitFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

static float AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static void PidReset(BallPid_t *pid, float integral_limit,
                     float derivative_filter_hz)
{
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->derivative = 0.0f;
  pid->integral_limit = integral_limit;
  pid->derivative_filter_hz = derivative_filter_hz;
  pid->initialized = 0U;
}

static float PidStep(BallPid_t *pid, float target, float feedback,
                     float kp, float ki, float kd, float dt,
                     float output_limit)
{
  float error = target - feedback;
  float raw_derivative = 0.0f;
  float alpha;

  if (pid->initialized != 0U)
  {
    raw_derivative = (error - pid->last_error) / dt;
    if (pid->derivative_filter_hz > 0.0f)
    {
      alpha = 2.0f * BALL_PI * pid->derivative_filter_hz * dt;
      alpha = alpha / (1.0f + alpha);
      pid->derivative += alpha * (raw_derivative - pid->derivative);
    }
    else
    {
      pid->derivative = raw_derivative;
    }
  }
  else
  {
    pid->initialized = 1U;
  }

  pid->integral += error * dt;
  pid->integral = LimitFloat(pid->integral, pid->integral_limit);
  pid->last_error = error;
  return LimitFloat(kp * error + ki * pid->integral
                    + kd * pid->derivative, output_limit);
}

static float SlewTarget(float current, float target)
{
  const float step = BALL_TARGET_SLEW_MM_S
                     * (0.001f * (float)BALL_CONTROL_PERIOD_MS)
                     * (float)BALL_POSITION_LOOP_DIVIDER;
  float delta = target - current;

  if (delta > step)
  {
    return current + step;
  }
  if (delta < -step)
  {
    return current - step;
  }
  return target;
}

static void ResetControl(void)
{
  PidReset(&s_position_pid, BALL_POSITION_I_LIMIT,
           BALL_POSITION_D_FILTER_HZ);
  PidReset(&s_angle_pid, BALL_ANGLE_I_LIMIT, 0.0f);
  s_position_divider = 0U;
  s_target_angle_adc = 0.0f;
  s_stable_ms = 0U;
  g_ball_debug.breakaway_active = 0U;
}

static void EnterError(BallError_t error)
{
  BallStepper_SetEnabled(0U);
  ResetControl();
  g_ball_debug.error = error;
  g_ball_debug.state = BALL_STATE_ERROR;
}

static void RunAngleLoop(float angle_feedback_adc)
{
  float command_hz;

  command_hz = PidStep(&s_angle_pid,
                       s_target_angle_adc,
                       angle_feedback_adc,
                       g_ball_angle_kp,
                       g_ball_angle_ki,
                       g_ball_angle_kd,
                       0.001f * (float)BALL_CONTROL_PERIOD_MS,
                       BALL_STEPPER_MAX_HZ);
  g_ball_debug.stepper_command_hz = BallStepper_SetCommand(command_hz);
}

static void RunCascade(const BallVisionData_t *vision,
                       float angle_feedback_adc)
{
  float angle_limit;
  float error_abs;
  float position_kd;
  float position_error;
  float position_ki;
  float position_kp;
  float position_output;
  float position_d_scale;
  uint8_t final_capture;

  if (s_position_divider == 0U)
  {
    final_capture = ((g_ball_debug.state == BALL_STATE_SETTLE)
                     || (g_ball_debug.state == BALL_STATE_DONE)) ? 1U : 0U;
    if (final_capture != 0U)
    {
      position_kp = g_ball_final_kp;
      position_ki = g_ball_final_ki;
      position_kd = g_ball_final_kd;
      if (g_ball_debug.state == BALL_STATE_SETTLE)
      {
        angle_limit = AbsFloat(g_ball_final_angle_limit_adc);
      }
      else
      {
        angle_limit = AbsFloat(g_ball_final_hold_angle_limit_adc);
      }
    }
    else
    {
      position_kp = g_ball_position_kp;
      position_ki = g_ball_position_ki;
      position_kd = g_ball_position_kd;
      angle_limit = BALL_ANGLE_TARGET_LIMIT_ADC;
    }
    s_target_position_mm = SlewTarget(s_target_position_mm,
                                      s_requested_position_mm);
    position_output = PidStep(
      &s_position_pid,
      s_target_position_mm,
      vision->position_mm,
      position_kp,
      position_ki,
      0.0f,
      0.001f * (float)BALL_CONTROL_PERIOD_MS
        * (float)BALL_POSITION_LOOP_DIVIDER,
      angle_limit);
    position_error = s_target_position_mm - vision->position_mm;
    error_abs = AbsFloat(position_error);
    position_d_scale = 1.0f;
    if ((final_capture == 0U)
        && ((position_error * vision->velocity_mm_s) > 0.0f))
    {
      if (error_abs >= BALL_POSITION_D_OFF_ERROR_MM)
      {
        position_d_scale = 0.0f;
      }
      else if (error_abs > BALL_POSITION_D_FULL_ERROR_MM)
      {
        position_d_scale =
          (BALL_POSITION_D_OFF_ERROR_MM - error_abs)
          / (BALL_POSITION_D_OFF_ERROR_MM
             - BALL_POSITION_D_FULL_ERROR_MM);
      }
    }
    position_output -= position_kd
                       * vision->velocity_mm_s
                       * position_d_scale;
    g_ball_debug.position_d_scale = position_d_scale;
    g_ball_debug.position_angle_limit_adc = angle_limit;
    g_ball_debug.breakaway_active = 0U;
    if ((final_capture == 0U)
        && (AbsFloat(position_error) >= BALL_BREAKAWAY_ERROR_MM)
        && (AbsFloat(vision->velocity_mm_s)
            <= BALL_BREAKAWAY_SPEED_MM_S))
    {
      position_output += (position_error >= 0.0f)
                         ? g_ball_breakaway_boost_adc
                         : -g_ball_breakaway_boost_adc;
      angle_limit = BALL_BREAKAWAY_ANGLE_LIMIT_ADC;
      g_ball_debug.position_angle_limit_adc = angle_limit;
      g_ball_debug.breakaway_active = 1U;
    }
    s_target_angle_adc = BALL_POSITION_CONTROL_SIGN
                         * LimitFloat(position_output,
                                      angle_limit);
  }
  ++s_position_divider;
  if (s_position_divider >= BALL_POSITION_LOOP_DIVIDER)
  {
    s_position_divider = 0U;
  }
  RunAngleLoop(angle_feedback_adc);
}

static uint8_t IsStable(float position_mm, float velocity_mm_s,
                        float target_mm, float position_tolerance,
                        float speed_tolerance)
{
  return ((AbsFloat(target_mm - position_mm) <= position_tolerance)
          && (AbsFloat(velocity_mm_s) <= speed_tolerance)) ? 1U : 0U;
}

HAL_StatusTypeDef BallBalance_Init(void)
{
  HAL_StatusTypeDef status;

  status = BallAngle_Init();
  if (status != HAL_OK)
  {
    return status;
  }
  status = BallVision_Init();
  if (status != HAL_OK)
  {
    return status;
  }
  status = BallStepper_Init();
  if (status != HAL_OK)
  {
    return status;
  }

  ResetControl();
  s_init_tick = HAL_GetTick();
  s_state_tick = s_init_tick;
  s_sequence_tick = s_init_tick;
  s_target_position_mm = 0.0f;
  s_requested_position_mm = 0.0f;
  s_direction_baseline_adc = 0U;
  s_control_mode = BALL_CONTROL_MODE;
  g_ball_debug.state = (s_control_mode == BALL_MODE_SENSOR_MONITOR)
                       ? BALL_STATE_MONITOR : BALL_STATE_WAIT_START;
  g_ball_debug.error = BALL_ERROR_NONE;
  g_ball_debug.elapsed_ms = 0U;
  return HAL_OK;
}

void BallBalance_Step(uint32_t now_ms)
{
  const BallVisionData_t *vision;
  uint16_t angle_raw = 0U;
  float angle_filtered = 0.0f;
  float angle_feedback;
  uint8_t angle_read_ok;
  uint8_t vision_fresh;

  BallVision_Update(now_ms);
  vision = BallVision_GetData();
  angle_read_ok = BallAngle_Read(&angle_raw, &angle_filtered);
  g_ball_debug.angle_adc_raw = angle_raw;
  g_ball_debug.angle_adc_filtered = angle_filtered;
  g_ball_debug.angle_valid = ((angle_read_ok != 0U)
                              && (angle_raw >= BALL_ANGLE_ADC_VALID_MIN)
                              && (angle_raw <= BALL_ANGLE_ADC_VALID_MAX))
                             ? 1U : 0U;
  angle_feedback = (angle_filtered - (float)BALL_ANGLE_ADC_ZERO)
                   * BALL_ANGLE_SENSOR_SIGN;
  vision_fresh = BallVision_IsFresh(now_ms);

  g_ball_debug.angle_feedback_adc = angle_feedback;
  g_ball_debug.ball_position_mm = vision->position_mm;
  g_ball_debug.ball_velocity_mm_s = vision->velocity_mm_s;
  g_ball_debug.vision_good_frames = vision->good_frames;
  g_ball_debug.vision_parse_errors = vision->parse_errors;
  g_ball_debug.vision_valid = vision_fresh;

  if (s_control_mode == BALL_MODE_SENSOR_MONITOR)
  {
    BallStepper_SetEnabled(0U);
    g_ball_debug.state = BALL_STATE_MONITOR;
    g_ball_debug.error = BALL_ERROR_NONE;
    g_ball_debug.elapsed_ms = 0U;
    g_ball_debug.target_angle_adc = 0.0f;
    g_ball_debug.target_position_mm = 0.0f;
    g_ball_debug.requested_position_mm = 0.0f;
    g_ball_debug.stepper_command_hz = 0;
    g_ball_debug.stepper_enabled = 0U;
    return;
  }

  if (g_ball_debug.state == BALL_STATE_WAIT_START)
  {
    if ((s_control_mode == BALL_MODE_CASCADE)
        && (g_ball_debug.angle_valid != 0U))
    {
      /* Support the beam before vision starts so the ball is not released
         down an unpowered slope. */
      BallStepper_SetEnabled(1U);
      s_target_angle_adc = 0.0f;
      RunAngleLoop(angle_feedback);
    }
    else
    {
      BallStepper_SetEnabled(0U);
    }
    if (((s_control_mode == BALL_MODE_CASCADE)
         && (vision_fresh != 0U)
         && (AbsFloat(vision->position_mm)
             <= BALL_START_POSITION_TOLERANCE_MM))
        || ((s_control_mode != BALL_MODE_CASCADE)
            && ((uint32_t)(now_ms - s_init_tick)
                >= BALL_AUTO_START_DELAY_MS)))
    {
      if (g_ball_debug.angle_valid == 0U)
      {
        EnterError(BALL_ERROR_ANGLE_ADC);
      }
      else if ((s_control_mode == BALL_MODE_CASCADE)
               && (vision_fresh == 0U))
      {
        /* Wait for the first valid MaixCAM frame; this is not an error yet. */
      }
      else
      {
        BallStepper_SetEnabled(1U);
        ResetControl();
        s_state_tick = now_ms;
        s_direction_baseline_adc = angle_raw;
        if (s_control_mode == BALL_MODE_STEPPER_DIRECTION_TEST)
        {
          g_ball_debug.state = BALL_STATE_DIRECTION_TEST;
        }
        else if (s_control_mode == BALL_MODE_ANGLE_LOOP)
        {
          g_ball_debug.state = BALL_STATE_ANGLE_HOLD;
        }
        else
        {
          s_target_position_mm = vision->position_mm;
          s_requested_position_mm = BALL_NEGATIVE_TARGET_MM;
          s_sequence_tick = now_ms;
          g_ball_debug.elapsed_ms = 0U;
          g_ball_debug.state = BALL_STATE_MOVE_NEGATIVE;
          RunCascade(vision, angle_feedback);
        }
      }
    }
  }
  else if (g_ball_debug.state == BALL_STATE_DIRECTION_TEST)
  {
    if ((uint32_t)(now_ms - s_state_tick) < BALL_STEPPER_TEST_MS)
    {
      g_ball_debug.stepper_command_hz =
        BallStepper_SetCommand(BALL_STEPPER_TEST_HZ);
    }
    else
    {
      BallStepper_SetEnabled(0U);
      g_ball_debug.stepper_command_hz = 0;
      g_ball_debug.state = BALL_STATE_DONE;
    }
    g_ball_debug.target_angle_adc =
      (float)((int32_t)angle_raw - (int32_t)s_direction_baseline_adc);
  }
  else
  {
    if (g_ball_debug.angle_valid == 0U)
    {
      EnterError(BALL_ERROR_ANGLE_ADC);
    }
    else if ((s_control_mode == BALL_MODE_CASCADE)
             && (vision_fresh == 0U))
    {
      /* Keep the beam level instead of dropping it on a short camera gap.
         The requested waypoint is preserved and resumed on recovery. */
      PidReset(&s_position_pid, BALL_POSITION_I_LIMIT,
               BALL_POSITION_D_FILTER_HZ);
      s_position_divider = 0U;
      s_target_angle_adc = 0.0f;
      g_ball_debug.error = BALL_ERROR_VISION_TIMEOUT;
      RunAngleLoop(angle_feedback);
      if ((g_ball_debug.state == BALL_STATE_MOVE_POSITIVE)
          || (g_ball_debug.state == BALL_STATE_MOVE_NEGATIVE)
          || (g_ball_debug.state == BALL_STATE_SETTLE))
      {
        g_ball_debug.elapsed_ms = now_ms - s_sequence_tick;
      }
    }
    else if (g_ball_debug.state == BALL_STATE_ANGLE_HOLD)
    {
      s_target_angle_adc = 0.0f;
      RunAngleLoop(angle_feedback);
      g_ball_debug.elapsed_ms = now_ms - s_state_tick;
    }
    else if ((s_control_mode == BALL_MODE_CASCADE)
             && ((g_ball_debug.state == BALL_STATE_MOVE_POSITIVE)
                 || (g_ball_debug.state == BALL_STATE_MOVE_NEGATIVE)
                 || (g_ball_debug.state == BALL_STATE_SETTLE)
                 || (g_ball_debug.state == BALL_STATE_DONE)))
    {
      if (g_ball_debug.error == BALL_ERROR_VISION_TIMEOUT)
      {
        PidReset(&s_position_pid, BALL_POSITION_I_LIMIT,
                 BALL_POSITION_D_FILTER_HZ);
        s_position_divider = 0U;
        s_target_position_mm = vision->position_mm;
      }
      g_ball_debug.error =
        (g_ball_debug.elapsed_ms > BALL_SEQUENCE_TIMEOUT_MS)
        ? BALL_ERROR_SEQUENCE_TIMEOUT : BALL_ERROR_NONE;
      RunCascade(vision, angle_feedback);
      if (g_ball_debug.state != BALL_STATE_DONE)
      {
        g_ball_debug.elapsed_ms = now_ms - s_sequence_tick;
        if (g_ball_debug.elapsed_ms > BALL_SEQUENCE_TIMEOUT_MS)
        {
          /* Keep balancing after the five-second scoring limit.  The error
             flag records the overtime result without disabling the motor. */
          g_ball_debug.error = BALL_ERROR_SEQUENCE_TIMEOUT;
        }
        else
        {
          g_ball_debug.error = BALL_ERROR_NONE;
        }
        if (g_ball_debug.state == BALL_STATE_MOVE_NEGATIVE)
        {
          /* Predict the near-future position from measured ball velocity,
             then reverse early enough for the ball to peak near -5 cm. */
          if ((vision->position_mm <= BALL_NEGATIVE_TARGET_MM)
              || ((vision->velocity_mm_s < 0.0f)
                  && ((vision->position_mm
                       + BALL_FIRST_BRAKE_LOOKAHEAD_S
                         * vision->velocity_mm_s)
                      <= BALL_NEGATIVE_TARGET_MM)))
          {
            s_requested_position_mm = BALL_POSITIVE_TARGET_MM;
            s_target_position_mm = BALL_POSITIVE_TARGET_MM;
            s_stable_ms = 0U;
            PidReset(&s_position_pid, BALL_POSITION_I_LIMIT,
                     BALL_POSITION_D_FILTER_HZ);
            g_ball_debug.state = BALL_STATE_MOVE_POSITIVE;
          }
        }
        else
        {
          if ((g_ball_debug.state == BALL_STATE_MOVE_POSITIVE)
              && ((vision->position_mm >= BALL_FINAL_CAPTURE_ENTRY_MM)
                  || ((vision->position_mm >= 0.0f)
                      && (vision->velocity_mm_s > 0.0f)
                      && ((vision->position_mm
                           + g_ball_final_capture_lookahead_s
                             * vision->velocity_mm_s)
                          >= BALL_FINAL_CAPTURE_ENTRY_MM))))
          {
            s_target_position_mm = BALL_POSITIVE_TARGET_MM;
            s_requested_position_mm = BALL_POSITIVE_TARGET_MM;
            s_position_divider = 0U;
            s_stable_ms = 0U;
            PidReset(&s_position_pid, BALL_POSITION_I_LIMIT,
                     BALL_POSITION_D_FILTER_HZ);
            g_ball_debug.state = BALL_STATE_SETTLE;
          }
          if (IsStable(vision->position_mm, vision->velocity_mm_s,
                       BALL_POSITIVE_TARGET_MM,
                       BALL_FINAL_TOLERANCE_MM,
                       BALL_FINAL_SPEED_MM_S) != 0U)
          {
            s_stable_ms += BALL_CONTROL_PERIOD_MS;
            if (s_stable_ms >= BALL_FINAL_HOLD_MS)
            {
              PidReset(&s_position_pid, BALL_POSITION_I_LIMIT,
                       BALL_POSITION_D_FILTER_HZ);
              s_position_divider = 0U;
              g_ball_debug.state = BALL_STATE_DONE;
            }
          }
          else
          {
            s_stable_ms = 0U;
          }
        }
      }
    }
  }

  g_ball_debug.target_angle_adc = s_target_angle_adc;
  g_ball_debug.target_position_mm = s_target_position_mm;
  g_ball_debug.requested_position_mm = s_requested_position_mm;
  g_ball_debug.stepper_enabled = BallStepper_IsEnabled();
  if ((g_ball_debug.state == BALL_STATE_ERROR)
      || (g_ball_debug.state == BALL_STATE_DONE
          && s_control_mode == BALL_MODE_STEPPER_DIRECTION_TEST))
  {
    g_ball_debug.stepper_command_hz = BallStepper_GetCommand();
  }
}

const BallDebug_t *BallBalance_GetDebug(void)
{
  return (const BallDebug_t *)&g_ball_debug;
}
