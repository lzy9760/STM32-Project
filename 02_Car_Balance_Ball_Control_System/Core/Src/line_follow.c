#include "line_follow.h"

#include "app_config.h"

static int16_t s_last_error;

static int16_t ClampSpeed(int32_t value)
{
  if (value > LINE_MAX_TARGET_SPEED)
  {
    value = LINE_MAX_TARGET_SPEED;
  }
  else if (value < -LINE_MAX_TARGET_SPEED)
  {
    value = -LINE_MAX_TARGET_SPEED;
  }
  return (int16_t)value;
}

void LineFollow_Reset(void)
{
  s_last_error = 0;
}

LineFollowOutput_t LineFollow_Step(const LineSensorFrame_t *frame,
                                   int16_t base_speed)
{
  LineFollowOutput_t output;
  int32_t turn;

  if (frame->detected != 0U)
  {
    int32_t error = frame->error;
    int32_t derivative;

    if ((error >= -LINE_ERROR_DEADBAND)
        && (error <= LINE_ERROR_DEADBAND))
    {
      error = 0;
    }
    derivative = error - (int32_t)s_last_error;
    turn = ((int32_t)LINE_KP_NUM * error
            + (int32_t)LINE_KD_NUM * derivative) / LINE_GAIN_DEN;
    turn *= LINE_STEERING_SIGN;
    s_last_error = (int16_t)error;
  }
  else
  {
    turn = ((s_last_error >= 0) ? base_speed : -base_speed)
           * LINE_STEERING_SIGN;
  }

  output.turn = (int16_t)turn;
  output.left_target = ClampSpeed((int32_t)base_speed + turn);
  output.right_target = ClampSpeed((int32_t)base_speed - turn);
  return output;
}
