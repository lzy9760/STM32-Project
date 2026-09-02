#include "app.h"

#include "app_config.h"
#include "ball_balance.h"
#include "chassis.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "main.h"
#include "oled.h"
#include "vofa.h"

#include <string.h>

volatile AppDebug_t g_app_debug;
volatile int16_t g_bench_left_target = 0;
volatile int16_t g_bench_right_target = 0;

static uint32_t s_last_control_tick;
static uint32_t s_last_display_tick;
static uint32_t s_next_oled_init_tick;
static I2C_HandleTypeDef *s_display_i2c;
static AppState_t s_display_state;
static char s_display_time[12];
#if APP_SPEED_PI_BENCH_ENABLE == 0U
static uint32_t s_start_tick;
static uint32_t s_line_lost_since;
static int32_t s_stop_start_distance;
#if APP_AUTO_START_ENABLE == 0U
static uint8_t s_start_debounce;
#endif
static uint8_t s_leave_counter;
static uint8_t s_finish_counter;
#endif

static uint8_t IsActive(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == APP_INPUT_ACTIVE_STATE) ? 1U : 0U;
}

static const char *DisplayStateText(AppState_t state)
{
  switch (state)
  {
    case APP_WAIT_START:
      return "READY ";
    case APP_BENCH_SPEED_PI:
      return "BENCH ";
    case APP_LEAVE_START_LINE:
    case APP_RUNNING:
      return "RUN   ";
    case APP_APPROACH_STOP:
      return "FINISH";
    case APP_FINISHED:
      return "DONE  ";
    case APP_ERROR:
    default:
      return "LOST  ";
  }
}

static void BuildTimeText(uint32_t elapsed_ms, char text[12])
{
  uint32_t seconds = elapsed_ms / 1000U;

  if (seconds > 999U)
  {
    seconds = 999U;
    elapsed_ms = 999900U;
  }
  memcpy(text, "TIME 000.0S", 12U);
  text[5] = (char)('0' + ((seconds / 100U) % 10U));
  text[6] = (char)('0' + ((seconds / 10U) % 10U));
  text[7] = (char)('0' + (seconds % 10U));
  text[9] = (char)('0' + ((elapsed_ms / 100U) % 10U));
}

static void DisplayTask(uint32_t now)
{
  char text[12];
  char changed[12];
  uint8_t first = 11U;
  uint8_t last = 0U;
  uint8_t i;
  uint8_t count;
  uint8_t state_changed;

  if (Oled_IsPresent() == 0U)
  {
    g_app_debug.oled_present = 0U;
    if ((g_app_debug.state != APP_WAIT_START)
        || ((int32_t)(now - s_next_oled_init_tick) < 0))
    {
      return;
    }
    if (Oled_Init(s_display_i2c) != HAL_OK)
    {
      s_next_oled_init_tick = now + 500U;
      return;
    }
    s_display_state = (AppState_t)0xFF;
    memset(s_display_time, 0, sizeof(s_display_time));
  }
  g_app_debug.oled_present = 1U;

  state_changed = (g_app_debug.state != s_display_state) ? 1U : 0U;
  if (state_changed != 0U)
  {
    Oled_WriteText(0U, 0U, DisplayStateText(g_app_debug.state));
    s_display_state = g_app_debug.state;
  }

  if ((state_changed == 0U)
      && ((uint32_t)(now - s_last_display_tick) < 100U))
  {
    return;
  }
  s_last_display_tick = now;
  BuildTimeText(g_app_debug.elapsed_ms, text);

  for (i = 0U; i < 11U; ++i)
  {
    if (text[i] != s_display_time[i])
    {
      if (first == 11U)
      {
        first = i;
      }
      last = i;
    }
  }
  if (first == 11U)
  {
    return;
  }

  count = (uint8_t)(last - first + 1U);
  memcpy(changed, &text[first], count);
  changed[count] = '\0';
  Oled_WriteText(3U, first, changed);
  memcpy(s_display_time, text, sizeof(s_display_time));
}

#if (APP_SPEED_PI_BENCH_ENABLE == 0U) && (APP_AUTO_START_ENABLE == 0U)
static uint8_t StartPressed(void)
{
  if (IsActive(START_KEY_GPIO_Port, START_KEY_Pin) != 0U)
  {
    if (s_start_debounce < APP_INPUT_DEBOUNCE_TICKS)
    {
      ++s_start_debounce;
    }
  }
  else
  {
    s_start_debounce = 0U;
  }
  return (s_start_debounce >= APP_INPUT_DEBOUNCE_TICKS) ? 1U : 0U;
}
#endif

#if APP_SPEED_PI_BENCH_ENABLE == 0U
static void StartRun(uint32_t now)
{
  Chassis_Reset();
  LineFollow_Reset();
  s_start_tick = now;
  s_line_lost_since = 0U;
  s_leave_counter = 0U;
  s_finish_counter = 0U;
  g_app_debug.elapsed_ms = 0U;
  g_app_debug.final_time_ms = 0U;
  g_app_debug.max_black_count = 0U;
  g_app_debug.state = APP_LEAVE_START_LINE;
}

static void FollowStep(int16_t base_speed, const LineSensorFrame_t *frame)
{
  LineFollowOutput_t output = LineFollow_Step(frame, base_speed);
  Chassis_SetSpeedTarget(output.left_target, output.right_target);
  Chassis_ControlStep();
}

static int16_t RunSpeed(void)
{
  uint32_t elapsed = g_app_debug.elapsed_ms;
  int32_t speed;

  if (elapsed >= LINE_START_RAMP_MS)
  {
    return LINE_BASE_SPEED;
  }
  speed = LINE_START_SPEED
          + ((int32_t)(LINE_BASE_SPEED - LINE_START_SPEED)
             * (int32_t)elapsed) / (int32_t)LINE_START_RAMP_MS;
  return (int16_t)speed;
}
#endif

static void UpdateDebug(const LineSensorFrame_t *frame)
{
  const ChassisDebug_t *chassis = Chassis_GetDebug();

  g_app_debug.line_error = frame->error;
  g_app_debug.line_strength = frame->strength;
  g_app_debug.line_peak = frame->peak;
  g_app_debug.line_black_count = frame->black_count;
  g_app_debug.cross_detected = frame->cross_detected;
  g_app_debug.line_detected = frame->detected;
  g_app_debug.finish_left = IsActive(FINISH_L_GPIO_Port, FINISH_L_Pin);
  g_app_debug.finish_right = IsActive(FINISH_R_GPIO_Port, FINISH_R_Pin);
  g_app_debug.left_target = chassis->left_target;
  g_app_debug.right_target = chassis->right_target;
  g_app_debug.left_speed_raw = chassis->left_speed_raw;
  g_app_debug.right_speed_raw = chassis->right_speed_raw;
  g_app_debug.left_speed = chassis->left_speed;
  g_app_debug.right_speed = chassis->right_speed;
  g_app_debug.left_duty = chassis->left_duty;
  g_app_debug.right_duty = chassis->right_duty;
  Vofa_SendWheelSpeed((float)chassis->left_target,
                      chassis->left_speed,
                      (float)chassis->right_target,
                      chassis->right_speed);
}

static void LineControlTask(uint32_t now)
{
  const LineSensorFrame_t *frame;
#if APP_SPEED_PI_BENCH_ENABLE == 0U
  uint8_t finish_both;
  uint8_t finish_none;
#endif

  LineSensor_Update();
  frame = LineSensor_GetFrame();
#if APP_SPEED_PI_BENCH_ENABLE == 0U
  if ((g_app_debug.state != APP_WAIT_START)
      && (g_app_debug.state != APP_FINISHED)
      && (g_app_debug.state != APP_ERROR)
      && (frame->black_count > g_app_debug.max_black_count))
  {
    g_app_debug.max_black_count = frame->black_count;
  }
  finish_both = frame->cross_detected;
  finish_none = (frame->cross_detected == 0U) ? 1U : 0U;
#endif

#if APP_SPEED_PI_BENCH_ENABLE != 0U
  g_app_debug.state = APP_BENCH_SPEED_PI;
  g_app_debug.elapsed_ms = now;
  Chassis_SetSpeedTarget(g_bench_left_target, g_bench_right_target);
  Chassis_ControlStep();
  UpdateDebug(frame);
#else

  if ((g_app_debug.state != APP_WAIT_START)
      && (g_app_debug.state != APP_FINISHED)
      && (g_app_debug.state != APP_ERROR))
  {
    g_app_debug.elapsed_ms = now - s_start_tick;
  }

  switch (g_app_debug.state)
  {
    case APP_WAIT_START:
      Chassis_Coast();
#if APP_AUTO_START_ENABLE != 0U
      if (now >= APP_AUTO_START_DELAY_MS)
#else
      if (StartPressed() != 0U)
#endif
      {
        StartRun(now);
      }
      break;

    case APP_LEAVE_START_LINE:
      FollowStep(RunSpeed(), frame);
      if (finish_none != 0U)
      {
        if (++s_leave_counter >= APP_LEAVE_LINE_TICKS)
        {
          g_app_debug.state = APP_RUNNING;
        }
      }
      else
      {
        s_leave_counter = 0U;
      }
      break;

    case APP_RUNNING:
      FollowStep(RunSpeed(), frame);
      if ((g_app_debug.elapsed_ms >= APP_FINISH_ARM_TIME_MS)
          && (finish_both != 0U))
      {
        if (++s_finish_counter >= APP_FINISH_LINE_TICKS)
        {
          s_stop_start_distance = Chassis_GetAverageDistance();
          g_app_debug.state = APP_APPROACH_STOP;
        }
      }
      else
      {
        s_finish_counter = 0U;
      }
      break;

    case APP_APPROACH_STOP:
      if ((Chassis_GetAverageDistance() - s_stop_start_distance)
          >= APP_STOP_ADVANCE_COUNTS)
      {
        Chassis_Brake();
        g_app_debug.final_time_ms = now - s_start_tick;
        g_app_debug.elapsed_ms = g_app_debug.final_time_ms;
        g_app_debug.state = APP_FINISHED;
      }
      else
      {
        FollowStep(LINE_APPROACH_SPEED, frame);
      }
      break;

    case APP_FINISHED:
      Chassis_Brake();
      break;

    case APP_ERROR:
    default:
      Chassis_Brake();
      break;
  }

  if ((g_app_debug.state == APP_LEAVE_START_LINE)
      || (g_app_debug.state == APP_RUNNING)
      || (g_app_debug.state == APP_APPROACH_STOP))
  {
    if (frame->detected != 0U)
    {
      s_line_lost_since = 0U;
    }
    else if (s_line_lost_since == 0U)
    {
      s_line_lost_since = now;
    }
    else if ((now - s_line_lost_since) >= APP_LINE_LOST_STOP_MS)
    {
      Chassis_Brake();
      g_app_debug.state = APP_ERROR;
    }
  }

  UpdateDebug(frame);
#endif
}

static void BallControlTask(uint32_t now)
{
  const BallDebug_t *ball;

  BallBalance_Step(now);
  ball = BallBalance_GetDebug();
  g_app_debug.elapsed_ms = ball->elapsed_ms;
  if ((ball->state == BALL_STATE_MONITOR)
      || (ball->state == BALL_STATE_WAIT_START))
  {
    g_app_debug.state = APP_WAIT_START;
  }
  else if (ball->state == BALL_STATE_DONE)
  {
    g_app_debug.final_time_ms = ball->elapsed_ms;
    g_app_debug.state = APP_FINISHED;
  }
  else if (ball->state == BALL_STATE_ERROR)
  {
    g_app_debug.state = APP_ERROR;
  }
  else
  {
    g_app_debug.state = APP_RUNNING;
  }

  Vofa_Send4(ball->target_position_mm,
             ball->ball_position_mm,
             ball->target_angle_adc,
             ball->angle_feedback_adc);
}

static void ControlTask(uint32_t now)
{
#if APP_ACTIVE_TASK == APP_TASK_BALL_STATIC
  (void)&LineControlTask;
  BallControlTask(now);
#else
  LineControlTask(now);
#endif
}

HAL_StatusTypeDef App_Init(ADC_HandleTypeDef *hadc,
                           TIM_HandleTypeDef *adc_trigger_timer,
                           TIM_HandleTypeDef *pwm_timer,
                           TIM_HandleTypeDef *left_encoder,
                           TIM_HandleTypeDef *right_encoder,
                           I2C_HandleTypeDef *display_i2c,
                           UART_HandleTypeDef *debug_uart)
{
  HAL_StatusTypeDef status;

  status = Chassis_Init(pwm_timer, left_encoder, right_encoder);
  if (status != HAL_OK)
  {
    return status;
  }
  status = LineSensor_Init(hadc, adc_trigger_timer);
  if (status != HAL_OK)
  {
    return status;
  }
  Vofa_Init(debug_uart);
  s_display_i2c = display_i2c;

#if APP_ACTIVE_TASK == APP_TASK_BALL_STATIC
  status = BallBalance_Init();
  if (status != HAL_OK)
  {
    return status;
  }
#endif

#if APP_ACTIVE_TASK == APP_TASK_BALL_STATIC
  g_app_debug.state = APP_WAIT_START;
#elif APP_SPEED_PI_BENCH_ENABLE != 0U
  g_app_debug.state = APP_BENCH_SPEED_PI;
#else
  g_app_debug.state = APP_WAIT_START;
#endif
  g_app_debug.elapsed_ms = 0U;
  g_app_debug.final_time_ms = 0U;
  s_last_control_tick = HAL_GetTick();
  s_last_display_tick = s_last_control_tick;
  s_next_oled_init_tick = s_last_control_tick + 200U;
  s_display_state = (AppState_t)0xFF;
  memset(s_display_time, 0, sizeof(s_display_time));
  g_app_debug.oled_present = 0U;
  DisplayTask(s_last_control_tick);
  return HAL_OK;
}

void App_Run(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - s_last_control_tick) >= APP_CONTROL_PERIOD_MS)
  {
    s_last_control_tick += APP_CONTROL_PERIOD_MS;
    ControlTask(now);
  }
  DisplayTask(now);
}

const AppDebug_t *App_GetDebug(void)
{
  return (const AppDebug_t *)&g_app_debug;
}
