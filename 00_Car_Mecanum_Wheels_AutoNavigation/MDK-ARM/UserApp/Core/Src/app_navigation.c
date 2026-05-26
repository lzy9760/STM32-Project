#include "app_navigation.h"

#include "app_chassis.h"
#include "app_encoder.h"
#include "app_mpu6050.h"
#include "app_path_planner.h"
#include "pid.h"

#include <math.h>

#define NAV_TASK_PERIOD_MS              5U
#define NAV_LOOKAHEAD_MM                100.0f
#define NAV_POSITION_TOL_MM             25.0f
#define NAV_MIN_TARGET_SPEED            70.0f
#define NAV_MAX_TARGET_SPEED            420.0f
#define NAV_CROSS_TRACK_KP              0.009f
#define NAV_GOAL_TRACK_KP               0.0020f
#define NAV_YAW_KP                      0.010f
#define NAV_YAW_LIMIT                   0.28f
#define NAV_YAW_DEADBAND_DEG            2.0f
#define NAV_YAW_SIGN                    1.0f
#define NAV_PROGRESS_SEARCH_STEPS       24U
#define NAV_FOLLOW_PATH_YAW_ENABLE      1U
#define NAV_CURVE_YAW_ENABLE_DEG        12.0f
#define NAV_FINAL_APPROACH_MM           300.0f
#define NAV_FINAL_POS_KP                0.0040f
#define NAV_FINAL_SPEED_LIMIT           0.35f
#define NAV_FINAL_YAW_DISABLE_MM        80.0f
#define NAV_MOVING_CMD_EPS              0.01f
#define NAV_MOVING_YAW_EPS              0.03f
#define NAV_PI                          3.14159265f

typedef struct
{
  uint8_t active;
  uint8_t follow_path_yaw;
  PathPlanner_CubicBezier_t path;
  PathPlanner_Point_t start;
  PathPlanner_Point_t goal;
  float path_length_mm;
  float line_length_mm;
  float traveled_mm;
  float start_yaw_deg;
  float straight_forward_x;
  float straight_forward_y;
  float straight_left_x;
  float straight_left_y;
  uint32_t last_update_tick;
} Navigation_State_t;

static Navigation_State_t g_nav;

static float Nav_Abs(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float Nav_Sqrt(float value)
{
  if (value <= 0.0f)
  {
    return 0.0f;
  }

  return sqrtf(value);
}

static float Nav_ClampFloat(float value, float min_value, float max_value)
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

static float Nav_WrapDeg(float angle)
{
  while (angle > 180.0f)
  {
    angle -= 360.0f;
  }
  while (angle < -180.0f)
  {
    angle += 360.0f;
  }

  return angle;
}

static float Nav_DistanceTo(PathPlanner_Point_t point, Odometry_Pose_t pose)
{
  float dx;
  float dy;

  dx = point.x_mm - pose.x_mm;
  dy = point.y_mm - pose.y_mm;

  return Nav_Sqrt((dx * dx) + (dy * dy));
}

static float Nav_GetYawCommand(float target_yaw_deg, float current_yaw_deg)
{//目标偏航角与当前偏航角的比例控制
  float heading_error;

  heading_error = Nav_WrapDeg(target_yaw_deg - current_yaw_deg);//角度归一化
  if (Nav_Abs(heading_error) <= NAV_YAW_DEADBAND_DEG)
  {//当偏航误差足够小时，直接返回0，避免过度修正导致震荡
    return 0.0f;
  }

  return Nav_ClampFloat(heading_error * NAV_YAW_KP * NAV_YAW_SIGN,
                        -NAV_YAW_LIMIT,
                        NAV_YAW_LIMIT);//公式：偏航误差*比例增益*符号，结果限制在[-NAV_YAW_LIMIT, NAV_YAW_LIMIT]范围内
}

static uint8_t Nav_HasMotionCommand(float forward_cmd, float strafe_cmd, float yaw_cmd)
{
  return ((Nav_Abs(forward_cmd) > NAV_MOVING_CMD_EPS) ||
          (Nav_Abs(strafe_cmd) > NAV_MOVING_CMD_EPS) ||
          (Nav_Abs(yaw_cmd) > NAV_MOVING_YAW_EPS)) ? 1U : 0U;
}

static float Nav_GetStraightTravel(const Odometry_Pose_t *pose)
{//计算当前位置到目标位置的直线距离
  float start_to_pose_x;
  float start_to_pose_y;
  float travel;

  start_to_pose_x = pose->x_mm - g_nav.start.x_mm;//当前位置与起始点的x轴距离
  start_to_pose_y = pose->y_mm - g_nav.start.y_mm;//当前位置与起始点的y轴距离
  travel = (start_to_pose_x * g_nav.straight_forward_x) +
           (start_to_pose_y * g_nav.straight_forward_y);//计算当前位置到目标位置的直线距离

  return Nav_ClampFloat(travel, 0.0f, g_nav.line_length_mm);
}

static float Nav_FindBezierTravel(const Odometry_Pose_t *pose)
{//在贝塞尔曲线中搜索当前位置对应的行驶距离
  uint8_t i;
  float progress;
  float distance;
  float best_distance;
  float best_travel;
  PathPlanner_Point_t point;

  best_distance = 1000000000.0f;
  best_travel = g_nav.traveled_mm;

  for (i = 0U; i <= NAV_PROGRESS_SEARCH_STEPS; i++)
  {
    progress = (float)i / (float)NAV_PROGRESS_SEARCH_STEPS;//计算进度值
    point = PathPlanner_PointAtProgress(&g_nav.path, progress);//获取路径上的点
    distance = Nav_DistanceTo(point, *pose);//计算欧几里得距离
    if (distance < best_distance)//更新最小距离和对应的行驶距离
    {
      best_distance = distance;//更新最小距离
      best_travel = progress * g_nav.path_length_mm;//计算对应的行驶距离
    }
  }

  if (best_travel < g_nav.traveled_mm)//防后退检查
  {
    return g_nav.traveled_mm;
  }

  return best_travel;
}

static void Nav_RunFinalApproach(const Odometry_Pose_t *pose,
                                 float goal_dx_world,
                                 float goal_dy_world,
                                 float goal_distance)
{
  float dx_world;
  float dy_world;
  float dx_body;
  float dy_body;
  float yaw_cmd;
  float yaw_scale;
  float cos_yaw;
  float sin_yaw;

  cos_yaw = cosf((pose->yaw_deg * NAV_PI) / 180.0f);
  sin_yaw = sinf((pose->yaw_deg * NAV_PI) / 180.0f);

  dx_world = Nav_ClampFloat(goal_dx_world * NAV_FINAL_POS_KP,
                            -NAV_FINAL_SPEED_LIMIT,
                            NAV_FINAL_SPEED_LIMIT);
  dy_world = Nav_ClampFloat(goal_dy_world * NAV_FINAL_POS_KP,
                            -NAV_FINAL_SPEED_LIMIT,
                            NAV_FINAL_SPEED_LIMIT);

  dx_body = (dx_world * cos_yaw) + (dy_world * sin_yaw);
  dy_body = (-dx_world * sin_yaw) + (dy_world * cos_yaw);
  yaw_cmd = Nav_GetYawCommand(g_nav.start_yaw_deg, pose->yaw_deg);

  if (goal_distance <= NAV_FINAL_YAW_DISABLE_MM)
  {
    yaw_cmd = 0.0f;
  }
  else
  {
    yaw_scale = Nav_ClampFloat((goal_distance - NAV_FINAL_YAW_DISABLE_MM) /
                               (NAV_FINAL_APPROACH_MM - NAV_FINAL_YAW_DISABLE_MM),
                               0.0f,
                               1.0f);
    yaw_cmd *= yaw_scale;
  }

  MPU6050_NotifyMoving(Nav_HasMotionCommand(dx_body, dy_body, yaw_cmd));
  Chassis_SetNormalizedMotion(dx_body, dy_body, yaw_cmd);
}

void Navigation_Init(void)
{
  Odometry_Init();
  g_nav.active = 0U;
  g_nav.follow_path_yaw = 0U;
  g_nav.start.x_mm = 0.0f;
  g_nav.start.y_mm = 0.0f;
  g_nav.goal.x_mm = 0.0f;
  g_nav.goal.y_mm = 0.0f;
  g_nav.path_length_mm = 0.0f;
  g_nav.line_length_mm = 0.0f;
  g_nav.traveled_mm = 0.0f;
  g_nav.start_yaw_deg = 0.0f;
  g_nav.straight_forward_x = 1.0f;
  g_nav.straight_forward_y = 0.0f;
  g_nav.straight_left_x = 0.0f;
  g_nav.straight_left_y = 1.0f;
  g_nav.last_update_tick = HAL_GetTick();
}

void Navigation_StartTo(float target_x_mm, float target_y_mm, float max_speed_mm_s)
{
  Odometry_Pose_t pose;
  PathPlanner_Point_t start;
  PathPlanner_Point_t goal;
  float goal_dx;
  float goal_dy;
  float goal_heading_deg;
  float goal_heading_error;

  Odometry_Update();
  pose = Odometry_GetPose();

  start.x_mm = pose.x_mm;
  start.y_mm = pose.y_mm;
  goal.x_mm = target_x_mm;
  goal.y_mm = target_y_mm;

  PathPlanner_BuildCubicTo(&g_nav.path, start, goal, pose.yaw_deg, max_speed_mm_s);
  g_nav.start = start;
  g_nav.goal = goal;
  g_nav.path_length_mm = PathPlanner_EstimateLength(&g_nav.path);
  g_nav.traveled_mm = 0.0f;
  g_nav.start_yaw_deg = pose.yaw_deg;

  goal_dx = goal.x_mm - start.x_mm;
  goal_dy = goal.y_mm - start.y_mm;
  g_nav.line_length_mm = Nav_Sqrt((goal_dx * goal_dx) + (goal_dy * goal_dy));
  if (g_nav.line_length_mm < 1.0f)
  {
    g_nav.line_length_mm = 1.0f;
  }

  g_nav.straight_forward_x = goal_dx / g_nav.line_length_mm;
  g_nav.straight_forward_y = goal_dy / g_nav.line_length_mm;
  g_nav.straight_left_x = -g_nav.straight_forward_y;
  g_nav.straight_left_y = g_nav.straight_forward_x;

  goal_heading_deg = atan2f(goal_dy, goal_dx) * 180.0f / NAV_PI;
  goal_heading_error = Nav_WrapDeg(goal_heading_deg - pose.yaw_deg);
#if (NAV_FOLLOW_PATH_YAW_ENABLE != 0U)
  g_nav.follow_path_yaw = (Nav_Abs(goal_heading_error) > NAV_CURVE_YAW_ENABLE_DEG) ? 1U : 0U;
#else
  (void)goal_heading_error;
  g_nav.follow_path_yaw = 0U;
#endif

  g_nav.last_update_tick = HAL_GetTick();
  g_nav.active = 1U;
}

void Navigation_Stop(void)
{
  g_nav.active = 0U;
  Pid_StopAll();
}

uint8_t Navigation_IsActive(void)
{
  return g_nav.active;
}

Odometry_Pose_t Navigation_GetPose(void)
{
  return Odometry_GetPose();
}

void Navigation_Task(void)
{//导航控制任务，周期执行
  uint32_t now_tick;
  float dt;
  float lookahead_travel;
  float speed_scale;
  float dx_world;
  float dy_world;
  float goal_dx_world;
  float goal_dy_world;
  float goal_distance;
  float path_forward_x;
  float path_forward_y;
  float path_left_x;
  float path_left_y;
  float along_error;
  float cross_error;
  float cross_cmd;
  float goal_cross_error;
  float dx_body;
  float dy_body;
  float desired_heading;
  float yaw_target_deg;
  float yaw_cmd;
  float cos_yaw;
  float sin_yaw;
  float start_to_pose_x;
  float start_to_pose_y;
  Odometry_Pose_t pose;
  PathPlanner_Sample_t sample;

  if (g_nav.active == 0U)
  {
    return;
  }

  MPU6050_Update();
  Odometry_Update();

  now_tick = HAL_GetTick();
  dt = (float)(now_tick - g_nav.last_update_tick) / 1000.0f;
  g_nav.last_update_tick = now_tick;
  if ((dt <= 0.0f) || (dt > 0.2f))
  {
    dt = (float)NAV_TASK_PERIOD_MS / 1000.0f;
  }

  pose = Odometry_GetPose();
  goal_dx_world = g_nav.goal.x_mm - pose.x_mm;
  goal_dy_world = g_nav.goal.y_mm - pose.y_mm;
  goal_distance = Nav_Sqrt((goal_dx_world * goal_dx_world) +
                           (goal_dy_world * goal_dy_world));

  if (g_nav.follow_path_yaw != 0U)
  {
    g_nav.traveled_mm = Nav_FindBezierTravel(&pose);//贝塞尔曲线路径
  }
  else
  {
    g_nav.traveled_mm = Nav_GetStraightTravel(&pose);//直线路径
  }

  if (goal_distance <= NAV_POSITION_TOL_MM)
  {//到达目标点足够近的时候，停止导航
    Navigation_Stop();
    return;
  }

  if ((goal_distance <= NAV_FINAL_APPROACH_MM) ||
      (g_nav.traveled_mm >= g_nav.line_length_mm))
  {//切换精确接近控制模式
    Nav_RunFinalApproach(&pose, goal_dx_world, goal_dy_world, goal_distance);
    return;
  }

  sample = PathPlanner_Sample(&g_nav.path, g_nav.traveled_mm);//速度规划
  speed_scale = sample.speed_mm_s / NAV_MAX_TARGET_SPEED;//速度归一化
  speed_scale = Nav_ClampFloat(speed_scale, 0.0f, 1.0f);
  if (speed_scale < (NAV_MIN_TARGET_SPEED / NAV_MAX_TARGET_SPEED))
  {
    speed_scale = NAV_MIN_TARGET_SPEED / NAV_MAX_TARGET_SPEED;
  }

  if (g_nav.follow_path_yaw != 0U)
  {//路径跟踪算法(Pure Pursuit)
    lookahead_travel = g_nav.traveled_mm + NAV_LOOKAHEAD_MM;//前瞻距离
    if (lookahead_travel > g_nav.path_length_mm)
    {
      lookahead_travel = g_nav.path_length_mm;
    }
    sample.target = PathPlanner_Sample(&g_nav.path, lookahead_travel).target;//目标点

    dx_world = sample.target.x_mm - pose.x_mm;//目标点与当前位置的x轴距离
    dy_world = sample.target.y_mm - pose.y_mm;//目标点与当前位置的y轴距离
    desired_heading = atan2f(sample.tangent.y_mm, sample.tangent.x_mm) * 180.0f / NAV_PI;//目标点方向
    path_forward_x = cosf((desired_heading * NAV_PI) / 180.0f);//目标点方向的x轴分量
    path_forward_y = sinf((desired_heading * NAV_PI) / 180.0f);//目标点方向的y轴分量
    path_left_x = -path_forward_y;
    path_left_y = path_forward_x;
    along_error = (dx_world * path_forward_x) + (dy_world * path_forward_y);
    cross_error = (dx_world * path_left_x) + (dy_world * path_left_y);
    goal_cross_error = (goal_dx_world * path_left_x) + (goal_dy_world * path_left_y);
    yaw_target_deg = desired_heading;//目标点方向
  }
  else
  {//直线路径
    lookahead_travel = g_nav.traveled_mm + NAV_LOOKAHEAD_MM;//前瞻距离
    if (lookahead_travel > g_nav.line_length_mm)
    {
      lookahead_travel = g_nav.line_length_mm;
    }

    path_forward_x = g_nav.straight_forward_x;
    path_forward_y = g_nav.straight_forward_y;
    path_left_x = g_nav.straight_left_x;
    path_left_y = g_nav.straight_left_y;
    along_error = lookahead_travel - g_nav.traveled_mm;
    start_to_pose_x = pose.x_mm - g_nav.start.x_mm;//当前位置与目标点的y轴距离
    start_to_pose_y = pose.y_mm - g_nav.start.y_mm;//当前位置与目标点的x轴距离
    cross_error = -((start_to_pose_x * path_left_x) + (start_to_pose_y * path_left_y));//当前位置与目标点的x轴距离
    goal_cross_error = (goal_dx_world * path_left_x) + (goal_dy_world * path_left_y);//目标点与当前位置的x轴距离
    yaw_target_deg = g_nav.start_yaw_deg;
  }

  cross_cmd = Nav_ClampFloat((cross_error * NAV_CROSS_TRACK_KP) +
                             (goal_cross_error * NAV_GOAL_TRACK_KP),
                             -0.35f, 0.35f);

  dx_world = (path_forward_x * speed_scale) +
             (path_forward_x * Nav_ClampFloat(along_error * NAV_GOAL_TRACK_KP, -0.10f, 0.10f)) +
             (path_left_x * cross_cmd);//基础向前速度+前进误差修正+横向误差修正
  dy_world = (path_forward_y * speed_scale) +
             (path_forward_y * Nav_ClampFloat(along_error * NAV_GOAL_TRACK_KP, -0.10f, 0.10f)) +
             (path_left_y * cross_cmd);

  cos_yaw = cosf((pose.yaw_deg * NAV_PI) / 180.0f);//当前偏航角的余弦值
  sin_yaw = sinf((pose.yaw_deg * NAV_PI) / 180.0f);//当前偏航角的正弦值
  dx_body = (dx_world * cos_yaw) + (dy_world * sin_yaw);
  dy_body = (-dx_world * sin_yaw) + (dy_world * cos_yaw);
  yaw_cmd = Nav_GetYawCommand(yaw_target_deg, pose.yaw_deg);//目标偏航角-当前偏航角的比例控制

  MPU6050_NotifyMoving(Nav_HasMotionCommand(dx_body, dy_body, yaw_cmd));
  Chassis_SetNormalizedMotion(dx_body, dy_body, yaw_cmd);

  g_nav.traveled_mm += sample.speed_mm_s * dt * 0.25f;//更新已行驶距离
}
