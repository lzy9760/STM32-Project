#include "app_path_planner.h"

#include <math.h>

#define PATH_PI                    3.14159265f
#define PATH_LENGTH_SEGMENTS       24U
#define PATH_MIN_CONTROL_MM        80.0f
#define PATH_MAX_CONTROL_RATIO     0.45f
#define PATH_DEFAULT_MIN_SPEED     70.0f
#define PATH_ACCEL_RATIO           0.25f

static float Path_ClampFloat(float value, float min_value, float max_value)
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

static float Path_Sqrt(float value)
{
  if (value <= 0.0f)
  {
    return 0.0f;
  }

  return sqrtf(value);
}

static float Path_Distance(PathPlanner_Point_t a, PathPlanner_Point_t b)
{
  float dx;
  float dy;

  dx = b.x_mm - a.x_mm;
  dy = b.y_mm - a.y_mm;

  return Path_Sqrt((dx * dx) + (dy * dy));//计算欧几里得距离
}

static PathPlanner_Point_t Path_PointAt(const PathPlanner_CubicBezier_t *path,
                                        float t)
{
  float u;
  float tt;
  float uu;
  float uuu;
  float ttt;
  PathPlanner_Point_t point;

  u = 1.0f - t;
  tt = t * t;
  uu = u * u;
  uuu = uu * u;
  ttt = tt * t;

  point.x_mm = (uuu * path->p0.x_mm) +
               (3.0f * uu * t * path->p1.x_mm) +
               (3.0f * u * tt * path->p2.x_mm) +
               (ttt * path->p3.x_mm);
  point.y_mm = (uuu * path->p0.y_mm) +
               (3.0f * uu * t * path->p1.y_mm) +
               (3.0f * u * tt * path->p2.y_mm) +
               (ttt * path->p3.y_mm);

  return point;
}

static PathPlanner_Point_t Path_TangentAt(const PathPlanner_CubicBezier_t *path,
                                          float t)
{//计算路径上某点的切线向量
  float u;
  PathPlanner_Point_t tangent;

  u = 1.0f - t;
  tangent.x_mm = (3.0f * u * u * (path->p1.x_mm - path->p0.x_mm)) +
                 (6.0f * u * t * (path->p2.x_mm - path->p1.x_mm)) +
                 (3.0f * t * t * (path->p3.x_mm - path->p2.x_mm));
  tangent.y_mm = (3.0f * u * u * (path->p1.y_mm - path->p0.y_mm)) +
                 (6.0f * u * t * (path->p2.y_mm - path->p1.y_mm)) +
                 (3.0f * t * t * (path->p3.y_mm - path->p2.y_mm));

  return tangent;
}

void PathPlanner_BuildCubicTo(PathPlanner_CubicBezier_t *path,
                              PathPlanner_Point_t start,
                              PathPlanner_Point_t goal,
                              float start_yaw_deg,
                              float max_speed_mm_s)
{
  float distance;
  float control;//控制点距离，决定曲线的弯曲程度
  float yaw_rad;
  float goal_dx;
  float goal_dy;
  float goal_len;

  if (path == NULL)//预防空指针
  {
    return;
  }

  distance = Path_Distance(start, goal);//计算起点和目标点之间的直线距离
  control = distance * PATH_MAX_CONTROL_RATIO;//控制点距离为路径长度的45%
  if (control < PATH_MIN_CONTROL_MM)//最小控制距离80mm
  {
    control = PATH_MIN_CONTROL_MM;
  }

  yaw_rad = (start_yaw_deg * PATH_PI) / 180.0f;//将角度转换为弧度
  goal_dx = goal.x_mm - start.x_mm;//目标点与起始点的x轴距离
  goal_dy = goal.y_mm - start.y_mm;//目标点与起始点的y轴距离
  goal_len = Path_Sqrt((goal_dx * goal_dx) + (goal_dy * goal_dy));//计算目标点到起始点的直线距离
  if (goal_len < 1.0f)//防止距离过近
  {
    goal_dx = cosf(yaw_rad);//使用起始航向方向
    goal_dy = sinf(yaw_rad);
    goal_len = 1.0f;
  }

  path->p0 = start;
  path->p3 = goal;
  path->p1.x_mm = start.x_mm + (cosf(yaw_rad) * control);//第一个控制点位于起始点的航向方向上，距离为control
  path->p1.y_mm = start.y_mm + (sinf(yaw_rad) * control);
  path->p2.x_mm = goal.x_mm - ((goal_dx / goal_len) * control);//第二个控制点位于目标点的航向方向上，距离为control
  path->p2.y_mm = goal.y_mm - ((goal_dy / goal_len) * control);
  path->max_speed_mm_s = (max_speed_mm_s > PATH_DEFAULT_MIN_SPEED) ?
                         max_speed_mm_s : PATH_DEFAULT_MIN_SPEED;
  path->min_speed_mm_s = PATH_DEFAULT_MIN_SPEED;
  path->accel_distance_mm = distance * PATH_ACCEL_RATIO;
  if (path->accel_distance_mm < PATH_MIN_CONTROL_MM)
  {
    path->accel_distance_mm = PATH_MIN_CONTROL_MM;
  }
}

PathPlanner_Sample_t PathPlanner_Sample(const PathPlanner_CubicBezier_t *path,
                                        float traveled_mm)
{//根据路径和已行驶距离计算当前目标点、切线向量和速度
  float length;
  float accel_progress;
  float decel_progress;
  float speed_scale;
  PathPlanner_Sample_t sample;

  sample.target.x_mm = 0.0f;
  sample.target.y_mm = 0.0f;
  sample.tangent.x_mm = 0.0f;
  sample.tangent.y_mm = 0.0f;
  sample.speed_mm_s = 0.0f;
  sample.progress = 1.0f;
  sample.finished = 1U;

  if (path == NULL)
  {
    return sample;
  }

  length = PathPlanner_EstimateLength(path);
  if (length < 1.0f)
  {
    sample.target = path->p3;
    sample.tangent = Path_TangentAt(path, 1.0f);
    return sample;
  }

  sample.progress = Path_ClampFloat(traveled_mm / length, 0.0f, 1.0f);
  sample.target = Path_PointAt(path, sample.progress);
  sample.tangent = Path_TangentAt(path, sample.progress);
  sample.finished = (sample.progress >= 1.0f) ? 1U : 0U;

  accel_progress = Path_ClampFloat(traveled_mm / path->accel_distance_mm, 0.0f, 1.0f);
  decel_progress = Path_ClampFloat((length - traveled_mm) / path->accel_distance_mm, 0.0f, 1.0f);
  speed_scale = accel_progress;
  if (decel_progress < speed_scale)
  {
    speed_scale = decel_progress;
  }
  speed_scale = 3.0f * speed_scale * speed_scale -
                2.0f * speed_scale * speed_scale * speed_scale;//使用三次插值函数平滑加速和减速

  sample.speed_mm_s = path->min_speed_mm_s +
                      ((path->max_speed_mm_s - path->min_speed_mm_s) * speed_scale);
  if (sample.finished != 0U)
  {
    sample.speed_mm_s = 0.0f;
  }

  return sample;
}

PathPlanner_Point_t PathPlanner_PointAtProgress(const PathPlanner_CubicBezier_t *path,
                                                float progress)
{//获取路径上某个进度对应的点坐标
  PathPlanner_Point_t point;

  point.x_mm = 0.0f;
  point.y_mm = 0.0f;
  if (path == NULL)
  {
    return point;
  }

  return Path_PointAt(path, Path_ClampFloat(progress, 0.0f, 1.0f));
}

PathPlanner_Point_t PathPlanner_TangentAtProgress(const PathPlanner_CubicBezier_t *path,
                                                  float progress)
{//获取路径上某个进度对应的切线向量
  PathPlanner_Point_t tangent;

  tangent.x_mm = 0.0f;
  tangent.y_mm = 0.0f;
  if (path == NULL)
  {
    return tangent;
  }

  return Path_TangentAt(path, Path_ClampFloat(progress, 0.0f, 1.0f));
}

float PathPlanner_EstimateLength(const PathPlanner_CubicBezier_t *path)
{//通过将路径分成若干段并计算每段的距离来估计路径长度
  uint8_t i;
  float length;
  PathPlanner_Point_t last;
  PathPlanner_Point_t now;

  if (path == NULL)
  {
    return 0.0f;
  }

  length = 0.0f;
  last = path->p0;
  for (i = 1U; i <= PATH_LENGTH_SEGMENTS; i++)//将路径分成24段，计算每段的距离并累加得到总长度
  {
    now = Path_PointAt(path, (float)i / (float)PATH_LENGTH_SEGMENTS);
    length += Path_Distance(last, now);//累加每段的距离
    last = now;
  }

  return length;
}
