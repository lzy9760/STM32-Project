#ifndef __APP_PATH_PLANNER_H
#define __APP_PATH_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_hal.h"

typedef struct
{
  float x_mm;
  float y_mm;
} PathPlanner_Point_t;

typedef struct
{
  PathPlanner_Point_t p0;
  PathPlanner_Point_t p1;
  PathPlanner_Point_t p2;
  PathPlanner_Point_t p3;
  float max_speed_mm_s;
  float min_speed_mm_s;
  float accel_distance_mm;
} PathPlanner_CubicBezier_t;

typedef struct
{
  PathPlanner_Point_t target;
  PathPlanner_Point_t tangent;
  float speed_mm_s;
  float progress;
  uint8_t finished;
} PathPlanner_Sample_t;

void PathPlanner_BuildCubicTo(PathPlanner_CubicBezier_t *path,
                              PathPlanner_Point_t start,
                              PathPlanner_Point_t goal,
                              float start_yaw_deg,
                              float max_speed_mm_s);
PathPlanner_Sample_t PathPlanner_Sample(const PathPlanner_CubicBezier_t *path,
                                        float traveled_mm);
PathPlanner_Point_t PathPlanner_PointAtProgress(const PathPlanner_CubicBezier_t *path,
                                                float progress);
PathPlanner_Point_t PathPlanner_TangentAtProgress(const PathPlanner_CubicBezier_t *path,
                                                  float progress);
float PathPlanner_EstimateLength(const PathPlanner_CubicBezier_t *path);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PATH_PLANNER_H */
