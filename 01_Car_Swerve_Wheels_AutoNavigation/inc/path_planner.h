#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include "at32f403a_407_board.h"

#define PATH_BSPLINE_CONTROL_COUNT  9U
#define PATH_BSPLINE_LUT_SEGMENTS   128U

typedef struct
{
    float x_mm;
    float y_mm;
} PathPoint;

typedef struct
{
    PathPoint control[PATH_BSPLINE_CONTROL_COUNT];
    float distance_lut[PATH_BSPLINE_LUT_SEGMENTS + 1U];
    float length_mm;
    float min_speed_mm_s;
    float max_speed_mm_s;
    float accel_distance_mm;
} CubicBSplinePath;

typedef struct
{
    PathPoint point;
    PathPoint tangent;
    float curvature;    /* 曲率，单位 1/mm */
    float speed_mm_s;
    float distance_mm;
    uint8_t finished;
} PathSample;

void PathPlanner_BuildHeartBSpline(CubicBSplinePath *path,
                                   PathPoint start,
                                   float start_yaw_deg,
                                   float forward_scale_mm,
                                   float half_width_mm,
                                   float max_speed_mm_s);
PathSample PathPlanner_SampleByDistance(const CubicBSplinePath *path,
                                        float distance_mm);
float PathPlanner_FindNearestDistance(const CubicBSplinePath *path,
                                      PathPoint position,
                                      float minimum_distance_mm);

#endif
