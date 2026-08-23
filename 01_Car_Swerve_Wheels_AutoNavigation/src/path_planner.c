#include "path_planner.h"

#include <math.h>

#define PATH_PI                 3.14159265358979323846f
#define PATH_MIN_SPEED_MM_S     70.0f
#define PATH_ACCEL_RATIO        0.25f
#define PATH_MIN_ACCEL_MM       100.0f
#define PATH_MAX_ACCEL_MM       400.0f
#define PATH_TANGENT_STEP       0.002f
#define PATH_S_LENGTH           150.0f

static const float g_knots[10] =
{
    0.0f, 0.0f, 0.0f, 0.0f,
    0.33333333f, 0.66666667f,
    1.0f, 1.0f, 1.0f, 1.0f
};

static float Path_Clamp(float value, float minimum, float maximum)
{
    if(value < minimum)
    {
        return minimum;
    }
    if(value > maximum)
    {
        return maximum;
    }
    return value;
}

static float Path_Distance(PathPoint a, PathPoint b)
{
    float dx;
    float dy;

    dx = b.x_mm - a.x_mm;
    dy = b.y_mm - a.y_mm;
    return sqrtf((dx * dx) + (dy * dy));
}

static PathPoint Path_PointAt(const CubicBSplinePath *path, float parameter)
{
    PathPoint d[4];
    PathPoint result;
    int span;
    int j;
    int r;
    int control_index;
    float alpha;
    float denominator;

    parameter = Path_Clamp(parameter, 0.0f, 1.0f);
    if(parameter < g_knots[4])
    {
        span = 3;
    }
    else if(parameter < g_knots[5])
    {
        span = 4;
    }
    else
    {
        span = 5;
    }

    for(j = 0; j <= 3; j++)
    {
        control_index = span - 3 + j;
        d[j] = path->control[control_index];
    }

    for(r = 1; r <= 3; r++)
    {
        for(j = 3; j >= r; j--)
        {
            denominator = g_knots[j + 1 + span - r] -
                          g_knots[j + span - 3];
            if(denominator > 0.000001f)
            {
                alpha = (parameter - g_knots[j + span - 3]) / denominator;
            }
            else
            {
                alpha = 0.0f;
            }
            d[j].x_mm = ((1.0f - alpha) * d[j - 1].x_mm) +
                        (alpha * d[j].x_mm);
            d[j].y_mm = ((1.0f - alpha) * d[j - 1].y_mm) +
                        (alpha * d[j].y_mm);
        }
    }

    result = d[3];
    return result;
}

static PathPoint Path_TangentAt(const CubicBSplinePath *path, float parameter)
{
    PathPoint before;
    PathPoint after;
    PathPoint tangent;
    float before_parameter;
    float after_parameter;
    float length;

    before_parameter = Path_Clamp(parameter - PATH_TANGENT_STEP, 0.0f, 1.0f);
    after_parameter = Path_Clamp(parameter + PATH_TANGENT_STEP, 0.0f, 1.0f);
    before = Path_PointAt(path, before_parameter);
    after = Path_PointAt(path, after_parameter);
    tangent.x_mm = after.x_mm - before.x_mm;
    tangent.y_mm = after.y_mm - before.y_mm;
    length = sqrtf((tangent.x_mm * tangent.x_mm) +
                   (tangent.y_mm * tangent.y_mm));
    if(length > 0.0001f)
    {
        tangent.x_mm /= length;
        tangent.y_mm /= length;
    }
    return tangent;
}

static float Path_ParameterAtDistance(const CubicBSplinePath *path,
                                      float distance_mm)
{
    uint8_t i;
    float segment_length;
    float fraction;

    distance_mm = Path_Clamp(distance_mm, 0.0f, path->length_mm);
    for(i = 1U; i <= PATH_BSPLINE_LUT_SEGMENTS; i++)
    {
        if(path->distance_lut[i] >= distance_mm)
        {
            segment_length = path->distance_lut[i] - path->distance_lut[i - 1U];
            if(segment_length <= 0.0001f)
            {
                return (float)i / (float)PATH_BSPLINE_LUT_SEGMENTS;
            }
            fraction = (distance_mm - path->distance_lut[i - 1U]) /
                       segment_length;
            return ((float)(i - 1U) + fraction) /
                   (float)PATH_BSPLINE_LUT_SEGMENTS;
        }
    }
    return 1.0f;
}

void PathPlanner_BuildCubicBSpline(CubicBSplinePath *path,
                                   PathPoint start,
                                   PathPoint goal,
                                   float start_yaw_deg,
                                   float max_speed_mm_s)
{
    uint8_t i;
    float dx;
    float dy;
    float distance;
    
    float heading_x;
    float heading_y;
    float control_spacing;
    float parameter;
    PathPoint last;
    PathPoint now;

    if(path == 0)
    {
        return;
    }

    dx = goal.x_mm - start.x_mm;
    dy = goal.y_mm - start.y_mm;
    distance = sqrtf((dx * dx) + (dy * dy));
    
    heading_x = cosf(start_yaw_deg * PATH_PI / 180.0f);
    heading_y = sinf(start_yaw_deg * PATH_PI / 180.0f);

    control_spacing = Path_Clamp(PATH_S_LENGTH, 50.0f, 400.0f);
    if(control_spacing > (distance * 0.22f))
    {
        control_spacing = distance * 0.22f;
    }

    path->control[0] = start;
    path->control[1].x_mm = start.x_mm + heading_x * control_spacing;
    path->control[1].y_mm = start.y_mm + heading_y * control_spacing;
    path->control[2].x_mm = start.x_mm + heading_x * control_spacing * 2.0f;
    path->control[2].y_mm = start.y_mm + heading_y * control_spacing * 2.0f;
    path->control[3].x_mm = goal.x_mm - heading_x * control_spacing * 2.0f;
    path->control[3].y_mm = goal.y_mm - heading_y * control_spacing * 2.0f;
    path->control[4].x_mm = goal.x_mm - heading_x * control_spacing;
    path->control[4].y_mm = goal.y_mm - heading_y * control_spacing;
    path->control[5] = goal;

    path->distance_lut[0] = 0.0f;
    last = Path_PointAt(path, 0.0f);
    for(i = 1U; i <= PATH_BSPLINE_LUT_SEGMENTS; i++)
    {
        parameter = (float)i / (float)PATH_BSPLINE_LUT_SEGMENTS;
        now = Path_PointAt(path, parameter);
        path->distance_lut[i] = path->distance_lut[i - 1U] +
                                Path_Distance(last, now);
        last = now;
    }

    path->length_mm = path->distance_lut[PATH_BSPLINE_LUT_SEGMENTS];
    path->min_speed_mm_s = PATH_MIN_SPEED_MM_S;
    path->max_speed_mm_s = (max_speed_mm_s > PATH_MIN_SPEED_MM_S) ?
                           max_speed_mm_s : PATH_MIN_SPEED_MM_S;
    path->accel_distance_mm = Path_Clamp(path->length_mm * PATH_ACCEL_RATIO,
                                         PATH_MIN_ACCEL_MM,
                                         PATH_MAX_ACCEL_MM);
    if(path->accel_distance_mm > (path->length_mm * 0.5f))
    {
        path->accel_distance_mm = path->length_mm * 0.5f;
    }
}

PathSample PathPlanner_SampleByDistance(const CubicBSplinePath *path,
                                        float distance_mm)
{
    PathSample sample;
    float parameter;
    float accel_progress;
    float decel_progress;
    float speed_progress;

    sample.point.x_mm = 0.0f;
    sample.point.y_mm = 0.0f;
    sample.tangent.x_mm = 1.0f;
    sample.tangent.y_mm = 0.0f;
    sample.speed_mm_s = 0.0f;
    sample.distance_mm = 0.0f;
    sample.finished = 1U;
    if((path == 0) || (path->length_mm <= 0.001f))
    {
        return sample;
    }

    sample.distance_mm = Path_Clamp(distance_mm, 0.0f, path->length_mm);
    parameter = Path_ParameterAtDistance(path, sample.distance_mm);
    sample.point = Path_PointAt(path, parameter);
    sample.tangent = Path_TangentAt(path, parameter);
    sample.finished = (sample.distance_mm >= path->length_mm) ? 1U : 0U;

    if(path->accel_distance_mm > 0.001f)
    {
        accel_progress = Path_Clamp(sample.distance_mm /
                                    path->accel_distance_mm, 0.0f, 1.0f);
        decel_progress = Path_Clamp((path->length_mm - sample.distance_mm) /
                                    path->accel_distance_mm, 0.0f, 1.0f);
    }
    else
    {
        accel_progress = 1.0f;
        decel_progress = 1.0f;
    }
    speed_progress = (accel_progress < decel_progress) ?
                     accel_progress : decel_progress;
    speed_progress = 3.0f * speed_progress * speed_progress -
                     2.0f * speed_progress * speed_progress * speed_progress;
    sample.speed_mm_s = path->min_speed_mm_s +
                        (path->max_speed_mm_s - path->min_speed_mm_s) *
                        speed_progress;
    if(sample.finished != 0U)
    {
        sample.speed_mm_s = 0.0f;
    }
    return sample;
}

float PathPlanner_FindNearestDistance(const CubicBSplinePath *path,
                                      PathPoint position,
                                      float minimum_distance_mm)
{
    uint8_t i;
    float dx;
    float dy;
    float distance_squared;
    float best_distance_squared;
    float best_path_distance;
    float minimum_allowed;
    PathPoint point;

    if((path == 0) || (path->length_mm <= 0.001f))
    {
        return 0.0f;
    }

    minimum_allowed = Path_Clamp(minimum_distance_mm - 40.0f,
                                 0.0f,
                                 path->length_mm);
    best_distance_squared = 1000000000.0f;
    best_path_distance = minimum_distance_mm;
    for(i = 0U; i <= PATH_BSPLINE_LUT_SEGMENTS; i++)
    {
        if(path->distance_lut[i] < minimum_allowed)
        {
            continue;
        }
        point = Path_PointAt(path,
                             (float)i / (float)PATH_BSPLINE_LUT_SEGMENTS);
        dx = point.x_mm - position.x_mm;
        dy = point.y_mm - position.y_mm;
        distance_squared = (dx * dx) + (dy * dy);
        if(distance_squared < best_distance_squared)
        {
            best_distance_squared = distance_squared;
            best_path_distance = path->distance_lut[i];
        }
    }
    if(best_path_distance < minimum_distance_mm)
    {
        best_path_distance = minimum_distance_mm;
    }
    return best_path_distance;
}
