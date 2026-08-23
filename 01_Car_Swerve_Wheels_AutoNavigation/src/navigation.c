#include "navigation.h"

#include "hwt9018.h"
#include "path_planner.h"

#include <math.h>

#define NAV_PI                         3.14159265358979323846f
#define NAV_UPDATE_PERIOD_MS           10U
#define NAV_IMU_TIMEOUT_MS             200U
#define NAV_MAX_RUNTIME_MS             15000U
#define NAV_LOOKAHEAD_MM               120.0f
#define NAV_FINAL_APPROACH_MM          280.0f
#define NAV_POSITION_TOLERANCE_MM      10.0f
#define NAV_FINAL_YAW_TOLERANCE_DEG    0.2f
#define NAV_FINAL_YAW_TARGET_DEG       1.0f

#define NAV_FINAL_YAW_STABLE_MS        300U
#define NAV_FINAL_YAW_MIN_RAD_S         0.02f

#define NAV_FINAL_POSITION_KP          0.0020f
#define NAV_FINAL_SPEED_LIMIT_M_S      0.30f
#define NAV_YAW_KP_RAD_S_PER_DEG       0.020f
#define NAV_YAW_LIMIT_RAD_S            0.80f
#define NAV_YAW_DEADBAND_DEG           0.10f
#define NAV_YAW_COMMAND_SIGN           -1.0f
#define NAV_FOLLOW_TANGENT_YAW         1U




static NavigationState g_state;
static CubicBSplinePath g_path;
static float g_path_distance_mm;
static float g_vx_m_s;
static float g_vy_m_s;
static float g_vw_rad_s;
static uint32_t g_last_update_ms;
static uint32_t g_last_imu_ms;
static uint32_t g_last_imu_frame_count;
static uint32_t g_navigation_start_ms;
static uint32_t g_final_yaw_stable_start_ms;

static float Nav_Clamp(float value, float minimum, float maximum)
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

static float Nav_WrapDeg(float angle)
{
    while(angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while(angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static float Nav_YawCommand(float target_yaw_deg, float current_yaw_deg)
{
    float error_deg;

    error_deg = Nav_WrapDeg(target_yaw_deg - current_yaw_deg);
    if(fabsf(error_deg) <= NAV_YAW_DEADBAND_DEG)
    {
        return 0.0f;
    }
    return Nav_Clamp(error_deg * NAV_YAW_KP_RAD_S_PER_DEG *
                     NAV_YAW_COMMAND_SIGN,
                     -NAV_YAW_LIMIT_RAD_S,
                     NAV_YAW_LIMIT_RAD_S);
}

static void Nav_WorldToBody(float world_x,
                            float world_y,
                            float yaw_deg,
                            float *body_x,
                            float *body_y)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;

    yaw_rad = -yaw_deg * NAV_PI / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    *body_x = (world_x * cos_yaw) + (world_y * sin_yaw);
    *body_y = (-world_x * sin_yaw) + (world_y * cos_yaw);
}

static void Nav_Start(uint32_t now_ms)
{
    PathPoint start;
    PathPoint goal;

    HWT9018_ZeroYaw();
    Odometry_Init();
    start.x_mm = 0.0f;
    start.y_mm = 0.0f;
    goal.x_mm = NAV_TARGET_X_MM;
    goal.y_mm = NAV_TARGET_Y_MM;
    PathPlanner_BuildCubicBSpline(&g_path,
                                  start,
                                  goal,
                                  0.0f,
                                  NAV_MAX_SPEED_MM_S);
    g_path_distance_mm = 0.0f;
    g_vx_m_s = 0.0f;
    g_vy_m_s = 0.0f;
    g_vw_rad_s = 0.0f;
    g_final_yaw_stable_start_ms = 0U;
    g_navigation_start_ms = now_ms;
    g_last_update_ms = now_ms;
    g_state = NAV_STATE_RUNNING;
}

void Navigation_Init(void)
{
    g_state = NAV_STATE_WAIT_DELAY;
    g_path_distance_mm = 0.0f;
    g_vx_m_s = 0.0f;
    g_vy_m_s = 0.0f;
    g_vw_rad_s = 0.0f;
    g_last_update_ms = 0U;
    g_last_imu_ms = 0U;
    g_last_imu_frame_count = 0U;
    g_navigation_start_ms = 0U;
    g_final_yaw_stable_start_ms = 0U;
}

void Navigation_Task(uint32_t now_ms)
{
    uint32_t frame_count;
    OdometryPose pose;
    PathPoint position;
    PathSample lookahead;
    float goal_dx;
    float goal_dy;
    float goal_distance;
    float target_dx;
    float target_dy;
    float target_distance;
    float speed_m_s;
    float world_vx;
    float world_vy;
    float target_yaw_deg;
    uint32_t update_elapsed_ms;
    float final_yaw_deg;

    frame_count = HWT9018_GetAngleFrameCount();
    if(frame_count != g_last_imu_frame_count)
    {
        g_last_imu_frame_count = frame_count;
        g_last_imu_ms = now_ms;
    }

    if(now_ms < NAV_AUTO_START_DELAY_MS)
    {
        return;
    }

    if(g_state == NAV_STATE_WAIT_DELAY)
    {
        g_state = NAV_STATE_WAIT_IMU;
    }
    if(g_state == NAV_STATE_WAIT_IMU)
    {
        if((HWT9018_YawValid() != 0U) &&
           ((uint32_t)(now_ms - g_last_imu_ms) <= NAV_IMU_TIMEOUT_MS))
        {
            Nav_Start(now_ms);
        }
        return;
    }
    if(g_state != NAV_STATE_RUNNING)
    {
        return;
    }

    if((uint32_t)(now_ms - g_last_imu_ms) > NAV_IMU_TIMEOUT_MS)
    {
        g_vx_m_s = 0.0f;
        g_vy_m_s = 0.0f;
        g_vw_rad_s = 0.0f;
        g_state = NAV_STATE_IMU_FAULT;
        return;
    }
    if((uint32_t)(now_ms - g_navigation_start_ms) >= NAV_MAX_RUNTIME_MS)
    {
        g_vx_m_s = 0.0f;
        g_vy_m_s = 0.0f;
        g_vw_rad_s = 0.0f;
        g_state = NAV_STATE_TIMEOUT;
        return;
    }

    update_elapsed_ms = (uint32_t)(now_ms - g_last_update_ms);
    if(update_elapsed_ms < NAV_UPDATE_PERIOD_MS)
    {
        return;
    }
    g_last_update_ms = now_ms;
    Odometry_Update((float)update_elapsed_ms * 0.001f);

    pose = Odometry_GetPose();
    final_yaw_deg = HWT9018_GetYawDeg();

    position.x_mm = pose.x_mm;
    position.y_mm = pose.y_mm;
    g_path_distance_mm = PathPlanner_FindNearestDistance(&g_path,
                                                         position,
                                                         g_path_distance_mm);
    goal_dx = NAV_TARGET_X_MM - pose.x_mm;
    goal_dy = NAV_TARGET_Y_MM - pose.y_mm;
    goal_distance = sqrtf((goal_dx * goal_dx) + (goal_dy * goal_dy));
    if(goal_distance <= NAV_POSITION_TOLERANCE_MM)
{
    g_vx_m_s = 0.0f;
    g_vy_m_s = 0.0f;
    g_vw_rad_s = Nav_YawCommand(NAV_FINAL_YAW_TARGET_DEG, final_yaw_deg);

    if(fabsf(Nav_WrapDeg(NAV_FINAL_YAW_TARGET_DEG - final_yaw_deg)) <=
       NAV_FINAL_YAW_TOLERANCE_DEG)
    {
        g_vw_rad_s = 0.0f;

        if(g_final_yaw_stable_start_ms == 0U)
        {
            g_final_yaw_stable_start_ms = now_ms;
        }
        else if((uint32_t)(now_ms - g_final_yaw_stable_start_ms) >=
                NAV_FINAL_YAW_STABLE_MS)
        {
            g_state = NAV_STATE_FINISHED;
        }
    }
    else
{
    if((g_vw_rad_s > 0.0f) &&
       (g_vw_rad_s < NAV_FINAL_YAW_MIN_RAD_S))
    {
        g_vw_rad_s = NAV_FINAL_YAW_MIN_RAD_S;
    }
    else if((g_vw_rad_s < 0.0f) &&
            (g_vw_rad_s > -NAV_FINAL_YAW_MIN_RAD_S))
    {
        g_vw_rad_s = -NAV_FINAL_YAW_MIN_RAD_S;
    }

    g_final_yaw_stable_start_ms = 0U;
}

    return;
}

g_final_yaw_stable_start_ms = 0U;
    if(goal_distance <= NAV_FINAL_APPROACH_MM)
    {
        world_vx = Nav_Clamp(goal_dx * NAV_FINAL_POSITION_KP,
                             -NAV_FINAL_SPEED_LIMIT_M_S,
                             NAV_FINAL_SPEED_LIMIT_M_S);
        world_vy = Nav_Clamp(goal_dy * NAV_FINAL_POSITION_KP,
                             -NAV_FINAL_SPEED_LIMIT_M_S,
                             NAV_FINAL_SPEED_LIMIT_M_S);
        Nav_WorldToBody(world_vx,
                        world_vy,
                        pose.yaw_deg,
                        &g_vx_m_s,
                        &g_vy_m_s);
        g_vw_rad_s = Nav_YawCommand(NAV_FINAL_YAW_TARGET_DEG, final_yaw_deg);
        return;
    }

    lookahead = PathPlanner_SampleByDistance(&g_path,
                                              g_path_distance_mm +
                                              NAV_LOOKAHEAD_MM);
    target_dx = lookahead.point.x_mm - pose.x_mm;
    target_dy = lookahead.point.y_mm - pose.y_mm;
    target_distance = sqrtf((target_dx * target_dx) +
                            (target_dy * target_dy));
    speed_m_s = lookahead.speed_mm_s / 1000.0f;
    if(target_distance > 0.001f)
    {
        world_vx = target_dx * speed_m_s / target_distance;
        world_vy = target_dy * speed_m_s / target_distance;
    }
    else
    {
        world_vx = 0.0f;
        world_vy = 0.0f;
    }
    Nav_WorldToBody(world_vx,
                    world_vy,
                    pose.yaw_deg,
                    &g_vx_m_s,
                    &g_vy_m_s);
#if (NAV_FOLLOW_TANGENT_YAW != 0U)
    target_yaw_deg = -atan2f(lookahead.tangent.y_mm,
                            lookahead.tangent.x_mm) * 180.0f / NAV_PI;
#else
    target_yaw_deg = 0.0f;
#endif
    g_vw_rad_s = Nav_YawCommand(target_yaw_deg, pose.yaw_deg);
}

uint8_t Navigation_GetMotion(float *vx_m_s,
                             float *vy_m_s,
                             float *vw_rad_s)
{
    float output_vx;
    float output_vy;
    float output_vw;
    float output_speed;
    float maximum_speed;
    float speed_scale;

    if((vx_m_s == 0) || (vy_m_s == 0) || (vw_rad_s == 0))
    {
        return 0U;
    }
    if(g_state == NAV_STATE_WAIT_DELAY)
    {
        return 0U;
    }
    output_vx = g_vx_m_s;
    output_vy = g_vy_m_s;
    output_vw = g_vw_rad_s;
    if((output_vx != output_vx) ||
       (output_vy != output_vy) ||
       (output_vw != output_vw) ||
       (fabsf(output_vx) > 100.0f) ||
       (fabsf(output_vy) > 100.0f) ||
       (fabsf(output_vw) > 100.0f))
    {
        g_vx_m_s = 0.0f;
        g_vy_m_s = 0.0f;
        g_vw_rad_s = 0.0f;
        g_state = NAV_STATE_TIMEOUT;
        output_vx = 0.0f;
        output_vy = 0.0f;
        output_vw = 0.0f;
    }

    maximum_speed = NAV_MAX_SPEED_MM_S / 1000.0f;
    output_speed = sqrtf((output_vx * output_vx) +
                         (output_vy * output_vy));
    if((output_speed > maximum_speed) && (output_speed > 0.0001f))
    {
        speed_scale = maximum_speed / output_speed;
        output_vx *= speed_scale;
        output_vy *= speed_scale;
    }
    output_vw = Nav_Clamp(output_vw,
                          -NAV_YAW_LIMIT_RAD_S,
                          NAV_YAW_LIMIT_RAD_S);

    *vx_m_s = output_vx;
    *vy_m_s = output_vy;
    *vw_rad_s = output_vw;
    return 1U;
}

NavigationState Navigation_GetState(void)
{
    return g_state;
}

OdometryPose Navigation_GetPose(void)
{
    return Odometry_GetPose();
}
