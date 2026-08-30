#include "navigation.h"

#include "hwt9018.h"
#include "path_planner.h"

#include <math.h>

#define NAV_PI                         3.14159265358979323846f
#define NAV_UPDATE_PERIOD_MS           10U
#define NAV_IMU_TIMEOUT_MS             200U
#define NAV_MAX_RUNTIME_MS             50000U
#define NAV_STOP_REASON_NONE           0U
#define NAV_STOP_REASON_IMU_TIMEOUT    1U
#define NAV_STOP_REASON_RUNTIME        2U
#define NAV_STOP_REASON_INVALID_OUTPUT 3U
#define NAV_STOP_REASON_FINISHED       4U
#define NAV_LOOKAHEAD_MM               120.0f
#define NAV_FINAL_APPROACH_MM          280.0f
#define NAV_POSITION_TOLERANCE_MM      10.0f
#define NAV_FINAL_YAW_TOLERANCE_DEG    0.2f
#define NAV_FINAL_YAW_TARGET_DEG       -3.0f
 
#define NAV_FINAL_YAW_STABLE_MS        300U
#define NAV_FINAL_YAW_MIN_RAD_S         0.02f

#define NAV_FINAL_POSITION_KP          0.0020f
#define NAV_FINAL_SPEED_LIMIT_M_S      0.30f
#define NAV_YAW_KP_RAD_S_PER_DEG       0.020f
#define NAV_YAW_LIMIT_RAD_S            0.50f
#define NAV_YAW_DEADBAND_DEG           0.10f
#define NAV_YAW_COMMAND_SIGN           -1.0f
#define NAV_FOLLOW_TANGENT_YAW         0U
#define NAV_CURVATURE_SPEED_LIMIT      1U
#define NAV_MAX_LAT_ACCEL_MM_S2        800.0f

/* Set to 0U and rebuild to restore the previous lookahead tracker. */
#define NAV_PATH_TRACKER_MPC_ENABLE    1U

#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
#define NAV_MPC_HORIZON                5U
#define NAV_MPC_ITERATIONS             5U
#define NAV_MPC_SOLVE_PERIOD_MS        20U
#define NAV_MPC_MODEL_DT_S             0.020f
#define NAV_MPC_REFERENCE_LEAD_MM      40.0f
#define NAV_MPC_Q_ALONG                20.0f
#define NAV_MPC_Q_CROSS                50.0f
#define NAV_MPC_TERMINAL_SCALE         3.0f
#define NAV_MPC_R_VELOCITY             2.0f
#define NAV_MPC_R_DELTA_VELOCITY       0.5f
#define NAV_MPC_MAX_ACCEL_M_S2         1.2f
#define NAV_MPC_PREVIEW_BRAKE_SCALE    0.75f
#define NAV_MPC_GRADIENT_STEP          0.05f
#endif
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
volatile uint8_t g_navigation_stop_reason_latched;

#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
static float g_mpc_u_x[NAV_MPC_HORIZON];
static float g_mpc_u_y[NAV_MPC_HORIZON];
static float g_mpc_state_x[NAV_MPC_HORIZON + 1U];
static float g_mpc_state_y[NAV_MPC_HORIZON + 1U];
static float g_mpc_ref_x[NAV_MPC_HORIZON];
static float g_mpc_ref_y[NAV_MPC_HORIZON];
static float g_mpc_ref_vx[NAV_MPC_HORIZON];
static float g_mpc_ref_vy[NAV_MPC_HORIZON];
static float g_mpc_ref_tx[NAV_MPC_HORIZON];
static float g_mpc_ref_ty[NAV_MPC_HORIZON];
static float g_mpc_gradient_x[NAV_MPC_HORIZON];
static float g_mpc_gradient_y[NAV_MPC_HORIZON];
static float g_mpc_speed_profile_m_s[PATH_BSPLINE_LUT_SEGMENTS + 1U];
static float g_mpc_last_world_vx;
static float g_mpc_last_world_vy;
static float g_mpc_output_world_vx;
static float g_mpc_output_world_vy;
static float g_mpc_target_world_vx;
static float g_mpc_target_world_vy;
static uint32_t g_mpc_last_solve_ms;
static uint8_t g_mpc_initialized;
static uint8_t g_mpc_solution_valid;

volatile uint8_t g_navigation_mpc_active;
volatile uint8_t g_navigation_mpc_fallback;
volatile float g_navigation_mpc_world_vx_m_s;
volatile float g_navigation_mpc_world_vy_m_s;
volatile float g_navigation_mpc_cross_error_mm;
volatile float g_navigation_mpc_cost;
volatile uint32_t g_navigation_mpc_solve_cycles_max;
volatile float g_navigation_mpc_reference_speed_m_s;
volatile uint8_t g_navigation_mpc_speed_profile_valid;
#endif

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

#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
static uint8_t Nav_MpcValueValid(float value)
{
    if((value != value) || (fabsf(value) > 1.0e20f))
    {
        return 0U;
    }
    return 1U;
}

static void Nav_MpcLimitVector(float *x, float *y, float limit)
{
    float magnitude;
    float scale;

    if((x == 0) || (y == 0))
    {
        return;
    }
    if(limit <= 0.0f)
    {
        *x = 0.0f;
        *y = 0.0f;
        return;
    }
    magnitude = sqrtf((*x * *x) + (*y * *y));
    if((magnitude > limit) && (magnitude > 0.000001f))
    {
        scale = limit / magnitude;
        *x *= scale;
        *y *= scale;
    }
}

static float Nav_MpcInstantaneousSpeed(const PathSample *sample)
{
    float speed_m_s;
    float curvature;
    float curvature_speed_m_s;
    float maximum_speed_m_s;

    if(sample == 0)
    {
        return 0.0f;
    }
    maximum_speed_m_s = NAV_MAX_SPEED_MM_S * 0.001f;
    speed_m_s = Nav_Clamp(sample->speed_mm_s * 0.001f,
                          0.0f,
                          maximum_speed_m_s);
#if (NAV_CURVATURE_SPEED_LIMIT != 0U)
    curvature = fabsf(sample->curvature);
    if(curvature > 0.0001f)
    {
        curvature_speed_m_s = sqrtf(NAV_MAX_LAT_ACCEL_MM_S2 /
                                    curvature) * 0.001f;
        if(speed_m_s > curvature_speed_m_s)
        {
            speed_m_s = curvature_speed_m_s;
        }
    }
#endif
    return speed_m_s;
}

static void Nav_MpcBuildSpeedProfile(void)
{
    uint16_t i;
    int reverse_i;
    PathSample sample;
    float distance_step_mm;
    float distance_step_m;
    float distance_mm;
    float brake_accel_m_s2;
    float next_speed_m_s;
    float reachable_speed_m_s;

    g_navigation_mpc_speed_profile_valid = 0U;
    if((Nav_MpcValueValid(g_path.length_mm) == 0U) ||
       (g_path.length_mm <= 0.001f))
    {
        return;
    }

    distance_step_mm = g_path.length_mm /
                       (float)PATH_BSPLINE_LUT_SEGMENTS;
    distance_step_m = distance_step_mm * 0.001f;
    brake_accel_m_s2 = NAV_MPC_MAX_ACCEL_M_S2 *
                       NAV_MPC_PREVIEW_BRAKE_SCALE;
    if((distance_step_m <= 0.0f) ||
       (brake_accel_m_s2 <= 0.0f))
    {
        return;
    }

    for(i = 0U; i <= PATH_BSPLINE_LUT_SEGMENTS; i++)
    {
        distance_mm = distance_step_mm * (float)i;
        sample = PathPlanner_SampleByDistance(&g_path, distance_mm);
        g_mpc_speed_profile_m_s[i] =
            Nav_MpcInstantaneousSpeed(&sample);
        if(Nav_MpcValueValid(g_mpc_speed_profile_m_s[i]) == 0U)
        {
            return;
        }
    }

    for(reverse_i = (int)PATH_BSPLINE_LUT_SEGMENTS - 1;
        reverse_i >= 0;
        reverse_i--)
    {
        next_speed_m_s = g_mpc_speed_profile_m_s[reverse_i + 1];
        reachable_speed_m_s = sqrtf((next_speed_m_s * next_speed_m_s) +
                                    (2.0f * brake_accel_m_s2 *
                                     distance_step_m));
        if(g_mpc_speed_profile_m_s[reverse_i] > reachable_speed_m_s)
        {
            g_mpc_speed_profile_m_s[reverse_i] = reachable_speed_m_s;
        }
    }
    g_navigation_mpc_speed_profile_valid = 1U;
}

static float Nav_MpcSpeedProfileAt(float distance_mm)
{
    uint16_t index;
    float scaled_index;
    float fraction;

    if((g_navigation_mpc_speed_profile_valid == 0U) ||
       (g_path.length_mm <= 0.001f))
    {
        return NAV_MAX_SPEED_MM_S * 0.001f;
    }
    scaled_index = Nav_Clamp(distance_mm / g_path.length_mm,
                             0.0f,
                             1.0f) *
                   (float)PATH_BSPLINE_LUT_SEGMENTS;
    index = (uint16_t)scaled_index;
    if(index >= PATH_BSPLINE_LUT_SEGMENTS)
    {
        return g_mpc_speed_profile_m_s[PATH_BSPLINE_LUT_SEGMENTS];
    }
    fraction = scaled_index - (float)index;
    return g_mpc_speed_profile_m_s[index] +
           ((g_mpc_speed_profile_m_s[index + 1U] -
             g_mpc_speed_profile_m_s[index]) * fraction);
}

static float Nav_MpcReferenceSpeed(const PathSample *sample)
{
    float speed_m_s;
    float preview_speed_m_s;

    if(sample == 0)
    {
        return 0.0f;
    }
    speed_m_s = Nav_MpcInstantaneousSpeed(sample);
    preview_speed_m_s = Nav_MpcSpeedProfileAt(sample->distance_mm);
    if(speed_m_s > preview_speed_m_s)
    {
        speed_m_s = preview_speed_m_s;
    }
    return speed_m_s;
}

static void Nav_MpcReset(void)
{
    uint8_t i;

    for(i = 0U; i < NAV_MPC_HORIZON; i++)
    {
        g_mpc_u_x[i] = 0.0f;
        g_mpc_u_y[i] = 0.0f;
        g_mpc_state_x[i] = 0.0f;
        g_mpc_state_y[i] = 0.0f;
        g_mpc_ref_x[i] = 0.0f;
        g_mpc_ref_y[i] = 0.0f;
        g_mpc_ref_vx[i] = 0.0f;
        g_mpc_ref_vy[i] = 0.0f;
        g_mpc_ref_tx[i] = 0.0f;
        g_mpc_ref_ty[i] = 0.0f;
        g_mpc_gradient_x[i] = 0.0f;
        g_mpc_gradient_y[i] = 0.0f;
    }
    g_mpc_state_x[NAV_MPC_HORIZON] = 0.0f;
    g_mpc_state_y[NAV_MPC_HORIZON] = 0.0f;
    g_mpc_last_world_vx = 0.0f;
    g_mpc_last_world_vy = 0.0f;
    g_mpc_output_world_vx = 0.0f;
    g_mpc_output_world_vy = 0.0f;
    g_mpc_target_world_vx = 0.0f;
    g_mpc_target_world_vy = 0.0f;
    g_mpc_last_solve_ms = 0U;
    g_mpc_initialized = 0U;
    g_mpc_solution_valid = 0U;
    g_navigation_mpc_active = 0U;
    g_navigation_mpc_fallback = 0U;
    g_navigation_mpc_world_vx_m_s = 0.0f;
    g_navigation_mpc_world_vy_m_s = 0.0f;
    g_navigation_mpc_cross_error_mm = 0.0f;
    g_navigation_mpc_cost = 0.0f;
    g_navigation_mpc_reference_speed_m_s = 0.0f;
}

static uint8_t Nav_MpcBuildReference(float pose_x_mm,
                                     float pose_y_mm,
                                     float path_distance_mm)
{
    uint8_t i;
    PathSample sample;
    float reference_distance_mm;
    float reference_speed_m_s;
    float tangent_length;
    float tangent_x;
    float tangent_y;
    float error_x_mm;
    float error_y_mm;
    float previous_reference_x_mm;
    float previous_reference_y_mm;

    if((Nav_MpcValueValid(pose_x_mm) == 0U) ||
       (Nav_MpcValueValid(pose_y_mm) == 0U) ||
       (Nav_MpcValueValid(path_distance_mm) == 0U))
    {
        return 0U;
    }

    reference_distance_mm = Nav_Clamp(path_distance_mm +
                                      NAV_MPC_REFERENCE_LEAD_MM,
                                      0.0f,
                                      g_path.length_mm);
    sample = PathPlanner_SampleByDistance(&g_path, reference_distance_mm);
    tangent_length = sqrtf((sample.tangent.x_mm * sample.tangent.x_mm) +
                           (sample.tangent.y_mm * sample.tangent.y_mm));
    if((Nav_MpcValueValid(tangent_length) == 0U) ||
       (tangent_length <= 0.0001f))
    {
        return 0U;
    }
    tangent_x = sample.tangent.x_mm / tangent_length;
    tangent_y = sample.tangent.y_mm / tangent_length;
    error_x_mm = pose_x_mm - sample.point.x_mm;
    error_y_mm = pose_y_mm - sample.point.y_mm;
    g_navigation_mpc_cross_error_mm =
        (error_x_mm * -tangent_y) + (error_y_mm * tangent_x);
    reference_speed_m_s = Nav_MpcReferenceSpeed(&sample);
    g_navigation_mpc_reference_speed_m_s = reference_speed_m_s;
    previous_reference_x_mm = sample.point.x_mm;
    previous_reference_y_mm = sample.point.y_mm;

    for(i = 0U; i < NAV_MPC_HORIZON; i++)
    {
        reference_distance_mm += reference_speed_m_s *
                                 NAV_MPC_MODEL_DT_S * 1000.0f;
        sample = PathPlanner_SampleByDistance(&g_path,
                                               reference_distance_mm);
        tangent_length = sqrtf((sample.tangent.x_mm * sample.tangent.x_mm) +
                               (sample.tangent.y_mm * sample.tangent.y_mm));
        if((Nav_MpcValueValid(tangent_length) == 0U) ||
           (tangent_length <= 0.0001f))
        {
            return 0U;
        }
        tangent_x = sample.tangent.x_mm / tangent_length;
        tangent_y = sample.tangent.y_mm / tangent_length;
        g_mpc_ref_x[i] = (sample.point.x_mm - pose_x_mm) * 0.001f;
        g_mpc_ref_y[i] = (sample.point.y_mm - pose_y_mm) * 0.001f;
        g_mpc_ref_tx[i] = tangent_x;
        g_mpc_ref_ty[i] = tangent_y;
        g_mpc_ref_vx[i] = (sample.point.x_mm - previous_reference_x_mm) *
                          0.001f / NAV_MPC_MODEL_DT_S;
        g_mpc_ref_vy[i] = (sample.point.y_mm - previous_reference_y_mm) *
                          0.001f / NAV_MPC_MODEL_DT_S;
        previous_reference_x_mm = sample.point.x_mm;
        previous_reference_y_mm = sample.point.y_mm;
        reference_speed_m_s = Nav_MpcReferenceSpeed(&sample);
        if((Nav_MpcValueValid(g_mpc_ref_x[i]) == 0U) ||
           (Nav_MpcValueValid(g_mpc_ref_y[i]) == 0U) ||
           (Nav_MpcValueValid(g_mpc_ref_vx[i]) == 0U) ||
           (Nav_MpcValueValid(g_mpc_ref_vy[i]) == 0U))
        {
            return 0U;
        }
    }
    return 1U;
}

static void Nav_MpcForwardPredict(void)
{
    uint8_t i;

    g_mpc_state_x[0] = 0.0f;
    g_mpc_state_y[0] = 0.0f;
    for(i = 0U; i < NAV_MPC_HORIZON; i++)
    {
        g_mpc_state_x[i + 1U] = g_mpc_state_x[i] +
                                     NAV_MPC_MODEL_DT_S * g_mpc_u_x[i];
        g_mpc_state_y[i + 1U] = g_mpc_state_y[i] +
                                     NAV_MPC_MODEL_DT_S * g_mpc_u_y[i];
    }
}

static float Nav_MpcCalculateCost(void)
{
    uint8_t i;
    float error_x;
    float error_y;
    float error_along;
    float error_cross;
    float velocity_error_x;
    float velocity_error_y;
    float delta_x;
    float delta_y;
    float previous_x;
    float previous_y;
    float position_scale;
    float cost;

    Nav_MpcForwardPredict();
    cost = 0.0f;
    previous_x = g_mpc_last_world_vx;
    previous_y = g_mpc_last_world_vy;
    for(i = 0U; i < NAV_MPC_HORIZON; i++)
    {
        error_x = g_mpc_state_x[i + 1U] - g_mpc_ref_x[i];
        error_y = g_mpc_state_y[i + 1U] - g_mpc_ref_y[i];
        error_along = (error_x * g_mpc_ref_tx[i]) +
                      (error_y * g_mpc_ref_ty[i]);
        error_cross = (error_x * -g_mpc_ref_ty[i]) +
                      (error_y * g_mpc_ref_tx[i]);
        position_scale = (i == (NAV_MPC_HORIZON - 1U)) ?
                         NAV_MPC_TERMINAL_SCALE : 1.0f;
        velocity_error_x = g_mpc_u_x[i] - g_mpc_ref_vx[i];
        velocity_error_y = g_mpc_u_y[i] - g_mpc_ref_vy[i];
        delta_x = g_mpc_u_x[i] - previous_x;
        delta_y = g_mpc_u_y[i] - previous_y;
        cost += position_scale *
                ((NAV_MPC_Q_ALONG * error_along * error_along) +
                 (NAV_MPC_Q_CROSS * error_cross * error_cross));
        cost += NAV_MPC_R_VELOCITY *
                ((velocity_error_x * velocity_error_x) +
                 (velocity_error_y * velocity_error_y));
        cost += NAV_MPC_R_DELTA_VELOCITY *
                ((delta_x * delta_x) + (delta_y * delta_y));
        previous_x = g_mpc_u_x[i];
        previous_y = g_mpc_u_y[i];
    }
    return cost;
}

static uint8_t Nav_MpcSolve(float first_step_dt_s)
{
    uint8_t i;
    uint8_t iteration;
    int reverse_i;
    float error_x;
    float error_y;
    float error_along;
    float error_cross;
    float position_gradient_x;
    float position_gradient_y;
    float accumulated_gradient_x;
    float accumulated_gradient_y;
    float previous_x;
    float previous_y;
    float delta_x;
    float delta_y;
    float delta_magnitude;
    float maximum_delta;
    float candidate_x;
    float candidate_y;
    float position_scale;
    float output_speed;
    float cost;

    if(g_mpc_initialized != 0U)
    {
        for(i = 0U; i < (NAV_MPC_HORIZON - 1U); i++)
        {
            g_mpc_u_x[i] = g_mpc_u_x[i + 1U];
            g_mpc_u_y[i] = g_mpc_u_y[i + 1U];
        }
        g_mpc_u_x[NAV_MPC_HORIZON - 1U] =
            g_mpc_ref_vx[NAV_MPC_HORIZON - 1U];
        g_mpc_u_y[NAV_MPC_HORIZON - 1U] =
            g_mpc_ref_vy[NAV_MPC_HORIZON - 1U];
    }
    else
    {
        for(i = 0U; i < NAV_MPC_HORIZON; i++)
        {
            g_mpc_u_x[i] = g_mpc_ref_vx[i];
            g_mpc_u_y[i] = g_mpc_ref_vy[i];
        }
    }

    for(iteration = 0U; iteration < NAV_MPC_ITERATIONS; iteration++)
    {
        Nav_MpcForwardPredict();
        accumulated_gradient_x = 0.0f;
        accumulated_gradient_y = 0.0f;
        for(reverse_i = (int)NAV_MPC_HORIZON - 1;
            reverse_i >= 0;
            reverse_i--)
        {
            i = (uint8_t)reverse_i;
            error_x = g_mpc_state_x[i + 1U] - g_mpc_ref_x[i];
            error_y = g_mpc_state_y[i + 1U] - g_mpc_ref_y[i];
            error_along = (error_x * g_mpc_ref_tx[i]) +
                          (error_y * g_mpc_ref_ty[i]);
            error_cross = (error_x * -g_mpc_ref_ty[i]) +
                          (error_y * g_mpc_ref_tx[i]);
            position_scale = (i == (NAV_MPC_HORIZON - 1U)) ?
                             NAV_MPC_TERMINAL_SCALE : 1.0f;
            position_gradient_x = 2.0f * position_scale *
                ((NAV_MPC_Q_ALONG * error_along * g_mpc_ref_tx[i]) -
                 (NAV_MPC_Q_CROSS * error_cross * g_mpc_ref_ty[i]));
            position_gradient_y = 2.0f * position_scale *
                ((NAV_MPC_Q_ALONG * error_along * g_mpc_ref_ty[i]) +
                 (NAV_MPC_Q_CROSS * error_cross * g_mpc_ref_tx[i]));
            accumulated_gradient_x += position_gradient_x;
            accumulated_gradient_y += position_gradient_y;
            previous_x = (i == 0U) ? g_mpc_last_world_vx :
                         g_mpc_u_x[i - 1U];
            previous_y = (i == 0U) ? g_mpc_last_world_vy :
                         g_mpc_u_y[i - 1U];
            g_mpc_gradient_x[i] = NAV_MPC_MODEL_DT_S *
                                  accumulated_gradient_x;
            g_mpc_gradient_y[i] = NAV_MPC_MODEL_DT_S *
                                  accumulated_gradient_y;
            g_mpc_gradient_x[i] += 2.0f * NAV_MPC_R_VELOCITY *
                                   (g_mpc_u_x[i] - g_mpc_ref_vx[i]);
            g_mpc_gradient_y[i] += 2.0f * NAV_MPC_R_VELOCITY *
                                   (g_mpc_u_y[i] - g_mpc_ref_vy[i]);
            g_mpc_gradient_x[i] += 2.0f * NAV_MPC_R_DELTA_VELOCITY *
                                   (g_mpc_u_x[i] - previous_x);
            g_mpc_gradient_y[i] += 2.0f * NAV_MPC_R_DELTA_VELOCITY *
                                   (g_mpc_u_y[i] - previous_y);
            if(i < (NAV_MPC_HORIZON - 1U))
            {
                g_mpc_gradient_x[i] -= 2.0f *
                    NAV_MPC_R_DELTA_VELOCITY *
                    (g_mpc_u_x[i + 1U] - g_mpc_u_x[i]);
                g_mpc_gradient_y[i] -= 2.0f *
                    NAV_MPC_R_DELTA_VELOCITY *
                    (g_mpc_u_y[i + 1U] - g_mpc_u_y[i]);
            }
        }

        previous_x = g_mpc_last_world_vx;
        previous_y = g_mpc_last_world_vy;
        for(i = 0U; i < NAV_MPC_HORIZON; i++)
        {
            candidate_x = g_mpc_u_x[i] -
                          NAV_MPC_GRADIENT_STEP * g_mpc_gradient_x[i];
            candidate_y = g_mpc_u_y[i] -
                          NAV_MPC_GRADIENT_STEP * g_mpc_gradient_y[i];
            delta_x = candidate_x - previous_x;
            delta_y = candidate_y - previous_y;
            delta_magnitude = sqrtf((delta_x * delta_x) +
                                    (delta_y * delta_y));
            maximum_delta = NAV_MPC_MAX_ACCEL_M_S2 *
                            ((i == 0U) ? first_step_dt_s :
                             NAV_MPC_MODEL_DT_S);
            if((delta_magnitude > maximum_delta) &&
               (delta_magnitude > 0.000001f))
            {
                delta_x *= maximum_delta / delta_magnitude;
                delta_y *= maximum_delta / delta_magnitude;
                candidate_x = previous_x + delta_x;
                candidate_y = previous_y + delta_y;
            }
            Nav_MpcLimitVector(&candidate_x,
                               &candidate_y,
                               NAV_MAX_SPEED_MM_S * 0.001f);
            g_mpc_u_x[i] = candidate_x;
            g_mpc_u_y[i] = candidate_y;
            previous_x = candidate_x;
            previous_y = candidate_y;
        }
    }

    cost = Nav_MpcCalculateCost();
    output_speed = sqrtf((g_mpc_u_x[0] * g_mpc_u_x[0]) +
                         (g_mpc_u_y[0] * g_mpc_u_y[0]));
    if((Nav_MpcValueValid(cost) == 0U) ||
       (Nav_MpcValueValid(g_mpc_u_x[0]) == 0U) ||
       (Nav_MpcValueValid(g_mpc_u_y[0]) == 0U) ||
       (Nav_MpcValueValid(output_speed) == 0U) ||
       (output_speed > (NAV_MAX_SPEED_MM_S * 0.001f + 0.001f)))
    {
        return 0U;
    }
    g_navigation_mpc_cost = cost;
    g_mpc_target_world_vx = g_mpc_u_x[0];
    g_mpc_target_world_vy = g_mpc_u_y[0];
    g_mpc_initialized = 1U;
    g_mpc_solution_valid = 1U;
    return 1U;
}

static uint8_t Nav_MpcTrack(float pose_x_mm,
                            float pose_y_mm,
                            float path_distance_mm,
                            uint32_t now_ms,
                            float update_dt_s,
                            float *world_vx,
                            float *world_vy)
{
    uint32_t solve_elapsed_ms;
    uint32_t solve_start_cycles;
    uint32_t solve_cycles;
    float first_step_dt_s;
    float delta_x;
    float delta_y;
    float delta_magnitude;
    float maximum_delta;

    if((world_vx == 0) || (world_vy == 0))
    {
        g_mpc_initialized = 0U;
        g_mpc_solution_valid = 0U;
        g_navigation_mpc_active = 0U;
        g_navigation_mpc_fallback = 1U;
        return 0U;
    }
    solve_elapsed_ms = (uint32_t)(now_ms - g_mpc_last_solve_ms);
    if((g_mpc_last_solve_ms == 0U) ||
       (solve_elapsed_ms >= NAV_MPC_SOLVE_PERIOD_MS))
    {
        if(g_mpc_last_solve_ms == 0U)
        {
            first_step_dt_s = NAV_MPC_SOLVE_PERIOD_MS * 0.001f;
        }
        else
        {
            first_step_dt_s = Nav_Clamp(solve_elapsed_ms * 0.001f,
                                        NAV_MPC_MODEL_DT_S,
                                        0.050f);
        }
        g_mpc_last_solve_ms = now_ms;
        solve_start_cycles = DWT->CYCCNT;
        if((Nav_MpcBuildReference(pose_x_mm,
                                  pose_y_mm,
                                  path_distance_mm) == 0U) ||
           (Nav_MpcSolve(first_step_dt_s) == 0U))
        {
            g_mpc_initialized = 0U;
            g_mpc_solution_valid = 0U;
        }
        solve_cycles = (uint32_t)(DWT->CYCCNT - solve_start_cycles);
        if(solve_cycles > g_navigation_mpc_solve_cycles_max)
        {
            g_navigation_mpc_solve_cycles_max = solve_cycles;
        }
    }

    if(g_mpc_solution_valid == 0U)
    {
        g_navigation_mpc_active = 0U;
        g_navigation_mpc_fallback = 1U;
        return 0U;
    }
    update_dt_s = Nav_Clamp(update_dt_s, 0.001f, 0.020f);
    delta_x = g_mpc_target_world_vx - g_mpc_output_world_vx;
    delta_y = g_mpc_target_world_vy - g_mpc_output_world_vy;
    delta_magnitude = sqrtf((delta_x * delta_x) +
                            (delta_y * delta_y));
    maximum_delta = NAV_MPC_MAX_ACCEL_M_S2 * update_dt_s;
    if((delta_magnitude > maximum_delta) &&
       (delta_magnitude > 0.000001f))
    {
        delta_x *= maximum_delta / delta_magnitude;
        delta_y *= maximum_delta / delta_magnitude;
    }
    g_mpc_output_world_vx += delta_x;
    g_mpc_output_world_vy += delta_y;
    Nav_MpcLimitVector(&g_mpc_output_world_vx,
                       &g_mpc_output_world_vy,
                       NAV_MAX_SPEED_MM_S * 0.001f);
    g_mpc_last_world_vx = g_mpc_output_world_vx;
    g_mpc_last_world_vy = g_mpc_output_world_vy;
    *world_vx = g_mpc_output_world_vx;
    *world_vy = g_mpc_output_world_vy;
    g_navigation_mpc_world_vx_m_s = *world_vx;
    g_navigation_mpc_world_vy_m_s = *world_vy;
    g_navigation_mpc_active = 1U;
    g_navigation_mpc_fallback = 0U;
    return 1U;
}

static void Nav_MpcRecordFallback(float *world_vx,
                                  float *world_vy,
                                  float update_dt_s)
{
    float delta_x;
    float delta_y;
    float delta_magnitude;
    float maximum_delta;

    if((world_vx == 0) || (world_vy == 0))
    {
        return;
    }
    if((Nav_MpcValueValid(*world_vx) != 0U) &&
       (Nav_MpcValueValid(*world_vy) != 0U))
    {
        update_dt_s = Nav_Clamp(update_dt_s, 0.001f, 0.050f);
        delta_x = *world_vx - g_mpc_last_world_vx;
        delta_y = *world_vy - g_mpc_last_world_vy;
        delta_magnitude = sqrtf((delta_x * delta_x) +
                                (delta_y * delta_y));
        maximum_delta = NAV_MPC_MAX_ACCEL_M_S2 * update_dt_s;
        if((delta_magnitude > maximum_delta) &&
           (delta_magnitude > 0.000001f))
        {
            delta_x *= maximum_delta / delta_magnitude;
            delta_y *= maximum_delta / delta_magnitude;
            *world_vx = g_mpc_last_world_vx + delta_x;
            *world_vy = g_mpc_last_world_vy + delta_y;
        }
        Nav_MpcLimitVector(world_vx,
                           world_vy,
                           NAV_MAX_SPEED_MM_S * 0.001f);
        g_mpc_last_world_vx = *world_vx;
        g_mpc_last_world_vy = *world_vy;
        g_mpc_output_world_vx = *world_vx;
        g_mpc_output_world_vy = *world_vy;
        g_mpc_target_world_vx = *world_vx;
        g_mpc_target_world_vy = *world_vy;
    }
    else
    {
        *world_vx = 0.0f;
        *world_vy = 0.0f;
        g_mpc_last_world_vx = 0.0f;
        g_mpc_last_world_vy = 0.0f;
        g_mpc_output_world_vx = 0.0f;
        g_mpc_output_world_vy = 0.0f;
        g_mpc_target_world_vx = 0.0f;
        g_mpc_target_world_vy = 0.0f;
    }
    g_navigation_mpc_world_vx_m_s = g_mpc_last_world_vx;
    g_navigation_mpc_world_vy_m_s = g_mpc_last_world_vy;
    g_navigation_mpc_active = 0U;
    g_navigation_mpc_fallback = 1U;
    g_navigation_mpc_cost = 0.0f;
}
#endif

static void Nav_Start(uint32_t now_ms)
{
    PathPoint start;

    HWT9018_ZeroYaw();
    Odometry_Init();
    start.x_mm = 0.0f;
    start.y_mm = 0.0f;
    PathPlanner_BuildHeartBSpline(&g_path,
                                  start,
                                  0.0f,
                                  NAV_HEART_FORWARD_SCALE_MM,
                                  NAV_HEART_HALF_WIDTH_MM,
                                  NAV_MAX_SPEED_MM_S);
    g_path_distance_mm = 0.0f;
    g_vx_m_s = 0.0f;
    g_vy_m_s = 0.0f;
    g_vw_rad_s = 0.0f;
    g_final_yaw_stable_start_ms = 0U;
#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
    Nav_MpcReset();
    Nav_MpcBuildSpeedProfile();
    g_navigation_mpc_solve_cycles_max = 0U;
#endif
    g_navigation_start_ms = now_ms;
    g_last_update_ms = now_ms;
    g_navigation_stop_reason_latched = NAV_STOP_REASON_NONE;
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
    g_navigation_stop_reason_latched = NAV_STOP_REASON_NONE;
#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
    Nav_MpcReset();
    g_navigation_mpc_speed_profile_valid = 0U;
    g_navigation_mpc_solve_cycles_max = 0U;
#endif
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
    float path_remaining_mm;
    float target_dx;
    float target_dy;
    float target_distance;
    float speed_m_s;
    float world_vx;
    float world_vy;
    float target_yaw_deg;
    uint32_t update_elapsed_ms;
    float final_yaw_deg;
    float curvature_speed_m_s;

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
#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
        Nav_MpcReset();
#endif
        g_navigation_stop_reason_latched = NAV_STOP_REASON_IMU_TIMEOUT;
        g_state = NAV_STATE_IMU_FAULT;
        return;
    }
    if((uint32_t)(now_ms - g_navigation_start_ms) >= NAV_MAX_RUNTIME_MS)
    {
        g_vx_m_s = 0.0f;
        g_vy_m_s = 0.0f;
        g_vw_rad_s = 0.0f;
#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
        Nav_MpcReset();
#endif
        g_navigation_stop_reason_latched = NAV_STOP_REASON_RUNTIME;
        g_state = NAV_STATE_TIMEOUT;
        return;
    }

    update_elapsed_ms = (uint32_t)(now_ms - g_last_update_ms);
    if(update_elapsed_ms < NAV_UPDATE_PERIOD_MS)
    {
        return;
    }
    g_last_update_ms = now_ms;
    Odometry_Update((float)update_elapsed_ms * 0.001f, now_ms);

    pose = Odometry_GetPose();
    final_yaw_deg = pose.yaw_deg;

    position.x_mm = pose.x_mm;
    position.y_mm = pose.y_mm;
    g_path_distance_mm = PathPlanner_FindNearestDistance(&g_path,
                                                         position,
                                                         g_path_distance_mm);
    path_remaining_mm = g_path.length_mm - g_path_distance_mm;
    if(path_remaining_mm < 0.0f)
    {
        path_remaining_mm = 0.0f;
    }
#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
    if((path_remaining_mm <= NAV_FINAL_APPROACH_MM) &&
       ((g_mpc_initialized != 0U) || (g_mpc_solution_valid != 0U)))
    {
        Nav_MpcReset();
    }
#endif
    goal_dx = g_path.control[PATH_BSPLINE_CONTROL_COUNT - 1U].x_mm -
              pose.x_mm;
    goal_dy = g_path.control[PATH_BSPLINE_CONTROL_COUNT - 1U].y_mm -
              pose.y_mm;
    goal_distance = sqrtf((goal_dx * goal_dx) + (goal_dy * goal_dy));
    if((path_remaining_mm <= NAV_FINAL_APPROACH_MM) &&
       (goal_distance <= NAV_POSITION_TOLERANCE_MM))
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
            g_navigation_stop_reason_latched = NAV_STOP_REASON_FINISHED;
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
    if(path_remaining_mm <= NAV_FINAL_APPROACH_MM)
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
#if (NAV_CURVATURE_SPEED_LIMIT != 0U)
    /* 曲率限速：向心加速度设上限，弯道自动减速，直线不受影响 */
    if(lookahead.curvature > 0.0001f)
    {
        curvature_speed_m_s = sqrtf(NAV_MAX_LAT_ACCEL_MM_S2 /
                                    lookahead.curvature) * 0.001f;
        if(speed_m_s > curvature_speed_m_s)
        {
            speed_m_s = curvature_speed_m_s;
        }
    }
#endif
#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
    speed_m_s = Nav_MpcReferenceSpeed(&lookahead);
    if(Nav_MpcTrack(pose.x_mm,
                    pose.y_mm,
                    g_path_distance_mm,
                    now_ms,
                    (float)update_elapsed_ms * 0.001f,
                    &world_vx,
                    &world_vy) == 0U)
    {
#endif
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
#if (NAV_PATH_TRACKER_MPC_ENABLE != 0U)
        Nav_MpcRecordFallback(&world_vx,
                              &world_vy,
                              (float)update_elapsed_ms * 0.001f);
    }
#endif
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
        g_navigation_stop_reason_latched = NAV_STOP_REASON_INVALID_OUTPUT;
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
