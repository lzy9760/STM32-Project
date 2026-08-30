#include "odometry.h"

#include "chassis.h"
#include "hwt9018.h"
#include "motor.h"

#include <math.h>

#define ODOM_PI                              3.14159265358979323846f
#define ODOM_M3508_REDUCTION_RATIO           19.0f
#define ODOM_WHEEL_CIRCUMFERENCE_MM          296.52f
#define ODOM_MAX_DELTA_TIME_S                0.050f
#define ODOM_STRAFE_SCALE                    0.9776376f
#define ODOM_FORWARD_SCALE                   0.9875f
#define ODOM_WORLD_Y_SCALE                   1.0f
#define ODOM_ROTATION_ARM_MM                 (396.52f * 0.7071f)
#define ODOM_WHEEL_YAW_SCALE                 1.0f

#define ODOM_EKF_YAW_INITIAL_VAR_DEG2        0.0625f
#define ODOM_EKF_RATE_INITIAL_VAR_DPS2       25.0f
#define ODOM_EKF_BIAS_INITIAL_VAR_DPS2       0.0400f
#define ODOM_EKF_YAW_PROCESS_VAR_PER_S       0.0025f
#define ODOM_EKF_RATE_PROCESS_VAR_PER_S      50.0f
#define ODOM_EKF_BIAS_PROCESS_VAR_PER_S      0.0001f
#define ODOM_EKF_WHEEL_NOISE_BASE_DPS        3.0f
#define ODOM_EKF_WHEEL_NOISE_SPEED_GAIN      0.004f
#define ODOM_EKF_WHEEL_NOISE_RATE_GAIN       0.15f
#define ODOM_EKF_WHEEL_NOISE_MAX_DPS         15.0f
#define ODOM_EKF_WHEEL_HARD_GATE_DPS         100.0f
#define ODOM_EKF_GYRO_NOISE_DPS              1.0f
#define ODOM_EKF_GYRO_HARD_GATE_DPS          15.0f
#define ODOM_EKF_GYRO_NIS_GATE               9.0f
#define ODOM_EKF_BIAS_UPDATE_MAX_RATE_DPS    5.0f
#define ODOM_EKF_BIAS_UPDATE_MAX_GYRO_DPS    8.0f
#define ODOM_EKF_BIAS_LIMIT_DPS              0.5f
#define ODOM_EKF_RATE_LIMIT_DPS              180.0f

#define ODOM_GYRO_STALE_TIMEOUT_MS           200U
#define ODOM_IMU_MEASUREMENT_MAX_AGE_MS      50U
#define ODOM_GYRO_SIGN_RATE_MIN_DPS          3.0f
#define ODOM_GYRO_SIGN_RATIO_MIN             0.5f
#define ODOM_GYRO_SIGN_RATIO_MAX             2.0f
#define ODOM_GYRO_SIGN_SCORE_LIMIT           4

#define ODOM_YAW_ANCHOR_INNOVATION_GATE_DEG  180.0f
#define ODOM_YAW_ANCHOR_RATE_GATE_DPS        360.0f
#define ODOM_YAW_ANCHOR_STABLE_TIME_S        0.0f
#define ODOM_YAW_ANCHOR_MAX_INTERVAL_S       0.25f
#define ODOM_YAW_ANCHOR_GAIN_PER_S           0.08f //磁力计增益
#define ODOM_YAW_ANCHOR_MAX_DPS              0.6f
#define ODOM_YAW_ANCHOR_MAX_RATE_DPS         70.0f
#define ODOM_YAW_ANCHOR_MAX_SPEED_MM_S       3000.0f

/* 速率标度在线估计开关：默认关闭。
   实测磁力计速率作为参考不可靠，开启会污染EKF速率（低速误差变大），
   留作后续单变量实验再开 */
#define ODOM_RATE_SCALE_EST                  0U
#define ODOM_RATE_SCALE_MIN_RATE_DPS         10.0f
#define ODOM_RATE_SCALE_FOLD_DEN             200000.0f
#define ODOM_RATE_SCALE_LEARN_DAMPING        0.5f
#define ODOM_RATE_SCALE_MIN                  0.90f
#define ODOM_RATE_SCALE_MAX                  1.10f

static const float g_drive_direction[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
static OdometryPose g_pose;
static uint8_t g_initialized;
static float g_ekf_state[3];
static float g_ekf_covariance[3][3];
static uint32_t g_last_gyro_frame_count;
static uint32_t g_last_angle_frame_count;
static uint32_t g_last_angle_timestamp_ms;
static uint32_t g_last_gyro_accept_ms;
static float g_last_imu_yaw_deg;
static float g_angle_elapsed_s;
static float g_yaw_anchor_stable_time_s;
static int8_t g_gyro_sign_score;
static float g_previous_wheel_forward_raw_mm_s[4];
static float g_previous_wheel_strafe_raw_mm_s[4];
static float g_previous_ekf_yaw_deg;
static float g_previous_hwt_yaw_deg;
static uint8_t g_wheel_vector_history_valid;
static uint8_t g_yaw_history_valid;
static float g_rate_scale_num;
static float g_rate_scale_den;

volatile uint8_t g_odometry_ekf_active;
volatile uint8_t g_odometry_gyro_fusion_active;
volatile uint8_t g_odometry_gyro_sign_locked;
volatile uint8_t g_odometry_gyro_sign_fault;
volatile uint8_t g_odometry_gyro_measurement_accepted;
volatile uint8_t g_odometry_yaw_anchor_accepted;
volatile float g_odometry_gyro_runtime_sign;
volatile float g_odometry_ekf_yaw_deg;
volatile float g_odometry_ekf_bias_dps;
volatile float g_odometry_ekf_p_yaw_deg2;
volatile float g_odometry_ekf_p_rate_dps2;
volatile float g_odometry_ekf_p_bias_dps2;
volatile float g_odometry_wheel_yaw_rate_dps;
volatile float g_odometry_imu_gyro_rate_dps;
volatile float g_odometry_gyro_innovation_dps;
volatile float g_odometry_yaw_innovation_deg;
volatile float g_odometry_yaw_rate_error_dps;
volatile uint32_t g_odometry_gyro_age_ms;
volatile uint32_t g_odometry_gyro_accept_count;
volatile uint32_t g_odometry_gyro_reject_count;
volatile uint32_t g_odometry_yaw_anchor_accept_count;
volatile uint32_t g_odometry_yaw_anchor_reject_count;
volatile float g_odometry_shadow_ekf_x_mm;
volatile float g_odometry_shadow_ekf_y_mm;
volatile float g_odometry_shadow_hwt_x_mm;
volatile float g_odometry_shadow_hwt_y_mm;
volatile float g_odometry_shadow_hwt_minus_ekf_x_mm;
volatile float g_odometry_shadow_hwt_minus_ekf_y_mm;
volatile float g_odometry_shadow_yaw_diff_deg;
volatile float g_odometry_shadow_max_abs_yaw_diff_deg;
volatile float g_odometry_shadow_gyro_yaw_deg;
volatile float g_odometry_shadow_gyro_x_mm;
volatile float g_odometry_shadow_gyro_y_mm;
volatile float g_odometry_shadow_hwt_minus_gyro_yaw_deg;
volatile float g_odometry_shadow_max_abs_hwt_minus_gyro_yaw_deg;
volatile uint8_t g_odometry_shadow_gyro_valid;
volatile uint32_t g_odometry_shadow_gyro_invalid_count;
volatile uint8_t g_odometry_trapezoid_active;
volatile float g_odometry_midpoint_yaw_deg;
volatile float g_odometry_rate_scale = 1.0f;

static float Odom_WrapDeg(float angle)
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

static float Odom_MidpointDeg(float previous_deg, float current_deg)
{
    return Odom_WrapDeg(previous_deg +
                        0.5f * Odom_WrapDeg(current_deg - previous_deg));
}

static float Odom_Clamp(float value, float minimum, float maximum)
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

static uint8_t Odom_MotorFeedbackAllReady(void)
{
    uint8_t i;

    for(i = 0U; i < 8U; i++)
    {
        if(motors[i].feedback_valid == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

static uint32_t Odom_AgeMs(uint32_t now_ms, uint32_t timestamp_ms)
{
    int32_t signed_age_ms;

    signed_age_ms = (int32_t)(now_ms - timestamp_ms);
    if(signed_age_ms < 0)
    {
        return 0U;
    }
    return (uint32_t)signed_age_ms;
}

static void Odom_EkfLimitCovariance(void)
{
    float symmetric_value;
    uint8_t i;
    uint8_t j;

    for(i = 0U; i < 3U; i++)
    {
        if(g_ekf_covariance[i][i] < 0.00000001f)
        {
            g_ekf_covariance[i][i] = 0.00000001f;
        }
        else if(g_ekf_covariance[i][i] > 1000000.0f)
        {
            g_ekf_covariance[i][i] = 1000000.0f;
        }

        for(j = (uint8_t)(i + 1U); j < 3U; j++)
        {
            symmetric_value = 0.5f * (g_ekf_covariance[i][j] +
                                      g_ekf_covariance[j][i]);
            g_ekf_covariance[i][j] = symmetric_value;
            g_ekf_covariance[j][i] = symmetric_value;
        }
    }
}

static void Odom_EkfPredict(float delta_time_s)
{
    float dt2;
    float p00;
    float p01;
    float p02;
    float p10;
    float p11;
    float p12;
    float p20;
    float p21;
    float p22;

    dt2 = delta_time_s * delta_time_s;
    p00 = g_ekf_covariance[0][0] +
          delta_time_s * (g_ekf_covariance[0][1] +
                          g_ekf_covariance[1][0]) +
          dt2 * g_ekf_covariance[1][1] +
          ODOM_EKF_YAW_PROCESS_VAR_PER_S * delta_time_s;
    p01 = g_ekf_covariance[0][1] +
          delta_time_s * g_ekf_covariance[1][1];
    p02 = g_ekf_covariance[0][2] +
          delta_time_s * g_ekf_covariance[1][2];
    p10 = g_ekf_covariance[1][0] +
          delta_time_s * g_ekf_covariance[1][1];
    p11 = g_ekf_covariance[1][1] +
          ODOM_EKF_RATE_PROCESS_VAR_PER_S * delta_time_s;
    p12 = g_ekf_covariance[1][2];
    p20 = g_ekf_covariance[2][0] +
          delta_time_s * g_ekf_covariance[2][1];
    p21 = g_ekf_covariance[2][1];
    p22 = g_ekf_covariance[2][2] +
          ODOM_EKF_BIAS_PROCESS_VAR_PER_S * delta_time_s;

    g_ekf_state[0] = Odom_WrapDeg(g_ekf_state[0] +
                                   g_ekf_state[1] * delta_time_s);

    g_ekf_covariance[0][0] = p00;
    g_ekf_covariance[0][1] = p01;
    g_ekf_covariance[0][2] = p02;
    g_ekf_covariance[1][0] = p10;
    g_ekf_covariance[1][1] = p11;
    g_ekf_covariance[1][2] = p12;
    g_ekf_covariance[2][0] = p20;
    g_ekf_covariance[2][1] = p21;
    g_ekf_covariance[2][2] = p22;
    Odom_EkfLimitCovariance();
}

static uint8_t Odom_EkfUpdateScalar(float measurement,
                                    float h0,
                                    float h1,
                                    float h2,
                                    float measurement_variance,
                                    float hard_gate,
                                    float nis_gate,
                                    uint8_t wrap_innovation,
                                    uint8_t freeze_bias_state,
                                    float *innovation_output)
{
    float predicted_measurement;
    float innovation;
    float h[3];
    float ph[3];
    float kalman_gain[3];
    float innovation_variance;
    float nis;
    float old_covariance[3][3];
    float identity_minus_kh[3][3];
    float temporary_covariance[3][3];
    float updated_covariance[3][3];
    float covariance_sum;
    float unclamped_rate_dps;
    float unclamped_bias_dps;
    uint8_t i;
    uint8_t j;
    uint8_t k;

    h[0] = h0;
    h[1] = h1;
    h[2] = h2;

    predicted_measurement = h0 * g_ekf_state[0] +
                            h1 * g_ekf_state[1] +
                            h2 * g_ekf_state[2];
    innovation = measurement - predicted_measurement;
    if(wrap_innovation != 0U)
    {
        innovation = Odom_WrapDeg(innovation);
    }
    if(innovation_output != 0)
    {
        *innovation_output = innovation;
    }

    ph[0] = g_ekf_covariance[0][0] * h0 +
            g_ekf_covariance[0][1] * h1 +
            g_ekf_covariance[0][2] * h2;
    ph[1] = g_ekf_covariance[1][0] * h0 +
            g_ekf_covariance[1][1] * h1 +
            g_ekf_covariance[1][2] * h2;
    ph[2] = g_ekf_covariance[2][0] * h0 +
            g_ekf_covariance[2][1] * h1 +
            g_ekf_covariance[2][2] * h2;
    innovation_variance = h0 * ph[0] + h1 * ph[1] +
                          h2 * ph[2] + measurement_variance;
    if(innovation_variance <= 0.00000001f)
    {
        return 0U;
    }
    if((hard_gate > 0.0f) && (fabsf(innovation) > hard_gate))
    {
        return 0U;
    }
    nis = (innovation * innovation) / innovation_variance;
    if((nis_gate > 0.0f) && (nis > nis_gate))
    {
        return 0U;
    }

    kalman_gain[0] = ph[0] / innovation_variance;
    kalman_gain[1] = ph[1] / innovation_variance;
    kalman_gain[2] = ph[2] / innovation_variance;
    if(freeze_bias_state != 0U)
    {
        kalman_gain[2] = 0.0f;
    }
    g_ekf_state[0] = Odom_WrapDeg(g_ekf_state[0] +
                                   kalman_gain[0] * innovation);
    g_ekf_state[1] += kalman_gain[1] * innovation;
    g_ekf_state[2] += kalman_gain[2] * innovation;

    for(i = 0U; i < 3U; i++)
    {
        for(j = 0U; j < 3U; j++)
        {
            old_covariance[i][j] = g_ekf_covariance[i][j];
            identity_minus_kh[i][j] =
                ((i == j) ? 1.0f : 0.0f) -
                kalman_gain[i] * h[j];
        }
    }

    for(i = 0U; i < 3U; i++)
    {
        for(j = 0U; j < 3U; j++)
        {
            covariance_sum = 0.0f;
            for(k = 0U; k < 3U; k++)
            {
                covariance_sum += identity_minus_kh[i][k] *
                                  old_covariance[k][j];
            }
            temporary_covariance[i][j] = covariance_sum;
        }
    }

    for(i = 0U; i < 3U; i++)
    {
        for(j = 0U; j < 3U; j++)
        {
            covariance_sum = 0.0f;
            for(k = 0U; k < 3U; k++)
            {
                covariance_sum += temporary_covariance[i][k] *
                                  identity_minus_kh[j][k];
            }
            updated_covariance[i][j] = covariance_sum +
                                       kalman_gain[i] *
                                       measurement_variance *
                                       kalman_gain[j];
        }
    }
    for(i = 0U; i < 3U; i++)
    {
        for(j = 0U; j < 3U; j++)
        {
            g_ekf_covariance[i][j] = updated_covariance[i][j];
        }
    }

    unclamped_rate_dps = g_ekf_state[1];
    unclamped_bias_dps = g_ekf_state[2];
    g_ekf_state[1] = Odom_Clamp(g_ekf_state[1],
                                -ODOM_EKF_RATE_LIMIT_DPS,
                                ODOM_EKF_RATE_LIMIT_DPS);
    g_ekf_state[2] = Odom_Clamp(g_ekf_state[2],
                                -ODOM_EKF_BIAS_LIMIT_DPS,
                                ODOM_EKF_BIAS_LIMIT_DPS);
    if(g_ekf_state[1] != unclamped_rate_dps)
    {
        g_ekf_covariance[1][1] += 25.0f;
    }
    if(g_ekf_state[2] != unclamped_bias_dps)
    {
        g_ekf_covariance[2][2] += ODOM_EKF_BIAS_INITIAL_VAR_DPS2;
    }
    Odom_EkfLimitCovariance();
    return 1U;
}

void Odometry_Init(void)
{
    HWT9018Snapshot imu_snapshot;
    uint8_t i;
    uint8_t j;

    HWT9018_GetSnapshot(&imu_snapshot);
    g_ekf_state[0] = imu_snapshot.yaw_deg;
    g_ekf_state[1] = 0.0f;
    g_ekf_state[2] = 0.0f;
    for(i = 0U; i < 3U; i++)
    {
        for(j = 0U; j < 3U; j++)
        {
            g_ekf_covariance[i][j] = 0.0f;
        }
    }
    g_ekf_covariance[0][0] = ODOM_EKF_YAW_INITIAL_VAR_DEG2;
    g_ekf_covariance[1][1] = ODOM_EKF_RATE_INITIAL_VAR_DPS2;
    g_ekf_covariance[2][2] = ODOM_EKF_BIAS_INITIAL_VAR_DPS2;

    g_pose.x_mm = 0.0f;
    g_pose.y_mm = 0.0f;
    g_pose.yaw_deg = g_ekf_state[0];
    g_last_gyro_frame_count = imu_snapshot.gyro_frame_count;
    g_last_angle_frame_count = imu_snapshot.angle_frame_count;
    g_last_angle_timestamp_ms = imu_snapshot.angle_timestamp_ms;
    g_last_gyro_accept_ms = 0U;
    g_last_imu_yaw_deg = imu_snapshot.yaw_deg;
    g_angle_elapsed_s = 0.0f;
    g_yaw_anchor_stable_time_s = 0.0f;
    g_gyro_sign_score = 0;
    for(i = 0U; i < 4U; i++)
    {
        g_previous_wheel_forward_raw_mm_s[i] = 0.0f;
        g_previous_wheel_strafe_raw_mm_s[i] = 0.0f;
    }
    g_previous_ekf_yaw_deg = g_ekf_state[0];
    g_previous_hwt_yaw_deg = imu_snapshot.yaw_deg;
    g_wheel_vector_history_valid = 0U;
    g_yaw_history_valid = 1U;

    g_odometry_ekf_active = 1U;
    g_odometry_gyro_fusion_active = 0U;
    g_odometry_gyro_sign_locked = 0U;
    g_odometry_gyro_sign_fault = 0U;
    g_odometry_gyro_measurement_accepted = 0U;
    g_odometry_yaw_anchor_accepted = 0U;
    g_odometry_gyro_runtime_sign = 1.0f;
    g_odometry_ekf_yaw_deg = g_ekf_state[0];
    g_odometry_ekf_bias_dps = 0.0f;
    g_odometry_ekf_p_yaw_deg2 = g_ekf_covariance[0][0];
    g_odometry_ekf_p_rate_dps2 = g_ekf_covariance[1][1];
    g_odometry_ekf_p_bias_dps2 = g_ekf_covariance[2][2];
    g_odometry_wheel_yaw_rate_dps = 0.0f;
    g_odometry_imu_gyro_rate_dps = 0.0f;
    g_odometry_gyro_innovation_dps = 0.0f;
    g_odometry_yaw_innovation_deg = 0.0f;
    g_odometry_yaw_rate_error_dps = 0.0f;
    g_odometry_gyro_age_ms = 0xFFFFFFFFU;
    g_odometry_gyro_accept_count = 0U;
    g_odometry_gyro_reject_count = 0U;
    g_odometry_yaw_anchor_accept_count = 0U;
    g_odometry_yaw_anchor_reject_count = 0U;
    g_odometry_shadow_ekf_x_mm = 0.0f;
    g_odometry_shadow_ekf_y_mm = 0.0f;
    g_odometry_shadow_hwt_x_mm = 0.0f;
    g_odometry_shadow_hwt_y_mm = 0.0f;
    g_odometry_shadow_hwt_minus_ekf_x_mm = 0.0f;
    g_odometry_shadow_hwt_minus_ekf_y_mm = 0.0f;
    g_odometry_shadow_yaw_diff_deg = 0.0f;
    g_odometry_shadow_max_abs_yaw_diff_deg = 0.0f;
    g_odometry_shadow_gyro_yaw_deg = imu_snapshot.yaw_deg;
    g_odometry_shadow_gyro_x_mm = 0.0f;
    g_odometry_shadow_gyro_y_mm = 0.0f;
    g_odometry_shadow_hwt_minus_gyro_yaw_deg = 0.0f;
    g_odometry_shadow_max_abs_hwt_minus_gyro_yaw_deg = 0.0f;
    g_odometry_shadow_gyro_valid = 0U;
    g_odometry_shadow_gyro_invalid_count = 0U;
    g_odometry_trapezoid_active = 0U;
    g_odometry_midpoint_yaw_deg = g_ekf_state[0];
    g_rate_scale_num = 0.0f;
    g_rate_scale_den = 0.0f;
    /* 保留上一圈学到的标度，只在异常时复位 */
    if((g_odometry_rate_scale < ODOM_RATE_SCALE_MIN) ||
       (g_odometry_rate_scale > ODOM_RATE_SCALE_MAX))
    {
        g_odometry_rate_scale = 1.0f;
    }
    g_initialized = 1U;
}

void Odometry_Update(float delta_time_s, uint32_t now_ms)
{
    uint8_t i;
    float wheel_speed_mm_s;
    float wheel_heading_rad;
    float wheel_forward_raw_mm_s;
    float wheel_strafe_raw_mm_s;
    float wheel_forward_raw_mm;
    float wheel_strafe_raw_mm;
    float wheel_forward_mm;
    float wheel_strafe_mm;
    float body_forward_mm;
    float body_strafe_mm;
    float current_body_forward_mm_s;
    float current_body_strafe_mm_s;
    float body_speed_mm_s;
    float rotation_numerator_mm;
    float wheel_yaw_delta_rad;
    float wheel_yaw_delta_deg;
    float wheel_yaw_rate_dps;
    float wheel_noise_dps;
    float wheel_measurement_variance;
    HWT9018Snapshot imu_snapshot;
    float gyro_rate_dps;
    float gyro_rate_abs;
    float wheel_rate_abs;
    float gyro_wheel_ratio;
    float signed_gyro_rate_dps;
    float gyro_measurement_variance;
    float gyro_innovation_dps;
    float angle_delta_deg;
    float angle_delta_time_s;
    float angle_rate_dps;
    float yaw_error_deg;
    float yaw_correction_deg;
    float yaw_correction_limit_deg;
    float yaw_deg;
    float midpoint_yaw_deg;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float hwt_midpoint_yaw_deg;
    float hwt_yaw_rad;
    float hwt_cos_yaw;
    float hwt_sin_yaw;
    float shadow_yaw_diff_deg;
    float shadow_abs_yaw_diff_deg;
    float gyro_shadow_rate_dps;
    float previous_gyro_shadow_yaw_deg;
    float gyro_midpoint_yaw_deg;
    float gyro_yaw_rad;
    float gyro_cos_yaw;
    float gyro_sin_yaw;
    float hwt_minus_gyro_yaw_deg;
    float abs_hwt_minus_gyro_yaw_deg;
    uint32_t angle_delta_time_ms;
    uint32_t angle_age_ms;
    uint8_t wheel_feedback_ready;
    uint8_t new_gyro_frame;
    uint8_t new_angle_frame;
    uint8_t gyro_measurement_ok;
    uint8_t freeze_gyro_bias;

    if((g_initialized == 0U) || (delta_time_s <= 0.0f))
    {
        return;
    }
    if(delta_time_s > ODOM_MAX_DELTA_TIME_S)
    {
        delta_time_s = ODOM_MAX_DELTA_TIME_S;
        g_wheel_vector_history_valid = 0U;
        g_yaw_history_valid = 0U;
    }

    wheel_feedback_ready = Odom_MotorFeedbackAllReady();
    if(wheel_feedback_ready == 0U)
    {
        g_wheel_vector_history_valid = 0U;
    }
    g_odometry_trapezoid_active = g_wheel_vector_history_valid;
    body_forward_mm = 0.0f;
    body_strafe_mm = 0.0f;
    current_body_forward_mm_s = 0.0f;
    current_body_strafe_mm_s = 0.0f;
    rotation_numerator_mm = 0.0f;
    for(i = 0U; i < 4U; i++)
    {
        wheel_speed_mm_s = (float)motors[i + 4U].rpm *
                           g_drive_direction[i] *
                           ODOM_WHEEL_CIRCUMFERENCE_MM /
                           (60.0f * ODOM_M3508_REDUCTION_RATIO);
        wheel_heading_rad = chassis_get_wheel_heading_deg(i) *
                            ODOM_PI / 180.0f;
        wheel_forward_raw_mm_s = wheel_speed_mm_s *
                                 cosf(wheel_heading_rad);
        wheel_strafe_raw_mm_s = wheel_speed_mm_s *
                                sinf(wheel_heading_rad);
        if(g_odometry_trapezoid_active != 0U)
        {
            wheel_forward_raw_mm =
                0.5f * (g_previous_wheel_forward_raw_mm_s[i] +
                        wheel_forward_raw_mm_s) * delta_time_s;
            wheel_strafe_raw_mm =
                0.5f * (g_previous_wheel_strafe_raw_mm_s[i] +
                        wheel_strafe_raw_mm_s) * delta_time_s;
        }
        else
        {
            wheel_forward_raw_mm = wheel_forward_raw_mm_s * delta_time_s;
            wheel_strafe_raw_mm = wheel_strafe_raw_mm_s * delta_time_s;
        }
        g_previous_wheel_forward_raw_mm_s[i] =
            wheel_forward_raw_mm_s;
        g_previous_wheel_strafe_raw_mm_s[i] =
            wheel_strafe_raw_mm_s;
        wheel_forward_mm = wheel_forward_raw_mm * ODOM_FORWARD_SCALE;
        wheel_strafe_mm = wheel_strafe_raw_mm * ODOM_STRAFE_SCALE;
        body_forward_mm += wheel_forward_mm;
        body_strafe_mm += wheel_strafe_mm;
        current_body_forward_mm_s +=
            wheel_forward_raw_mm_s * ODOM_FORWARD_SCALE;
        current_body_strafe_mm_s +=
            wheel_strafe_raw_mm_s * ODOM_STRAFE_SCALE;

        switch(i)
        {
            case 0U:
                rotation_numerator_mm +=
                    (-wheel_forward_raw_mm_s +
                     wheel_strafe_raw_mm_s) * delta_time_s;
                break;
            case 1U:
                rotation_numerator_mm +=
                    (wheel_forward_raw_mm_s +
                     wheel_strafe_raw_mm_s) * delta_time_s;
                break;
            case 2U:
                rotation_numerator_mm +=
                    (wheel_forward_raw_mm_s -
                     wheel_strafe_raw_mm_s) * delta_time_s;
                break;
            default:
                rotation_numerator_mm +=
                    (-wheel_forward_raw_mm_s -
                     wheel_strafe_raw_mm_s) * delta_time_s;
                break;
        }
    }
    g_wheel_vector_history_valid = wheel_feedback_ready;
    body_forward_mm *= 0.25f;
    body_strafe_mm *= 0.25f;
    current_body_forward_mm_s *= 0.25f;
    current_body_strafe_mm_s *= 0.25f;
    wheel_yaw_delta_rad = rotation_numerator_mm /
                          (8.0f * ODOM_ROTATION_ARM_MM);
    wheel_yaw_delta_deg = -wheel_yaw_delta_rad *
                          180.0f / ODOM_PI *
                          ODOM_WHEEL_YAW_SCALE;
    wheel_yaw_rate_dps = wheel_yaw_delta_deg / delta_time_s;
    g_odometry_wheel_yaw_rate_dps = wheel_yaw_rate_dps;

    Odom_EkfPredict(delta_time_s);
    body_speed_mm_s = sqrtf(
        (current_body_forward_mm_s * current_body_forward_mm_s) +
        (current_body_strafe_mm_s * current_body_strafe_mm_s));
    wheel_noise_dps = ODOM_EKF_WHEEL_NOISE_BASE_DPS +
                      ODOM_EKF_WHEEL_NOISE_SPEED_GAIN * body_speed_mm_s +
                      ODOM_EKF_WHEEL_NOISE_RATE_GAIN *
                      fabsf(wheel_yaw_rate_dps);
    if(wheel_noise_dps > ODOM_EKF_WHEEL_NOISE_MAX_DPS)
    {
        wheel_noise_dps = ODOM_EKF_WHEEL_NOISE_MAX_DPS;
    }
    wheel_measurement_variance = wheel_noise_dps * wheel_noise_dps;
    if(wheel_feedback_ready != 0U)
    {
        (void)Odom_EkfUpdateScalar(wheel_yaw_rate_dps *
                                   g_odometry_rate_scale,
                                   0.0f,
                                   1.0f,
                                   0.0f,
                                   wheel_measurement_variance,
                                   ODOM_EKF_WHEEL_HARD_GATE_DPS,
                                   0.0f,
                                   0U,
                                   1U,
                                   0);
    }

    HWT9018_GetSnapshot(&imu_snapshot);
    g_odometry_gyro_measurement_accepted = 0U;
    g_odometry_yaw_anchor_accepted = 0U;
    new_gyro_frame = ((imu_snapshot.gyro_valid != 0U) &&
                      (imu_snapshot.gyro_frame_count !=
                       g_last_gyro_frame_count)) ? 1U : 0U;
    if(imu_snapshot.gyro_timestamp_ms != 0U)
    {
        g_odometry_gyro_age_ms =
            Odom_AgeMs(now_ms, imu_snapshot.gyro_timestamp_ms);
    }
    else
    {
        g_odometry_gyro_age_ms = 0xFFFFFFFFU;
    }

    gyro_rate_dps = imu_snapshot.gyro_z_deg_s;
    if(new_gyro_frame != 0U)
    {
        g_last_gyro_frame_count = imu_snapshot.gyro_frame_count;
        gyro_rate_abs = fabsf(gyro_rate_dps);
        wheel_rate_abs = fabsf(wheel_yaw_rate_dps);
        if((g_odometry_gyro_sign_locked == 0U) &&
           (g_odometry_gyro_sign_fault == 0U) &&
           (wheel_feedback_ready != 0U) &&
           (imu_snapshot.gyro_bias_valid != 0U) &&
           (g_odometry_gyro_age_ms <= ODOM_IMU_MEASUREMENT_MAX_AGE_MS) &&
           (gyro_rate_abs >= ODOM_GYRO_SIGN_RATE_MIN_DPS) &&
           (wheel_rate_abs >= ODOM_GYRO_SIGN_RATE_MIN_DPS))
        {
            gyro_wheel_ratio = gyro_rate_abs / wheel_rate_abs;
            if((gyro_wheel_ratio >= ODOM_GYRO_SIGN_RATIO_MIN) &&
               (gyro_wheel_ratio <= ODOM_GYRO_SIGN_RATIO_MAX))
            {
                if((gyro_rate_dps * wheel_yaw_rate_dps) >= 0.0f)
                {
                    if(g_gyro_sign_score < ODOM_GYRO_SIGN_SCORE_LIMIT)
                    {
                        g_gyro_sign_score++;
                    }
                }
                else
                {
                    if(g_gyro_sign_score > -ODOM_GYRO_SIGN_SCORE_LIMIT)
                    {
                        g_gyro_sign_score--;
                    }
                }
                if(g_gyro_sign_score >= ODOM_GYRO_SIGN_SCORE_LIMIT)
                {
                    g_odometry_gyro_runtime_sign = 1.0f;
                    g_odometry_gyro_sign_locked = 1U;
                }
                else if(g_gyro_sign_score <= -ODOM_GYRO_SIGN_SCORE_LIMIT)
                {
                    g_odometry_gyro_runtime_sign = 1.0f;
                    g_odometry_gyro_sign_fault = 1U;
                }
            }
        }

        signed_gyro_rate_dps = gyro_rate_dps *
                               g_odometry_gyro_runtime_sign;
        g_odometry_imu_gyro_rate_dps = signed_gyro_rate_dps;
        if((g_odometry_gyro_sign_locked != 0U) &&
           (imu_snapshot.gyro_bias_valid != 0U) &&
           (g_odometry_gyro_age_ms <= ODOM_IMU_MEASUREMENT_MAX_AGE_MS))
        {
            gyro_measurement_variance = ODOM_EKF_GYRO_NOISE_DPS *
                                         ODOM_EKF_GYRO_NOISE_DPS;
            if((wheel_feedback_ready != 0U) &&
               (fabsf(wheel_yaw_rate_dps) <=
                ODOM_EKF_BIAS_UPDATE_MAX_RATE_DPS) &&
               (fabsf(signed_gyro_rate_dps) <=
                ODOM_EKF_BIAS_UPDATE_MAX_GYRO_DPS))
            {
                freeze_gyro_bias = 0U;
            }
            else
            {
                freeze_gyro_bias = 1U;
            }
            gyro_measurement_ok = Odom_EkfUpdateScalar(
                signed_gyro_rate_dps * g_odometry_rate_scale,
                0.0f,
                1.0f,
                1.0f,
                gyro_measurement_variance,
                ODOM_EKF_GYRO_HARD_GATE_DPS,
                ODOM_EKF_GYRO_NIS_GATE,
                0U,
                freeze_gyro_bias,
                &gyro_innovation_dps);
            g_odometry_gyro_innovation_dps = gyro_innovation_dps;
            if(gyro_measurement_ok != 0U)
            {
                g_last_gyro_accept_ms = now_ms;
                g_odometry_gyro_measurement_accepted = 1U;
                g_odometry_gyro_accept_count++;
            }
            else
            {
                g_odometry_gyro_reject_count++;
            }
        }
    }
    if((g_last_gyro_accept_ms != 0U) &&
       ((uint32_t)(now_ms - g_last_gyro_accept_ms) <=
        ODOM_GYRO_STALE_TIMEOUT_MS))
    {
        g_odometry_gyro_fusion_active = 1U;
    }
    else
    {
        g_odometry_gyro_fusion_active = 0U;
    }

    g_angle_elapsed_s += delta_time_s;
    new_angle_frame = ((imu_snapshot.yaw_valid != 0U) &&
                       (imu_snapshot.angle_frame_count !=
                        g_last_angle_frame_count)) ? 1U : 0U;
    if(new_angle_frame != 0U)
    {
        g_last_angle_frame_count = imu_snapshot.angle_frame_count;
        angle_delta_time_s = g_angle_elapsed_s;
        if((g_last_angle_timestamp_ms != 0U) &&
           (imu_snapshot.angle_timestamp_ms != 0U))
        {
            angle_delta_time_ms =
                (uint32_t)(imu_snapshot.angle_timestamp_ms -
                           g_last_angle_timestamp_ms);
            if((angle_delta_time_ms > 0U) &&
               (angle_delta_time_ms <= 500U))
            {
                angle_delta_time_s =
                    (float)angle_delta_time_ms * 0.001f;
            }
        }
        angle_age_ms = Odom_AgeMs(now_ms,
                                  imu_snapshot.angle_timestamp_ms);
        angle_delta_deg = Odom_WrapDeg(imu_snapshot.yaw_deg -
                                       g_last_imu_yaw_deg);
        yaw_error_deg = Odom_WrapDeg(imu_snapshot.yaw_deg -
                                     g_ekf_state[0]);
        g_odometry_yaw_innovation_deg = yaw_error_deg;
        if(angle_delta_time_s > 0.0001f)
        {
            angle_rate_dps = angle_delta_deg / angle_delta_time_s;
            g_odometry_yaw_rate_error_dps =
                fabsf(angle_rate_dps - g_ekf_state[1]);
        }
        else
        {
            g_odometry_yaw_rate_error_dps = 1000.0f;
        }

        if((angle_age_ms <= ODOM_IMU_MEASUREMENT_MAX_AGE_MS) &&
           (angle_delta_time_s <= ODOM_YAW_ANCHOR_MAX_INTERVAL_S) &&
           (fabsf(yaw_error_deg) <=
            ODOM_YAW_ANCHOR_INNOVATION_GATE_DEG) &&
           (g_odometry_yaw_rate_error_dps <=
            ODOM_YAW_ANCHOR_RATE_GATE_DPS) &&
           /* 运动状态门控：只在低速/接近直行时允许磁力计修正 */
           (fabsf(wheel_yaw_rate_dps) <=
            ODOM_YAW_ANCHOR_MAX_RATE_DPS) &&
           (body_speed_mm_s <= ODOM_YAW_ANCHOR_MAX_SPEED_MM_S))
        {
            if(g_yaw_anchor_stable_time_s <
               ODOM_YAW_ANCHOR_STABLE_TIME_S)
            {
                g_yaw_anchor_stable_time_s += angle_delta_time_s;
            }
            if(g_yaw_anchor_stable_time_s >=
               ODOM_YAW_ANCHOR_STABLE_TIME_S)
            {
                yaw_correction_deg = yaw_error_deg *
                                     ODOM_YAW_ANCHOR_GAIN_PER_S *
                                     angle_delta_time_s;
                yaw_correction_limit_deg =
                    ODOM_YAW_ANCHOR_MAX_DPS * angle_delta_time_s;
                yaw_correction_deg = Odom_Clamp(
                    yaw_correction_deg,
                    -yaw_correction_limit_deg,
                    yaw_correction_limit_deg);
                g_ekf_state[0] = Odom_WrapDeg(g_ekf_state[0] +
                                               yaw_correction_deg);
                g_odometry_yaw_anchor_accepted = 1U;
                g_odometry_yaw_anchor_accept_count++;
#if (ODOM_RATE_SCALE_EST != 0U)
                /* 速率标度在线估计：以磁力计速率为基准，
                   拟合EKF角速率的比例误差并修正 */
                if((fabsf(angle_rate_dps) >=
                    ODOM_RATE_SCALE_MIN_RATE_DPS) &&
                   (fabsf(g_ekf_state[1]) >=
                    ODOM_RATE_SCALE_MIN_RATE_DPS))
                {
                    g_rate_scale_num +=
                        (angle_rate_dps - g_ekf_state[1]) *
                        g_ekf_state[1];
                    g_rate_scale_den +=
                        g_ekf_state[1] * g_ekf_state[1];
                    if(g_rate_scale_den >= ODOM_RATE_SCALE_FOLD_DEN)
                    {
                        g_odometry_rate_scale = Odom_Clamp(
                            g_odometry_rate_scale +
                            ODOM_RATE_SCALE_LEARN_DAMPING *
                            (g_rate_scale_num / g_rate_scale_den),
                            ODOM_RATE_SCALE_MIN,
                            ODOM_RATE_SCALE_MAX);
                        g_rate_scale_num = 0.0f;
                        g_rate_scale_den = 0.0f;
                    }
                }
#endif
            }
        }
        else
        {
            g_yaw_anchor_stable_time_s = 0.0f;
            g_odometry_yaw_anchor_reject_count++;
        }
        g_last_angle_timestamp_ms = imu_snapshot.angle_timestamp_ms;
        g_last_imu_yaw_deg = imu_snapshot.yaw_deg;
        g_angle_elapsed_s = 0.0f;
    }

    g_odometry_ekf_yaw_deg = g_ekf_state[0];
    g_odometry_ekf_bias_dps = g_ekf_state[2];
    g_odometry_ekf_p_yaw_deg2 = g_ekf_covariance[0][0];
    g_odometry_ekf_p_rate_dps2 = g_ekf_covariance[1][1];
    g_odometry_ekf_p_bias_dps2 = g_ekf_covariance[2][2];
    yaw_deg = g_ekf_state[0];
    if(g_yaw_history_valid != 0U)
    {
        midpoint_yaw_deg = Odom_MidpointDeg(g_previous_ekf_yaw_deg,
                                            yaw_deg);
        hwt_midpoint_yaw_deg = Odom_MidpointDeg(
            g_previous_hwt_yaw_deg,
            imu_snapshot.yaw_deg);
    }
    else
    {
        midpoint_yaw_deg = yaw_deg;
        hwt_midpoint_yaw_deg = imu_snapshot.yaw_deg;
    }
    g_odometry_midpoint_yaw_deg = midpoint_yaw_deg;
    yaw_rad = -midpoint_yaw_deg * ODOM_PI / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    g_pose.x_mm += (body_forward_mm * cos_yaw) -
                   (body_strafe_mm * sin_yaw);
    g_pose.y_mm += ((body_forward_mm * sin_yaw) +
                    (body_strafe_mm * cos_yaw)) *
                   ODOM_WORLD_Y_SCALE;
    g_pose.yaw_deg = yaw_deg;

    g_odometry_shadow_ekf_x_mm +=
        (body_forward_mm * cos_yaw) -
        (body_strafe_mm * sin_yaw);
    g_odometry_shadow_ekf_y_mm +=
        ((body_forward_mm * sin_yaw) +
         (body_strafe_mm * cos_yaw)) *
        ODOM_WORLD_Y_SCALE;

    hwt_yaw_rad = -hwt_midpoint_yaw_deg * ODOM_PI / 180.0f;
    hwt_cos_yaw = cosf(hwt_yaw_rad);
    hwt_sin_yaw = sinf(hwt_yaw_rad);
    g_odometry_shadow_hwt_x_mm +=
        (body_forward_mm * hwt_cos_yaw) -
        (body_strafe_mm * hwt_sin_yaw);
    g_odometry_shadow_hwt_y_mm +=
        ((body_forward_mm * hwt_sin_yaw) +
         (body_strafe_mm * hwt_cos_yaw)) *
        ODOM_WORLD_Y_SCALE;

    g_odometry_shadow_hwt_minus_ekf_x_mm =
        g_odometry_shadow_hwt_x_mm -
        g_odometry_shadow_ekf_x_mm;
    g_odometry_shadow_hwt_minus_ekf_y_mm =
        g_odometry_shadow_hwt_y_mm -
        g_odometry_shadow_ekf_y_mm;
    shadow_yaw_diff_deg = Odom_WrapDeg(imu_snapshot.yaw_deg - yaw_deg);
    g_odometry_shadow_yaw_diff_deg = shadow_yaw_diff_deg;
    shadow_abs_yaw_diff_deg = fabsf(shadow_yaw_diff_deg);
    if(shadow_abs_yaw_diff_deg >
       g_odometry_shadow_max_abs_yaw_diff_deg)
    {
        g_odometry_shadow_max_abs_yaw_diff_deg =
            shadow_abs_yaw_diff_deg;
    }

    previous_gyro_shadow_yaw_deg = g_odometry_shadow_gyro_yaw_deg;
    gyro_shadow_rate_dps = imu_snapshot.gyro_z_deg_s *
                           g_odometry_gyro_runtime_sign;
    if((imu_snapshot.gyro_valid != 0U) &&
       (imu_snapshot.gyro_bias_valid != 0U) &&
       (g_odometry_gyro_age_ms <= ODOM_IMU_MEASUREMENT_MAX_AGE_MS) &&
       (g_odometry_gyro_sign_fault == 0U))
    {
        g_odometry_shadow_gyro_yaw_deg = Odom_WrapDeg(
            g_odometry_shadow_gyro_yaw_deg +
            gyro_shadow_rate_dps * delta_time_s);
        g_odometry_shadow_gyro_valid = 1U;
    }
    else
    {
        g_odometry_shadow_gyro_valid = 0U;
        g_odometry_shadow_gyro_invalid_count++;
    }

    if(g_yaw_history_valid != 0U)
    {
        gyro_midpoint_yaw_deg = Odom_MidpointDeg(
            previous_gyro_shadow_yaw_deg,
            g_odometry_shadow_gyro_yaw_deg);
    }
    else
    {
        gyro_midpoint_yaw_deg = g_odometry_shadow_gyro_yaw_deg;
    }
    gyro_yaw_rad = -gyro_midpoint_yaw_deg *
                   ODOM_PI / 180.0f;
    gyro_cos_yaw = cosf(gyro_yaw_rad);
    gyro_sin_yaw = sinf(gyro_yaw_rad);
    g_odometry_shadow_gyro_x_mm +=
        (body_forward_mm * gyro_cos_yaw) -
        (body_strafe_mm * gyro_sin_yaw);
    g_odometry_shadow_gyro_y_mm +=
        ((body_forward_mm * gyro_sin_yaw) +
         (body_strafe_mm * gyro_cos_yaw)) *
        ODOM_WORLD_Y_SCALE;

    hwt_minus_gyro_yaw_deg = Odom_WrapDeg(
        imu_snapshot.yaw_deg - g_odometry_shadow_gyro_yaw_deg);
    g_odometry_shadow_hwt_minus_gyro_yaw_deg =
        hwt_minus_gyro_yaw_deg;
    abs_hwt_minus_gyro_yaw_deg = fabsf(hwt_minus_gyro_yaw_deg);
    if(abs_hwt_minus_gyro_yaw_deg >
       g_odometry_shadow_max_abs_hwt_minus_gyro_yaw_deg)
    {
        g_odometry_shadow_max_abs_hwt_minus_gyro_yaw_deg =
            abs_hwt_minus_gyro_yaw_deg;
    }
    g_previous_ekf_yaw_deg = yaw_deg;
    g_previous_hwt_yaw_deg = imu_snapshot.yaw_deg;
    g_yaw_history_valid = 1U;
}

OdometryPose Odometry_GetPose(void)
{
    return g_pose;
}
