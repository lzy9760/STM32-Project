#include "odometry.h"

#include "chassis.h"
#include "hwt9018.h"
#include "motor.h"

#include <math.h>

#define ODOM_PI                         3.14159265358979323846f
#define ODOM_M3508_REDUCTION_RATIO      19.0f
#define ODOM_WHEEL_CIRCUMFERENCE_MM     296.52f
#define ODOM_MAX_DELTA_TIME_S           0.050f
#define ODOM_STRAFE_SCALE               0.8667f
#define ODOM_FORWARD_SCALE              0.8875f

#define ODOM_WORLD_Y_SCALE             1.1280f


#define ODOM_ROTATION_ARM_MM             (396.52f * 0.7071f)
#define ODOM_IMU_YAW_CORRECTION_PER_S    2.0f
#define ODOM_IMU_YAW_ERROR_LIMIT_DEG     5.0f


static const float g_drive_direction[4] = {-1.0f, 1.0f, 1.0f, -1.0f};
static OdometryPose g_pose;
static uint8_t g_initialized;
static float g_fused_yaw_deg;


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


void Odometry_Init(void)
{
    g_fused_yaw_deg = HWT9018_GetYawDeg();

    g_pose.x_mm = 0.0f;
    g_pose.y_mm = 0.0f;
    g_pose.yaw_deg = g_fused_yaw_deg;
    g_initialized = 1U;
}

void Odometry_Update(float delta_time_s)
{
    uint8_t i;
    float wheel_distance_mm;
    float wheel_heading_rad;
    float wheel_forward_mm;
    float wheel_strafe_mm;
    float body_forward_mm;
    float body_strafe_mm;
    float rotation_numerator_mm;
    float wheel_yaw_delta_rad;
    float wheel_yaw_delta_deg;
    float imu_yaw_deg;
    float yaw_error_deg;
    float yaw_deg;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;

    if((g_initialized == 0U) || (delta_time_s <= 0.0f))
    {
        return;
    }

    if(delta_time_s > ODOM_MAX_DELTA_TIME_S)
    {
        delta_time_s = ODOM_MAX_DELTA_TIME_S;
    }

    body_forward_mm = 0.0f;
    body_strafe_mm = 0.0f;
    rotation_numerator_mm = 0.0f;

    for(i = 0U; i < 4U; i++)
    {
        wheel_distance_mm = (float)motors[i + 4U].rpm *
                            g_drive_direction[i] *
                            ODOM_WHEEL_CIRCUMFERENCE_MM *
                            delta_time_s /
                            (60.0f * ODOM_M3508_REDUCTION_RATIO);

        wheel_heading_rad = chassis_get_wheel_heading_deg(i) *
                            ODOM_PI / 180.0f;

        wheel_forward_mm = wheel_distance_mm *
                           cosf(wheel_heading_rad) *
                           ODOM_FORWARD_SCALE;

        wheel_strafe_mm = wheel_distance_mm *
                          sinf(wheel_heading_rad) *
                          ODOM_STRAFE_SCALE;

        body_forward_mm += wheel_forward_mm;
        body_strafe_mm += wheel_strafe_mm;

        switch(i)
        {
            case 0U:
                rotation_numerator_mm += -wheel_forward_mm +
                                          wheel_strafe_mm;
                break;

            case 1U:
                rotation_numerator_mm += wheel_forward_mm +
                                         wheel_strafe_mm;
                break;

            case 2U:
                rotation_numerator_mm += wheel_forward_mm -
                                         wheel_strafe_mm;
                break;

            default:
                rotation_numerator_mm += -wheel_forward_mm -
                                          wheel_strafe_mm;
                break;
        }
    }

    body_forward_mm *= 0.25f;
    body_strafe_mm *= 0.25f;

    wheel_yaw_delta_rad = rotation_numerator_mm /
                          (8.0f * ODOM_ROTATION_ARM_MM);

    /*
     * 舵轮运动学的正转方向与当前HWT9018 yaw正方向相反。
     */
    wheel_yaw_delta_deg = -wheel_yaw_delta_rad *
                           180.0f / ODOM_PI;

    g_fused_yaw_deg = Odom_WrapDeg(g_fused_yaw_deg +
                                    wheel_yaw_delta_deg);

    imu_yaw_deg = HWT9018_GetYawDeg();
    yaw_error_deg = Odom_WrapDeg(imu_yaw_deg -
                                 g_fused_yaw_deg);

    if(yaw_error_deg > ODOM_IMU_YAW_ERROR_LIMIT_DEG)
    {
        yaw_error_deg = ODOM_IMU_YAW_ERROR_LIMIT_DEG;
    }
    else if(yaw_error_deg < -ODOM_IMU_YAW_ERROR_LIMIT_DEG)
    {
        yaw_error_deg = -ODOM_IMU_YAW_ERROR_LIMIT_DEG;
    }

    g_fused_yaw_deg = Odom_WrapDeg(
        g_fused_yaw_deg +
        yaw_error_deg *
        ODOM_IMU_YAW_CORRECTION_PER_S *
        delta_time_s);

    yaw_deg = g_fused_yaw_deg;
    yaw_rad = -yaw_deg * ODOM_PI / 180.0f;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    g_pose.x_mm += (body_forward_mm * cos_yaw) -
                   (body_strafe_mm * sin_yaw);

    g_pose.y_mm += ((body_forward_mm * sin_yaw) +
                   (body_strafe_mm * cos_yaw)) *
                   ODOM_WORLD_Y_SCALE;

    g_pose.yaw_deg = yaw_deg;
}
OdometryPose Odometry_GetPose(void)
{
    return g_pose;
}
