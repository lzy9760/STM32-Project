#include "chassis.h"
#include "pid.h"
#include "motor.h"
#include <math.h>

#define CHASSIS_RADIUS 0.39652f
#define REDUCTION_RATIO 19.0f
#define WHEEL_C  0.29752f   // m
#define STEERING_DRIVE_FULL_ERROR  5.0f
#define STEERING_DRIVE_STOP_ERROR 15.0f
#define INFERIOR_ARC_MIN_DRIVE_SPEED 1.0f
#define STEERING_FILTER_MAX_SPEED 1440.0f
#define STEERING_FILTER_MAX_ACCELERATION 15000.0f
#define STEERING_FILTER_CONTROL_PERIOD 0.001f
#define STEERING_FILTER_DEADBAND 0.3f
#define M_PI 3.14159265358979323846f   // π

float a = CHASSIS_RADIUS * 0.7071f;
float VX[4];
float VY[4];
float steering_angle[4];
float wheel_speed[4];
float error[4];
float steering_max_error;
float steering_drive_scale;
static float steering_filtered_target[4];
static float steering_filtered_speed[4];
static uint8_t steering_filter_initialized[4];


static const float steering_zero_offset[4] = {0.0f, 0.0f, 0.0f, 118.0f};

static float angle_wrap_360(float angle)
{
      while(angle >= 360.0f)
            angle -= 360.0f;
      while(angle < 0.0f)
            angle += 360.0f;
      return angle;
}

static float angle_wrap_180(float angle)
{
      while(angle > 180.0f)
            angle -= 360.0f;
      while(angle < -180.0f)
            angle += 360.0f;
      return angle;
}

float chassis_get_wheel_heading_deg(uint8_t wheel_index)
{
      float raw_angle;

      if(wheel_index >= 4U)
      {
            return 0.0f;
      }

      raw_angle = motors[wheel_index].angle / 8192.0f * 360.0f;
      return angle_wrap_180(raw_angle - steering_zero_offset[wheel_index]);
}

static float steering_target_filter(uint8_t index, float raw_target)
{
      float delta;
      float abs_delta;
      float desired_speed;
      float stop_speed;
      float speed_delta;
      float max_speed_delta;
      float target_step;

      if(steering_filter_initialized[index] == 0U)
      {
            steering_filtered_target[index] = raw_target;
            steering_filtered_speed[index] = 0.0f;
            steering_filter_initialized[index] = 1U;
            return raw_target;
      }

      delta = angle_wrap_180(raw_target - steering_filtered_target[index]);
      abs_delta = fabsf(delta);

      if(abs_delta < STEERING_FILTER_DEADBAND)
      {
            steering_filtered_speed[index] = 0.0f;
            return steering_filtered_target[index];
      }

      stop_speed = sqrtf(2.0f * STEERING_FILTER_MAX_ACCELERATION * abs_delta);
      if(stop_speed > STEERING_FILTER_MAX_SPEED)
      {
            stop_speed = STEERING_FILTER_MAX_SPEED;
      }

      desired_speed = (delta > 0.0f) ? stop_speed : -stop_speed;
      speed_delta = desired_speed - steering_filtered_speed[index];
      max_speed_delta = STEERING_FILTER_MAX_ACCELERATION *
                        STEERING_FILTER_CONTROL_PERIOD;

      if(speed_delta > max_speed_delta)
      {
            speed_delta = max_speed_delta;
      }
      else if(speed_delta < -max_speed_delta)
      {
            speed_delta = -max_speed_delta;
      }

      steering_filtered_speed[index] += speed_delta;
      target_step = steering_filtered_speed[index] *
                    STEERING_FILTER_CONTROL_PERIOD;

      if(((delta > 0.0f) && (target_step >= delta)) ||
         ((delta < 0.0f) && (target_step <= delta)))
      {
            steering_filtered_target[index] = raw_target;
            steering_filtered_speed[index] = 0.0f;
      }
      else
      {
            steering_filtered_target[index] = angle_wrap_360(
                steering_filtered_target[index] + target_step);
      }

      return steering_filtered_target[index];
}


void chassis_remote_calculate(float vx, float vy, float vw)
{
      /*右前轮*/
      VX[0] = vx - vw * a;
      VY[0] = vy + vw * a;
      steering_angle[0] = atan2(VY[0], VX[0]) * 180 / M_PI;
      motors[0].target = steering_target_filter(
          0U, angle_wrap_360(steering_angle[0] + steering_zero_offset[0]));

      wheel_speed[0] = sqrt(VX[0] * VX[0] + VY[0] * VY[0]);
      motors[4].target = -wheel_speed[0] * 60 * REDUCTION_RATIO / WHEEL_C;
      /*左前轮*/
      VX[1] = vx + vw * a;
      VY[1] = vy + vw * a;
      steering_angle[1] = atan2(VY[1], VX[1]) * 180 / M_PI;
      motors[1].target = steering_target_filter(
          1U, angle_wrap_360(steering_angle[1] + steering_zero_offset[1]));

      wheel_speed[1] = sqrt(VX[1] * VX[1] + VY[1] * VY[1]);
      motors[5].target = wheel_speed[1] * 60 * REDUCTION_RATIO / WHEEL_C;
      /*左后轮*/
      VX[2] = vx + vw * a;
      VY[2] = vy - vw * a;
      steering_angle[2] = atan2(VY[2], VX[2]) * 180 / M_PI;
      motors[2].target = steering_target_filter(
          2U, angle_wrap_360(steering_angle[2] + steering_zero_offset[2]));

      wheel_speed[2] = sqrt(VX[2] * VX[2] + VY[2] * VY[2]);
      motors[6].target = wheel_speed[2] * 60 * REDUCTION_RATIO / WHEEL_C;
      /*右后轮*/
      VX[3] = vx - vw * a;
      VY[3] = vy - vw * a;
      steering_angle[3] = atan2(VY[3], VX[3]) * 180 / M_PI;
      motors[3].target = steering_target_filter(
          3U, angle_wrap_360(steering_angle[3] + steering_zero_offset[3]));

      wheel_speed[3] = -sqrt(VX[3] * VX[3] + VY[3] * VY[3]);
      motors[7].target = wheel_speed[3] * 60 * REDUCTION_RATIO / WHEEL_C;

      

}

void inferior_arc(void)
{
     int i;
     for(i = 0; i < 4; i++)
     {
        float actual_angle = motors[i].angle / 8192.0f * 360.0f;

        error[i] = angle_wrap_180(motors[i].target - actual_angle);

        if(fabs(motors[i+4].target) <= INFERIOR_ARC_MIN_DRIVE_SPEED)
        {
            continue;
        }

        if(fabs(error[i]) > 90.0f)
        {
            motors[i].target = angle_wrap_360(motors[i].target + 180.0f);
            motors[i+4].target = -motors[i+4].target;
        }
     }
}

void chassis_drive_alignment_limit(void)
{
     int i;

     steering_max_error = 0.0f;

     for(i = 0; i < 4; i++)
     {
        float actual_angle = motors[i].angle / 8192.0f * 360.0f;
        float final_error = fabs(angle_wrap_180(motors[i].target - actual_angle));

        if(final_error > steering_max_error)
        {
            steering_max_error = final_error;
        }
     }

     if(steering_max_error >= STEERING_DRIVE_STOP_ERROR)
     {
        steering_drive_scale = 0.0f;
     }
     else if(steering_max_error <= STEERING_DRIVE_FULL_ERROR)
     {
        steering_drive_scale = 1.0f;
     }
     else
     {
        steering_drive_scale =
            (STEERING_DRIVE_STOP_ERROR - steering_max_error) /
            (STEERING_DRIVE_STOP_ERROR - STEERING_DRIVE_FULL_ERROR);
     }

     for(i = 4; i < 8; i++)
     {
        motors[i].target *= steering_drive_scale;
     }
}

