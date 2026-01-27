#include "Attitude.h"
#include <math.h>

// 加速度计算Roll角
float Attitude_Calc_Roll_Accel(int16_t ax, int16_t ay, int16_t az)
{
    float fax = (float)ax;
    float fay = (float)ay;
    float faz = (float)az;
    // 防止除零
    if(faz == 0) faz = 0.001f;
    return atan2f(fay, faz) * 57.3f; // 弧度转角度
}

// 互补滤波融合加速度+陀螺仪
float Attitude_Complementary_Filter(int16_t ax, int16_t ay, int16_t az, int16_t gx, float dt)
{
    // 加速度计Roll角（静态稳定）
    float accel_roll = Attitude_Calc_Roll_Accel(ax, ay, az);
    // 陀螺仪Roll角速度（动态响应）：250dps量程下，灵敏度131 LSB/(dps)
    float gyro_roll_rate = (float)gx / 131.0f;

    // 互补滤波参数：0.96（陀螺仪）+0.04（加速度）
    static float filtered_roll = 0;
    filtered_roll = 0.96f * (filtered_roll + gyro_roll_rate * dt) + 0.04f * accel_roll;

    return filtered_roll;
}
