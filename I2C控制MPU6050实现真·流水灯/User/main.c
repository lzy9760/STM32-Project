#include "stm32f10x.h"
#include "LED.h"
#include "MPU6050.h"
#include "Attitude.h"
#include "Delay.h"
#include "MyI2C.h"

int main(void)
{
    int16_t ax, ay, az;      // 加速度数据
    int16_t gx, gy, gz;      // 陀螺仪数据
    float roll;              // 滤波后的Roll角
    int led_index = 0;       // 当前点亮的LED索引
    int direction = 1;       // 流水灯方向（1：右，-1：左）
    uint8_t breath_brightness = 0;  // 全局呼吸亮度（0-255，所有LED同步）
    int8_t breath_dir = 1;   // 呼吸方向（1：变亮，-1：变暗）
    uint32_t last_time = 0;  // 时间戳，计算dt

    // 初始化系统
    //Delay_Init(72);          // 72MHz系统时钟
    LED_Init();              // LED初始化（去掉PWM）
    I2C_Init_Config();       // I2C初始化
    MPU6050_Init();          // MPU6050初始化

    last_time = SysTick->VAL; // 初始时间戳

    while(1)
    {
        // 1. 计算时间间隔dt（秒）
        uint32_t now = SysTick->VAL;
        float dt = (last_time - now) / 72000000.0f; // 72MHz系统时钟
        last_time = now;

        // 2. 读取MPU6050数据
        MPU6050_Read_Accel(&ax, &ay, &az);
        MPU6050_Read_Gyro(&gx, &gy, &gz);

        // 3. 互补滤波解算Roll角
        roll = Attitude_Complementary_Filter(ax, ay, az, gx, dt);

        // 4. 根据Roll角控制流水灯方向
        if(roll > 15)
            direction = 1;   // 右倾：向右流水
        else if(roll < -15)
            direction = -1;  // 左倾：向左流水

        // 5. 更新全局呼吸亮度（所有LED同步）
        breath_brightness += breath_dir;
        if(breath_brightness >= 255) breath_dir = -1;
        if(breath_brightness <= 0) breath_dir = 1;

        // 6. 流水灯+呼吸效果：仅点亮当前LED，同步呼吸
        LED_All_Off(); // 先灭所有LED
        // 核心：当前LED按全局呼吸亮度渐变
        LED_Breath_Control(led_index, breath_brightness);

        // 7. 更新流水灯索引（控制流水速度）
        static uint16_t flow_cnt = 0;
        if(++flow_cnt >= 50) // 流水速度：数值越大，流水越慢
        {
            flow_cnt = 0;
            led_index += direction;
            if(led_index >= LED_NUM) led_index = 0;
            if(led_index < 0) led_index = LED_NUM - 1;
        }
    }
}

