#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Servo.h"
#include "Encoder.h"

#define ENCODER_PPR  1000    // 一圈脉冲数（按你实际编码器改）
#define MAX_ANGLE    90

int main(void)
{
    OLED_Init();
    Servo_Init();
    Encoder_Init();

    Encoder_Clear();   // ★非常关键：设置当前位置为 0°

    OLED_ShowString(1, 1, "Angle:");

    while (1)
    {
        int16_t cnt = Encoder_GetCount();   // 有符号

        // 编码器计数 → 角度
        float angle = (float)cnt * 180.0f / ENCODER_PPR;

        // 限幅
        if (angle >  MAX_ANGLE) angle =  MAX_ANGLE;
        if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;

        // 映射到舵机（0~180）
        Servo_SetAngle(angle + 90);

        OLED_ShowSignedNum(1, 7, (int)angle, 3);

        Delay_ms(20);
    }
}




//int main(void)
//{
//    Servo_Init();

//    while (1)
//    {
//        Servo_SetAngle(0);    // 最左
//        Delay_ms(1000);

//        Servo_SetAngle(90);   // 中间
//        Delay_ms(1000);

//        Servo_SetAngle(180);  // 最右
//        Delay_ms(1000);
//    }
//}

