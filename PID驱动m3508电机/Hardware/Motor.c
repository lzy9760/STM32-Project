//Motor.c
#include "Motor.h"

M3508_Info_t Motor1;

void M3508_Parse(uint8_t data[8])
{
    Motor1.angle       = (data[0] << 8) | data[1];
    Motor1.speed       = (data[2] << 8) | data[3];
    Motor1.current     = (data[4] << 8) | data[5];
    Motor1.temp = data[6];
}


int16_t OpenLoop_Current_From_Speed(int16_t target_rpm)
{
    /* ⚠️ 经验系数，后面可调 */
    float k = 0.5f;

    int16_t iq = (int16_t)(k * target_rpm);

    /* 限幅，保护电机 */
    if (iq > 8000)  iq = 8000;
    if (iq < -8000) iq = -8000;

    return iq;
}


