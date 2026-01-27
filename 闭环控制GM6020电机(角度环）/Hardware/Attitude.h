#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f10x.h"

float Attitude_Calc_Roll_Accel(int16_t ax, int16_t ay, int16_t az);
float Attitude_Complementary_Filter(int16_t ax, int16_t ay, int16_t az, int16_t gx, float dt);

#endif
