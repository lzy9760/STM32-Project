#ifndef __CHASSIS_H__
#define __CHASSIS_H__


#include "motor.h"


void chassis_remote_calculate(float vx, float vy, float vw);
void inferior_arc(void);
void chassis_drive_alignment_limit(void);
float chassis_get_wheel_heading_deg(uint8_t wheel_index);

extern float steering_max_error;
extern float steering_drive_scale;



#endif
