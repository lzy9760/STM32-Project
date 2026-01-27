#ifndef __MONITOR_H
#define __MONITOR_H

//Monitor.h

typedef struct
{
    float kp;
    float ki;
    float kd;

    float target_speed;
    float actual_speed;
    float pid_out;
} Monitor_t;

extern Monitor_t monitor;

#endif
