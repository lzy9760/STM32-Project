#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "at32f403a_407_board.h"



typedef struct
{
    uint8_t header;
    uint8_t stick_x1;
    uint8_t stick_y1;
    uint8_t stick_x2;
    uint8_t stick_y2;
    uint8_t knob;
    uint8_t enable;
    uint8_t open_loop;
    uint8_t footer;

    float fwd;
    float strafe;
    float rot;
    float knob_ratio;

    float vx;
    float vy;
    float vw;
} remote_t;

extern remote_t remote;

void remote_init(void);
void remote_parse(void);

#endif
