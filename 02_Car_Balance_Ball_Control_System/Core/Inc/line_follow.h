#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>
#include "line_sensor.h"

typedef struct
{
  int16_t left_target;
  int16_t right_target;
  int16_t turn;
} LineFollowOutput_t;

void LineFollow_Reset(void);
LineFollowOutput_t LineFollow_Step(const LineSensorFrame_t *frame,
                                   int16_t base_speed);

#endif
