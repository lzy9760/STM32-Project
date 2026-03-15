#ifndef TASK_H
#define TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement);
#endif
