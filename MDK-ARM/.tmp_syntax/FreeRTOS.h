#ifndef FREERTOS_H
#define FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()
#endif
