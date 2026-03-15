#ifndef CMSIS_OS_H
#define CMSIS_OS_H
#include <stdint.h>
typedef void *osThreadId_t;
typedef enum
{
  osPriorityRealtime = 0,
  osPriorityAboveNormal = 0,
  osPriorityBelowNormal = 0
} osPriority_t;
typedef struct
{
  const char *name;
  uint32_t stack_size;
  osPriority_t priority;
} osThreadAttr_t;
osThreadId_t osThreadNew(void (*func)(void *), void *argument, const osThreadAttr_t *attr);
#endif
