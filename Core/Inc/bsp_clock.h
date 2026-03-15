#ifndef BSP_CLOCK_H
#define BSP_CLOCK_H

#include <stdint.h>

void BSP_Clock_Init(void);
uint32_t BSP_Clock_Millis(void);
float BSP_Clock_ElapsedSeconds(uint32_t *last_ms);

#endif /* BSP_CLOCK_H */
