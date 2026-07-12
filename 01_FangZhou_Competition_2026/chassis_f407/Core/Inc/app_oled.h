#ifndef APP_OLED_H
#define APP_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void AppOled_Init(void);
void AppOled_Clear(void);
void AppOled_Print(uint8_t x, uint8_t page, const char *text);
void AppOled_PrintLine(uint8_t page, const char *text);
void AppOled_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_OLED_H */
