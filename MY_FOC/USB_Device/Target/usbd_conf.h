#ifndef USBD_CONF_H
#define USBD_CONF_H
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stddef.h>
#define USE_USB_FS
#define USBD_MAX_NUM_INTERFACES 2U
#define USBD_MAX_NUM_CONFIGURATION 1U
#define USBD_MAX_STR_DESC_SIZ 256U
#define USBD_SELF_POWERED 1U
#define USBD_DEBUG_LEVEL 0U
#define USBD_LPM_ENABLED 0U
void *Debug_Alloc(uint32_t size);
void Debug_Free(void *pointer);
#define USBD_malloc Debug_Alloc
#define USBD_free Debug_Free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)
#endif
