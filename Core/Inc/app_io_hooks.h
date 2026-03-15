#ifndef APP_IO_HOOKS_H
#define APP_IO_HOOKS_H

#include <stdint.h>

void APP_RemoteRxFrameHook(const uint8_t *frame, uint16_t len);
void APP_RemoteRxByteHook(uint8_t byte);
void APP_VisionRxBytesHook(const uint8_t *data, uint16_t len);
void APP_VisionTxBytesHook(const uint8_t *data, uint16_t len);

#endif /* APP_IO_HOOKS_H */

