#ifndef VISION_COMM_H
#define VISION_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <stdint.h>

#define VISION_COMM_LIST_SIZE       96U
#define VISION_COMM_FIELD_SIZE      32U

#define VISION_COLOR_NO_CAMERA      (-1)
#define VISION_COLOR_NONE           0

typedef struct
{
  uint32_t frame_no;
  uint16_t elapsed_ms;
  uint8_t direct_hits;
  uint8_t fenge_hits;
  uint8_t total_codes;
  char qr_list[VISION_COMM_LIST_SIZE];
  char qr_tl[VISION_COMM_FIELD_SIZE];
  char qr_tr[VISION_COMM_FIELD_SIZE];
  char qr_bl[VISION_COMM_FIELD_SIZE];
  char qr_br[VISION_COMM_FIELD_SIZE];
  uint8_t valid;
} VisionResult_t;

typedef struct
{
  uint32_t frame_no;
  int8_t color1_id;
  int8_t color2_id;
  int8_t approach;
  int8_t dir_x;
  int8_t dir_y;
  uint8_t valid;
} VisionColorResult_t;

typedef struct
{
  uint32_t byte_count;
  uint32_t line_count;
  uint32_t frame_count;
  uint32_t parse_error_count;
  uint32_t overflow_count;
  uint32_t uart_error_count;
  uint32_t last_rx_tick;
} VisionCommStats_t;

HAL_StatusTypeDef VisionComm_Init(UART_HandleTypeDef *huart);
void VisionComm_OnRxCplt(UART_HandleTypeDef *huart);
void VisionComm_OnError(UART_HandleTypeDef *huart);
uint8_t VisionComm_GetLatest(VisionResult_t *result);
uint8_t VisionComm_GetLatestColor(VisionColorResult_t *result);
void VisionComm_GetStats(VisionCommStats_t *stats);
void VisionComm_FormatQrText(const VisionResult_t *result, char *out, size_t out_size);
uint8_t VisionComm_FormatQrColors(const VisionResult_t *result, char *out, size_t out_size);
void VisionComm_FormatQuadrantText(const VisionResult_t *result, char *out, size_t out_size);
void VisionComm_FormatColorText(const VisionColorResult_t *result, char *out, size_t out_size);
void VisionComm_FormatColorStateText(const VisionColorResult_t *result, char *out, size_t out_size);
void VisionComm_FormatDirectionText(const VisionColorResult_t *result, char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* VISION_COMM_H */
