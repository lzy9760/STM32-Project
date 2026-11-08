#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

typedef enum
{
    ENC_REJECT_NONE = 0,
    ENC_REJECT_SPI,
    ENC_REJECT_PARITY,
    ENC_REJECT_SENSOR_EF,
    ENC_REJECT_GLITCH
} enc_reject_reason_t;

extern volatile uint32_t enc_sb_count;
extern volatile uint32_t enc_raw; /* 最后通过所有检查的角度码，与 enc_deg 一致 */
extern volatile uint32_t enc_frame_raw; /* 最近收到的候选码，可能是坏数据 */
extern volatile uint32_t enc_glitch_count;
extern volatile uint32_t enc_spi_fail;
extern volatile uint32_t enc_parity_fail;
extern volatile uint32_t enc_zero_frame_count;
extern volatile uint32_t enc_zero_reject_count;
extern volatile uint16_t enc_last_frame;
extern volatile uint16_t enc_last_rejected_frame;
extern volatile enc_reject_reason_t enc_last_reject_reason;
extern volatile float enc_last_glitch_delta_deg;
extern volatile uint16_t enc_errfl;
extern volatile uint16_t enc_errfl_first;
extern volatile uint8_t enc_errfl_valid;
extern volatile uint16_t enc_diag_boot; /* 上电 DIAAGC，未修改传感器配置 */
extern volatile uint8_t enc_diag_boot_valid;
extern volatile uint8_t enc_diag_boot_ready;
extern volatile float enc_deg;
extern volatile uint8_t enc_valid; /* 本次更新通过 SPI/EF/奇偶/跳变检查 */
extern volatile uint32_t enc_valid_count; /* 只在接受新角度时递增 */


void encoder_666_update(void);
void encoder_init(void);



#endif
