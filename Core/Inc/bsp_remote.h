#ifndef BSP_REMOTE_H
#define BSP_REMOTE_H

#include <stdint.h>

#define BSP_REMOTE_FRAME_LEN 18U

typedef struct
{
  int16_t ch_raw[4];//原始通道值（-32768到32767）
  float ch_norm[4];//归一化通道值（-1.0到1.0）
  int16_t wheel;//轮值（-32768到32767）
  int16_t mouse_x;//鼠标X轴值（-32768到32767）
  int16_t mouse_y;//鼠标Y轴值（-32768到32767）
  int16_t mouse_z;//鼠标Z轴值（-32768到32767）
  uint8_t mouse_l;//鼠标左键状态（0或1）
  uint8_t mouse_r;//鼠标右键状态（0或1）
  uint16_t key_code;//按键码（0到65535）
  uint8_t sw_left;//左开关状态（0到3）
  uint8_t sw_right;//右开关状态（0到3）
  uint8_t online;//是否在线（0或1）
  uint32_t last_update_ms;//最后更新时间（毫秒）
} BSP_RemoteState_t;

void BSP_Remote_Init(void);
void BSP_Remote_DecodeFrame(const uint8_t *frame, uint16_t len);
void BSP_Remote_FeedByte(uint8_t byte);
void BSP_Remote_GetState(BSP_RemoteState_t *out_state);
uint8_t BSP_Remote_IsOnline(void);
void BSP_Remote_SetMockChannels(float ch0,
                                float ch1,
                                float ch2,
                                float ch3,
                                uint8_t sw_left,
                                uint8_t sw_right);

#endif /* BSP_REMOTE_H */
