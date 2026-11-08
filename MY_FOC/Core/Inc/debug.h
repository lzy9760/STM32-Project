#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>

#define DEBUG_CHANNEL_COUNT 10U
/* VOFA+ 的 Delta-t = 1000 / 此值 (ms)。采样来自已完成的电流环，有调度抖动。 */
#define DEBUG_SAMPLE_RATE_HZ 5000U

/* 初始化一次；Debug_Task 放在主循环中，不在中断中发送 USB。 */
void Debug_Init(void);
void Debug_Task(void);

/* 原电流环入口/签名保持不变；通道表在 debug.c 的 Debug_Capture 内。 */
void Debug_Capture(float id, float iq);

/* 保留原四通道接口兼容现有代码：现在发送固定十通道 JustFloat，后六路填零。
 * 名称参数不再发送；在 VOFA+ 内设置名称。不要与 Debug_Capture 混用。
 * 参数只求值一次，转为 float；整数超过 2^24 时可能丢失精度。 */
#define DEBUG_PLOT4(ch1, ch2, ch3, ch4) \
    Debug_Plot4((float)(ch1), (float)(ch2), (float)(ch3), (float)(ch4), \
                #ch1, #ch2, #ch3, #ch4)

void Debug_Plot4(float ch1, float ch2, float ch3, float ch4,
                 const char *name1, const char *name2, const char *name3, const char *name4);

extern volatile uint32_t debug_dropped;   /* USB 缓冲满，丢弃的帧数 */
extern volatile uint32_t debug_sent;      /* 已提交 USB 的帧数 */
extern volatile uint32_t debug_error;     /* USB 初始化失败 */
extern volatile uint32_t debug_skipped;   /* 控制调用延迟，跳过的采样时隙 */
extern volatile uint32_t debug_invalid;   /* NaN/Inf 导致丢弃的帧数 */
extern volatile uint32_t debug_samples;   /* 已采集有效帧数 */
extern volatile float debug_sample_hz;    /* 每秒更新的实际有效采集率 */

#endif
