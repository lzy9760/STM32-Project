#ifndef __DWT_DELAY_H
#define __DWT_DELAY_H
//dwt_delay.h
//ai提供
#include "stm32f10x.h"

/* DWT寄存器定义（Cortex-M3内核） */
#define DWT_CR      *(volatile uint32_t *)0xE0001000
#define DWT_CYCCNT  *(volatile uint32_t *)0xE0001004
#define DEM_CR      *(volatile uint32_t *)0xE000EDFC

#define DEM_CR_TRCENA   (1 << 24)
#define DWT_CR_CYCCNTENA (1 << 0)

/* 函数声明 */
void DWT_Init(void);
uint32_t DWT_GetTick(void);
uint8_t DWT_DelayNonBlock_us(uint32_t *last_tick, uint32_t delay_us);
uint8_t DWT_DelayNonBlock_ms(uint32_t *last_tick, uint32_t delay_ms);
void DWT_Delay_us(uint32_t us);
void DWT_Delay_ms(uint32_t ms);

/* 非阻塞定时器结构体 */
typedef struct {
    uint32_t last_tick;     // 上次记录的时刻
    uint32_t interval_us;   // 间隔时间（微秒）
    uint8_t  is_running;    // 是否已启动
} DWT_Timer_t;

void DWT_Timer_Init(DWT_Timer_t *timer, uint32_t interval_us);
uint8_t DWT_Timer_Check(DWT_Timer_t *timer);  // 返回1表示时间到

#endif
