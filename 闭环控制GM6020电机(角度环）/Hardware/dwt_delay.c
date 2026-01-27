#include "dwt_delay.h"
//ai提供
//dwt_delay.h
/* 系统时钟频率（72MHz） */
#define SYSTEM_CORE_CLOCK   72000000UL

/**
 * @brief  初始化DWT计数器
 * @note   必须在使用DWT功能前调用
 */
void DWT_Init(void)
{
    DEM_CR |= DEM_CR_TRCENA;    // 使能DWT外设
    DWT_CYCCNT = 0;              // 清零计数器
    DWT_CR |= DWT_CR_CYCCNTENA;  // 使能CYCCNT计数器
}

/**
 * @brief  获取当前DWT计数值
 * @return 当前CPU周期计数
 */
uint32_t DWT_GetTick(void)
{
    return DWT_CYCCNT;
}

/**
 * @brief  非阻塞微秒延时
 * @param  last_tick: 上次时间戳指针（需用户定义静态变量）
 * @param  delay_us: 延时微秒数
 * @return 1=时间到，0=时间未到
 */
uint8_t DWT_DelayNonBlock_us(uint32_t *last_tick, uint32_t delay_us)
{
    uint32_t current_tick = DWT_GetTick();
    uint32_t delay_ticks = delay_us * (SYSTEM_CORE_CLOCK / 1000000UL);
    
    if ((current_tick - *last_tick) >= delay_ticks)
    {
        *last_tick = current_tick;  // 更新时间戳
        return 1;  // 时间到
    }
    return 0;  // 时间未到
}

/**
 * @brief  非阻塞毫秒延时
 * @param  last_tick: 上次时间戳指针
 * @param  delay_ms: 延时毫秒数
 * @return 1=时间到，0=时间未到
 */
uint8_t DWT_DelayNonBlock_ms(uint32_t *last_tick, uint32_t delay_ms)
{
    return DWT_DelayNonBlock_us(last_tick, delay_ms * 1000);
}

/**
 * @brief  阻塞式微秒延时（精确）
 * @param  us: 延时微秒数
 */
void DWT_Delay_us(uint32_t us)
{
    uint32_t start_tick = DWT_GetTick();
    uint32_t delay_ticks = us * (SYSTEM_CORE_CLOCK / 1000000UL);
    
    while ((DWT_GetTick() - start_tick) < delay_ticks);
}

/**
 * @brief  阻塞式毫秒延时
 * @param  ms: 延时毫秒数
 */
void DWT_Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        DWT_Delay_us(1000);
    }
}

/**
 * @brief  初始化非阻塞定时器
 * @param  timer: 定时器结构体指针
 * @param  interval_us: 定时间隔（微秒）
 */
void DWT_Timer_Init(DWT_Timer_t *timer, uint32_t interval_us)
{
    timer->last_tick = DWT_GetTick();
    timer->interval_us = interval_us;
    timer->is_running = 1;
}

/**
 * @brief  检查定时器是否到时
 * @param  timer: 定时器结构体指针
 * @return 1=时间到（自动重置），0=时间未到
 */
uint8_t DWT_Timer_Check(DWT_Timer_t *timer)
{
    if (!timer->is_running) return 0;
    
    uint32_t current_tick = DWT_GetTick();
    uint32_t delay_ticks = timer->interval_us * (SYSTEM_CORE_CLOCK / 1000000UL);
    
    if ((current_tick - timer->last_tick) >= delay_ticks)
    {
        timer->last_tick = current_tick;
        return 1;
    }
    return 0;
}
