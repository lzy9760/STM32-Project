#include "stm32f10x.h"
#include "Usart.h"
#include "MyCAN.h"
#include "pid.h"
#include "GM6020.h"
#include "dwt_delay.h"
//main.c
int main(void)
{
    /* 系统初始化 */
    DWT_Init();
    Serial_Init();
    DWT_Delay_ms(100);
    GM6020_CAN_Init();
    DWT_Delay_ms(500);
    Motor_Control_Init();
    
    printf("\r\n=== GM6020 Cascade PID Angle Control ===\r\n");

    /* 非阻塞定时器 */
    static uint32_t tick_1ms = 0;
    static uint32_t tick_check = 0;

    while (1)
    {
        /* 1ms周期：电机控制 */
        if (DWT_DelayNonBlock_ms(&tick_1ms, 1))
        {
            Motor_Control_1ms();
        }
        
        /* 2s周期：状态检测，ai提供 */
        if (DWT_DelayNonBlock_ms(&tick_check, 2000))
        {
            if (!gm6020_status.is_connected)
            {
                printf("[WARN] Motor disconnected!\r\n");
            }
            else
            {
                printf("[INFO] Temp=%d C, Current=%.2fA\r\n",
                       gm6020_status.temperature,
                       gm6020_status.actual_current);
            }
        }
    }
}
