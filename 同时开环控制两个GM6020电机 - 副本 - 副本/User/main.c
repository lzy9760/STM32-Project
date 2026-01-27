#include "stm32f10x.h"
#include "Usart.h"
#include "Delay.h"
#include "MyCAN.h"

int main(void)
{
    Serial_Init();
    Delay_ms(100);
    GM6020_CAN_Init();
    Delay_ms(500);  // 增加等待时间，确保电机上电稳定
    
    printf("GM6020 Test Start!\r\n");
    printf("CAN MCR = 0x%08X\r\n", CAN1->MCR);
    printf("CAN MSR = 0x%08X\r\n", CAN1->MSR);

    static uint32_t print_cnt = 0;
    
    /* 设置目标电压 */
    GM6020_Set_Voltage(3, 8000);
    GM6020_Set_Voltage(7, 8000);

    while (1)
    {
        /* 统一发送所有电机控制帧 */
        GM6020_Send_All_Voltage();

        if (++print_cnt >= 500)  // 约500ms打印一次，AI提供思路，打印太频繁会阻塞程序
        {
            print_cnt = 0;

            printf("ID3: ang=%.1f, rpm=%d, cur=%.2fA, temp=%d\r\n",
                   gm6020_total_angle[3],
                   gm6020_status[3].speed_rpm,
                   gm6020_status[3].actual_current,
                   gm6020_status[3].temperature);

            printf("ID7: ang=%.1f, rpm=%d, cur=%.2fA, temp=%d\r\n\r\n",
                   gm6020_total_angle[7],
                   gm6020_status[7].speed_rpm,
                   gm6020_status[7].actual_current,
                   gm6020_status[7].temperature);
        }

        Delay_ms(1);  // 1ms周期，1kHz发送频率，Delay的时间要小一些，不然也会阻塞程序
    }
}

