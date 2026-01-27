#include "stm32f10x.h"
#include "Usart.h"
#include "Delay.h"
#include "MyCAN.h"
//main.c
//extern uint16_t flag;
int main(void)
{
    Serial_Init();          // 串口初始化（115200波特率）
    Delay_ms(100);          // 等待串口稳定
    GM6020_CAN_Init();      // CAN初始化（1Mbps）
    Delay_ms(300);          // 等待电机上电稳定（手册要求上电后需稳定时间）
    printf("GM6020 Test Start! Motor ID=%d\r\n", GM6020_MOTOR_ID);
    printf("CAN MCR = 0x%08X\r\n", CAN1->MCR);//调试方法，看CAN有没有起作用，ai提供
    printf("CAN MSR = 0x%08X\r\n", CAN1->MSR);

    while (1) {
        // 发送电压指令（6000对应约0.72V，安全小电压，电机缓慢转动）
        GM6020_Send_Voltage(10000);
        Delay_ms(10);  // 发送频率100Hz（手册建议不低于10Hz
        // 打印反馈数据（200ms一次，避免串口拥堵）
        if (gm6020_status.is_connected) {
					//static uint32_t cnt = 0;
  //cnt++;
//if (cnt >=500) {   // 1000ms
//    cnt = 0;
            printf("Feedback: Angle=%d(%.1f°), Speed=%d rpm, Current=%.2fA ,Temp=%d℃\r\n",
                   gm6020_status.mechanical_angle,
                   gm6020_total_angle,
                   gm6020_status.speed_rpm,
                   gm6020_status.actual_current,
                   gm6020_status.temperature);
        } 
			
				
				else {
            printf("Waiting for motor feedback...\r\n");
        }

        Delay_ms(100);//罪魁祸首
    }
}
