#include "stm32f10x.h"
// 补充u8/u16/u32类型定义
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

// 必须先包含Servo.h，才能识别宏和函数声明
#include "Servo.h"  
#include "Timer.h"
#include "Serial.h"
#include <stdlib.h> 
#include <string.h>
// 替换为你自己的OLED头文件
#include "OLED.h"  

//全局变量：当前舵机角度（初始90度）
u8 Servo_Angle = 90;  

int main(void)
{
    /************************** 外设初始化 **************************/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Serial_Init();     // 串口1 + DMA空闲中断（115200波特率）
    TIM3_Init();       // TIM3定时1秒中断（用于发送ONLINE）
    Servo_Init();      // 舵机PWM初始化（TIM2_CH1 PA0）
    OLED_Init();       // OLED初始化

    /************************** 初始显示 **************************/
    OLED_Clear();                      // 清屏
    OLED_ShowString(1, 1, "Angle: 90");// 初始角度显示
    OLED_ShowString(2, 1, "Count: 0"); // 初始接收次数显示

    /************************** 主循环 **************************/
    while(1)
    {
        // 任务1：每1秒向上位机发送"ONLINE" + VOFA+角度数据
        if(Timer1s_Flag == 1)
        {
            Serial_Printf("ONLINE\r\n");                // 发送心跳包
            Serial_Printf("Angle:%d\n", Servo_Angle);   // VOFA+波形数据
            Timer1s_Flag = 0;                           // 清空定时标志
        }

        // 任务2：处理串口接收的角度指令
        if(USART1_RX_Flag == 1)
        {
            // 1. 将接收的字符串转为角度
            Servo_Angle = atoi((char*)USART1_RX_BUF);
            
            // 2. 角度范围保护
            if(Servo_Angle > SERVO_MAX_ANGLE) Servo_Angle = SERVO_MIN_ANGLE;
          
            
            // 3. 控制舵机转到目标角度
            Servo_SetAngle(Servo_Angle);

            // 4. 更新OLED显示
            OLED_Clear();
            OLED_ShowString(1, 1, "Angle: ");        
            OLED_ShowNum(1, 7, Servo_Angle, 3);     
            OLED_ShowString(2, 1, "Count: ");        
            OLED_ShowNum(2, 8, USART1_RX_Count, 4); 

            // 5. 清空接收标志+缓冲区
            USART1_RX_Flag = 0;
            memset(USART1_RX_BUF, 0, USART1_RX_BUF_SIZE);
            USART1_RX_LEN = 0;
        }
    }
}
