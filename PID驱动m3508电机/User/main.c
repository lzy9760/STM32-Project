#include "stm32f10x.h"
#include "MyCAN.h"
#include "Motor.h"
#include "Pid.h"
#include "Monitor.h"
#include "Delay.h"

//main.c
Monitor_t monitor;
PID_t speed_pid;


volatile uint8_t ctrl_flag = 0;

void SysTick_Handler(void)//中断服务函数，按照系统时钟设定的周期，每隔一段时间进入一次函数，这里每过1ms触发一次
{
    ctrl_flag = 1;
}

int main(void)
{
    CAN1_Init();
    
    SysTick_Config(SystemCoreClock / 1000);//SystemCoreClock / 1000 = 72000（把 1 秒的时钟次数分成 1000 份，每份就是 1ms 的时钟次数）

    /* ===== 初始化 monitor ===== */
    monitor.kp = 8.0f;
    monitor.ki = 0.2f;
    monitor.kd = 0.0f;

    monitor.target_speed = 500;

    /* ===== 初始化 PID ===== */
    PID_Init(&speed_pid,// kp
             monitor.kp,// ki
             monitor.ki,// kd
             monitor.kd,// 最大输出 iq
             8000,      // 积分限幅
             3000); 

//    int16_t target_speed = 1000; // 目标转速 rpm
//    int16_t iq;


//    while (1)
//    {
//        if (ctrl_flag)
//        {
//            ctrl_flag = 0;

//           iq = OpenLoop_Current_From_Speed(target_speed);

//            CAN1_Send_Current(iq, 0, 0, 0);

//        }
//			
//		}

//    }


   while (1)
{
    /* ========= 1. 控制周期（1ms） ========= */
    if (ctrl_flag)
    {
        ctrl_flag = 0;

        /* 从监控变量读目标 */
        monitor.actual_speed = Motor1.speed;

        /* 在线更新 PID 参数 */
        speed_pid.kp = monitor.kp;
        speed_pid.ki = monitor.ki;
        speed_pid.kd = monitor.kd;

        /* PID 计算 */
        monitor.pid_out = PID_Calc(&speed_pid,
                                   monitor.actual_speed,
                                   monitor.target_speed);

        /* 输出到电机 */
        CAN1_Send_Current((int16_t)monitor.pid_out, 0, 0, 0);
    }

//    /* ========= 2. 上位机数据发送（10ms） ========= */
//    static uint8_t send_cnt = 0;
//    if (++send_cnt >= 10)
//			
//    {
//        send_cnt = 0;
//        USART1_SendMonitor();
//    }

    
}



}
