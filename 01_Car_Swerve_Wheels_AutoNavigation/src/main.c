#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "can.h"
#include "motor.h"
#include "pid.h"
#include "chassis.h"
#include "remote.h"
#include "hwt9018.h"
#include "navigation.h"
#include <math.h>

#define M_PI 3.14159265358979323846f


/* ========== 全局变量 ========== */
volatile uint8_t control_tick = 0;  
volatile uint32_t control_time_ms = 0U;
Motor motors[8];                   
static uint8_t navigation_auto_enabled = 1U;

static uint8_t Motor_Feedback_AllReady(void)
{
    uint8_t i;

    for(i = 0U; i < 8U; i++)
    {
        if(motors[i].feedback_valid == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

/* ========== 定时器初始化 (1kHz) ========== */
static void Timer1_Init(void)
{
    /* TMR1 时钟 = APB2 x 2 = 240MHz */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

    /* 240MHz / 240 / 1000 = 1kHz */
    tmr_base_init(TMR1, 999, 239);         // (周期, 预分频) ← 注意顺序！
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);

    /* 使能溢出中断 */
    tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0, 0);

    /* 启动定时器 */
    tmr_counter_enable(TMR1, TRUE);
}

/* ========== 主函数 ========== */
int main(void)
{
    /* 系统时钟: HEXT 8MHz → PLL → 240MHz */
    system_clock_config();

    /* 板级初始化 (delay_ms 等) */
    at32_board_init();

    /* CAN 初始化 (CAN1 + CAN2, 1Mbps) */
    CAN_Init();

    /* 1kHz 定时器 */
    Timer1_Init();

    /* ---- 初始化 4 个 GM6020 转向电机 (CAN1) ---- */
    Motor_Init(&motors[0], MOTOR_TYPE_GM6020, CAN1, MOTOR_1);
    Motor_Init(&motors[1], MOTOR_TYPE_GM6020, CAN1, MOTOR_3);
    Motor_Init(&motors[2], MOTOR_TYPE_GM6020, CAN2, MOTOR_2);
    Motor_Init(&motors[3], MOTOR_TYPE_GM6020, CAN2, MOTOR_4);

    /* ---- 初始化 4 个 M3508 驱动电机 (CAN2) ---- */
    Motor_Init(&motors[4], MOTOR_TYPE_M3508, CAN1, MOTOR_1);
    Motor_Init(&motors[5], MOTOR_TYPE_M3508, CAN1, MOTOR_3);
    Motor_Init(&motors[6], MOTOR_TYPE_M3508, CAN2, MOTOR_2);
    Motor_Init(&motors[7], MOTOR_TYPE_M3508, CAN2, MOTOR_4);

    pid_all_init();
    remote_init();
    HWT9018_Init();
    Navigation_Init();

    /* ---- 主循环 ---- */
    while (1)
    {
        can_rx_message_type rx_msg;

        while(can_receive_message_pending_get(CAN1, CAN_RX_FIFO0) > 0)
            {
                can_message_receive(CAN1, CAN_RX_FIFO0, &rx_msg);
                Motor_CAN_Nb_Receive(CAN1, &rx_msg);
            }


            while(can_receive_message_pending_get(CAN2, CAN_RX_FIFO0) > 0)
            {
                can_message_receive(CAN2, CAN_RX_FIFO0, &rx_msg);
                Motor_CAN_Nb_Receive(CAN2, &rx_msg);
            }

        remote_parse();

        if (control_tick)
        {
            float vx;
            float vy;
            float vw;
            int16_t g0, g1, g2, g3;
            int16_t k4, k5, k6, k7;

            vx = remote.vx;
            vy = remote.vy;
            vw = remote.vw;

            if((remote.fwd != 0.0f) ||
               (remote.strafe != 0.0f) ||
               (remote.rot != 0.0f))
            {
                navigation_auto_enabled = 0U;
            }

            if(navigation_auto_enabled != 0U)
            {
                Navigation_Task(control_time_ms);
                (void)Navigation_GetMotion(&vx, &vy, &vw);
            }


            if(Motor_Feedback_AllReady() != 0U)
            {
                chassis_remote_calculate(vx, vy, vw);
                inferior_arc();
                chassis_drive_alignment_limit();

                g0 = GM6020_cascade_control(&motors[0], motors[0].target);
                g1 = GM6020_cascade_control(&motors[1], motors[1].target);
                g2 = GM6020_cascade_control(&motors[2], motors[2].target);
                g3 = GM6020_cascade_control(&motors[3], motors[3].target);

                k4 = (int16_t)pid_speed_calculate(&motors[4].pid_speed, motors[4].target, motors[4].rpm);
                k5 = (int16_t)pid_speed_calculate(&motors[5].pid_speed, motors[5].target, motors[5].rpm);
                k6 = (int16_t)pid_speed_calculate(&motors[6].pid_speed, motors[6].target, motors[6].rpm);
                k7 = (int16_t)pid_speed_calculate(&motors[7].pid_speed, motors[7].target, motors[7].rpm);
            }
            else
            {
                g0 = 0;
                g1 = 0;
                g2 = 0;
                g3 = 0;
                k4 = 0;
                k5 = 0;
                k6 = 0;
                k7 = 0;
            }

            control_tick = 0;



            
            /* 给四个GM6020输出3000电压 */
            //CAN_Nb_Send(CAN1, CAN_CONTROL_ID_GM6020, 3000, 0, 3000, 0);
            //CAN_Nb_Send(CAN1, CAN_CONTROL_ID_M3508, 2000, 0, 2000, 0);

            /* 给四个M3508输出2000电流 */
            //CAN_Nb_Send(CAN2, CAN_CONTROL_ID_M3508, 0, 2000, 0, 2000);
            //CAN_Nb_Send(CAN2, CAN_CONTROL_ID_GM6020, 0, 3000, 0, 3000);
            
            CAN_Nb_Send(CAN1, CAN_CONTROL_ID_GM6020, g0, 0, g1, 0);
            CAN_Nb_Send(CAN1, CAN_CONTROL_ID_M3508, k4, 0, k5, 0);

            CAN_Nb_Send(CAN2, CAN_CONTROL_ID_GM6020, 0, g2, 0, g3);
            CAN_Nb_Send(CAN2, CAN_CONTROL_ID_M3508, 0, k6, 0, k7);
        }
    }

}

/* ========== TMR1 中断处理 ========== */
void TMR1_OVF_TMR10_IRQHandler(void)
{
    if (tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
    {
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
        control_time_ms++;
        control_tick = 1;
    }
}
