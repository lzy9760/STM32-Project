#include "stm32f10x.h"
#include <stdio.h>
#include "GM6020.h"
#include "pid.h"
#include "MyCAN.h"
#include "math.h"
#include "dwt_delay.h"
//GM60202.c
/* 双环PID */
PID_t angle_pid;    // 外环：角度环
PID_t speed_pid;    // 内环：速度环

/* 控制变量*/
ControlMode_e control_mode = CONTROL_MODE_ANGLE;
float target_angle = 0;         // 目标角度（度）
float target_speed = 0;         // 目标速度（rpm）

/*方波参数ai提供*/
#define SQUARE_WAVE_PERIOD_MS   4000    // 方波周期4秒
#define SQUARE_WAVE_HIGH        180.0f  // 方波高电平角度（度）
#define SQUARE_WAVE_LOW         0.0f    // 方波低电平角度（度）

static uint32_t wave_timer = 0;
static uint16_t print_cnt = 0;

//电机初始化
void Motor_Control_Init(void)
{
    
    PID_Init(&angle_pid,
             10.0f,      // Kp：角度误差1度 → 输出15rpm
             1.0f,       // Ki：角度环一般不用积分（内环会消除稳态误差）
             0.5f,       // Kd：抑制超调
             300.0f,     // 输出限幅：最大目标速度300rpm
             100.0f);    // 积分限幅

    /* 内环：速度PID
     * 输出为电压值，限幅为电机最大电压
     */
    PID_Init(&speed_pid,
             20.0f,      // Kp：速度误差1rpm → 输出80电压
             0.0f,       // Ki：消除速度稳态误差
             0.5f,       // Kd：速度环一般不用微分
             20000.0f,   // 输出限幅：最大电压20000
             8000.0f);   // 积分限幅

    printf("=== Cascade PID Initialized ===\r\n");
    printf("Angle PID: Kp=%.1f, Ki=%.2f, Kd=%.2f\r\n", 
           angle_pid.kp, angle_pid.ki, angle_pid.kd);
    printf("Speed PID: Kp=%.1f, Ki=%.2f, Kd=%.2f\r\n", 
           speed_pid.kp, speed_pid.ki, speed_pid.kd);
    printf("Square Wave: %.1f° <-> %.1f°, Period=%dms\r\n",
           SQUARE_WAVE_LOW, SQUARE_WAVE_HIGH, SQUARE_WAVE_PERIOD_MS);
}

//设置目标角度
void Motor_SetTargetAngle(float angle_deg)
{
    target_angle = angle_deg;
}


//设置目标速度
void Motor_SetTargetSpeed(float speed_rpm)
{
    target_speed = speed_rpm;
}


//设置控制模式
void Motor_SetMode(ControlMode_e mode)
{
    control_mode = mode;
    PID_Reset(&angle_pid);
    PID_Reset(&speed_pid);
}


//生成方波目标角度，ai提供
static float Generate_SquareWave(void)
{
    wave_timer++;
    if (wave_timer >= SQUARE_WAVE_PERIOD_MS)
    {
        wave_timer = 0;
    }
    
    /* 前半周期高电平，后半周期低电平 */
    if (wave_timer < SQUARE_WAVE_PERIOD_MS / 2)
    {
        return SQUARE_WAVE_HIGH;
    }
    else
    {
        return SQUARE_WAVE_LOW;
    }
}


//1ms控制周期（主控制函数）
void Motor_Control_1ms(void)
{
    float voltage_out = 0;
    
    /* 生成方波目标角度，ai提供 */
    target_angle = Generate_SquareWave();
    
    if (control_mode == CONTROL_MODE_ANGLE)
    {
        /* ========== 串级PID控制 ========== */
        
        /* 外环：角度PID → 输出目标速度 */
        float speed_target = PID_Calc(&angle_pid,
                                       target_angle,
                                       gm6020_total_angle);
        
        /* 内环：速度PID → 输出电压 */
        voltage_out = PID_Calc(&speed_pid,
                               speed_target,
                               (float)gm6020_status.speed_rpm);
    }
    else
    {
        /* 单速度环模式 */
        voltage_out = PID_Calc(&speed_pid,
                               target_speed,
                               (float)gm6020_status.speed_rpm);
    }
    
    /* 发送电压指令 */
    GM6020_Send_Voltage((int32_t)voltage_out);

    /* 降频打印（每10ms打印一次，100Hz，适合VOFA+显示） */
    print_cnt++;
    if (print_cnt >= 10)
    {
        print_cnt = 0;
        
        /* VOFA+ FireWater协议格式：数据1,数据2\r\n */
        printf("%.2f,%.2f\r\n", target_angle, gm6020_total_angle);
        
        /* 调试用：打印更多信息（注释掉以提高性能） *，ai提供思路*/
        // printf("%.2f,%.2f,%.1f,%d,%.0f\r\n", 
        //        target_angle, gm6020_total_angle,
        //        angle_pid.output, gm6020_status.speed_rpm, voltage_out);
    }
}
