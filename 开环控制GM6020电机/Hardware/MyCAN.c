#include "MyCAN.h"
#include "stm32f10x.h"
#include "Usart.h"
#include "Delay.h"
//MyCAN.c
/* 全局变量定义 */
GM6020_StatusTypeDef gm6020_status = {0};
float gm6020_total_angle = 0.0f;
static uint16_t gm6020_last_angle = 0;
static int32_t gm6020_round_cnt = 0;
//uint16_t flag = 0;

/* CAN初始化（修正波特率为1Mbps，手册要求） */
void GM6020_CAN_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    CAN_InitTypeDef CAN_InitStruct;
    CAN_FilterInitTypeDef CAN_FilterInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* 1. 使能时钟（AFIO必须开启，否则GPIO复用失效） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* 2. 配置GPIO（PA11=CAN_RX，PA12=CAN_TX） */
    // CAN_RX（PA11）：上拉输入（防止总线空闲时电平不稳定）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // CAN_TX（PA12）：复用推挽输出（由CAN外设控制）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 3. 配置CAN（关键修正：波特率1Mbps，PCLK1=36MHz） */
    CAN_DeInit(CAN1);  // 恢复默认配置
    CAN_InitStruct.CAN_TTCM = DISABLE;         // 关闭时间触发模式
    CAN_InitStruct.CAN_ABOM = ENABLE;          // 自动离线管理（异常后自动恢复）
    CAN_InitStruct.CAN_AWUM = ENABLE;          // 自动唤醒
    CAN_InitStruct.CAN_NART = DISABLE;         // 开启自动重发（确保指令送达）
    CAN_InitStruct.CAN_RFLM = DISABLE;         // 关闭接收FIFO锁定
    CAN_InitStruct.CAN_TXFP = DISABLE;         // 关闭发送优先级
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal; // 正常模式

    // 波特率计算：36MHz / (Prescaler * (BS1+BS2+1)) = 1MHz
    // 选择：Prescaler=3，BS1=8tq，BS2=3tq → 3*(8+3+1)=36 → 36MHz/36=1MHz（完全匹配手册要求）
    CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;      // 同步跳转宽度1tq
    CAN_InitStruct.CAN_BS1 = CAN_BS1_8tq;      // 时间段1=8tq
    CAN_InitStruct.CAN_BS2 = CAN_BS2_3tq;      // 时间段2=3tq
    CAN_InitStruct.CAN_Prescaler = 3;          // 预分频器=3
    if (CAN_Init(CAN1, &CAN_InitStruct) != CAN_InitStatus_Success) {
        printf("CAN init failed!\r\n");  // 初始化失败提示（调试用）
    }

    /* 4. 配置CAN过滤器（接收所有帧，调试阶段方便验证） */
    CAN_FilterInitStruct.CAN_FilterNumber = 0;
    CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStruct.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStruct.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x0000;  // 掩码全0=接收所有
    CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStruct);

    /* 5. 配置接收中断（优先级适中，避免抢占主程序） */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);  // 开启FIFO0满中断
    NVIC_InitStruct.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

//发送电压函数
void GM6020_Send_Voltage(int32_t voltage)
{
    // 电压限幅（防止超出手册范围）
    if (voltage > VOLTAGE_MAX) voltage = VOLTAGE_MAX;
    if (voltage < VOLTAGE_MIN) voltage = VOLTAGE_MIN;

    CanTxMsg tx_msg;
    tx_msg.StdId = CAN_CMD_VOLTAGE;  // 控制帧ID=0x1FF（手册P6）
    tx_msg.IDE = CAN_ID_STD;         // 标准帧
    tx_msg.RTR = CAN_RTR_Data;       // 数据帧
    tx_msg.DLC = 8;                  // 数据长度8字节

    // 初始化数据域为0（避免垃圾数据）
    for (uint8_t i = 0; i < 8; i++) tx_msg.Data[i] = 0;

    // 计算对应ID的电压数据偏移（ID1→0/1字节，ID2→2/3字节，ID3→4/5字节，手册P6）
    uint8_t offset = (GM6020_MOTOR_ID - 1) * 2;
    tx_msg.Data[offset]     = (voltage >> 8) & 0xFF;  // 高8位
    tx_msg.Data[offset + 1] = voltage & 0xFF;         // 低8位

    uint8_t mailbox = CAN_Transmit(CAN1, &tx_msg);
if (mailbox == CAN_TxStatus_NoMailBox) {
	
    return;
}






//等待发送完成
while(CAN_TransmitStatus(CAN1, mailbox) != CAN_TxStatus_Ok);

//}
//    // 发送失败提示（仅调试用，可注释）
//    if (tx_state != CAN_TxStatus_Ok) {
//        printf("CAN send failed! Retries=%d\r\n", retries);
//    }
}

//解析电机反馈数据
void GM6020_Parse_Feedback(CanRxMsg* rx_msg)
{
    // 只处理当前电机的反馈帧（0x204+ID=0x207）
    uint32_t expected_feedback_id = CAN_FEEDBACK_BASE + GM6020_MOTOR_ID;
    if (rx_msg->StdId != expected_feedback_id) {
        return;
    }

    // 解析数据域
    uint16_t mechanical_angle = (rx_msg->Data[0] << 8) | rx_msg->Data[1];  // 机械角度（0~8191）
    int16_t speed_rpm = (int16_t)((rx_msg->Data[2] << 8) | rx_msg->Data[3]); // 转速（rpm）
    int16_t current_raw = (int16_t)((rx_msg->Data[4] << 8) | rx_msg->Data[5]); // 电流原始值
    uint8_t temperature = rx_msg->Data[6];  // 温度（℃）

    // 多圈角度解算（避免角度溢出）
    int16_t angle_diff = (int16_t)(mechanical_angle - gm6020_last_angle);
    if (angle_diff > 4096) {  // 顺时针转一圈（超过半量程）
        gm6020_round_cnt--;
    } else if (angle_diff < -4096) {  // 逆时针转一圈
        gm6020_round_cnt++;
    }
    gm6020_last_angle = mechanical_angle;
    gm6020_total_angle = (gm6020_round_cnt * 8192 + mechanical_angle) * 360.0f / 8192.0f;  // 总角度（度），ai提供

    // 更新状态变量
    gm6020_status.mechanical_angle = mechanical_angle;
    gm6020_status.speed_rpm = speed_rpm;
    gm6020_status.actual_current = (float)current_raw / 16384.0f * 3.0f;  // 电流换算（-3A~3A），ai提供
    gm6020_status.temperature = temperature;
    gm6020_status.is_connected = 1;  // 标记为已连接
}

/* CAN接收中断服务函数（FIFO0满中断） */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	   
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CanRxMsg rx_msg;
        CAN_Receive(CAN1, CAN_FIFO0, &rx_msg);  // 读取接收数据
        GM6020_Parse_Feedback(&rx_msg);          // 解析反馈
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0); // 清除中断标志
			  
    }
		
}
