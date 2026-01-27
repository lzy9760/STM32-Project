#include "MyCAN.h"
#include "stm32f10x.h"
#include "Usart.h"
#include "Delay.h"

/* 全局变量定义 */
GM6020_StatusTypeDef gm6020_status[8];
float gm6020_total_angle[8];
static uint16_t gm6020_last_angle[8];
static int32_t  gm6020_round_cnt[8];

/* 新增：电压缓存数组，保存各电机的目标电压 */
static int32_t gm6020_voltage_buffer[8] = {0};

void GM6020_CAN_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    CAN_InitTypeDef CAN_InitStruct;
    CAN_FilterInitTypeDef CAN_FilterInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    CAN_DeInit(CAN1);
    CAN_InitStruct.CAN_TTCM = DISABLE;
    CAN_InitStruct.CAN_ABOM = ENABLE;
    CAN_InitStruct.CAN_AWUM = ENABLE;
    CAN_InitStruct.CAN_NART = DISABLE;
    CAN_InitStruct.CAN_RFLM = DISABLE;
    CAN_InitStruct.CAN_TXFP = DISABLE;
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStruct.CAN_BS1 = CAN_BS1_8tq;
    CAN_InitStruct.CAN_BS2 = CAN_BS2_3tq;
    CAN_InitStruct.CAN_Prescaler = 3;
    
    if (CAN_Init(CAN1, &CAN_InitStruct) != CAN_InitStatus_Success) {
        printf("CAN init failed!\r\n");
    }

    CAN_FilterInitStruct.CAN_FilterNumber = 0;
    CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStruct.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStruct.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStruct);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    NVIC_InitStruct.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

//发送电压函数
void GM6020_Set_Voltage(uint8_t id, int32_t voltage)
{
    if (id < 1 || id > 7) return;
    
    if (voltage > VOLTAGE_MAX) voltage = VOLTAGE_MAX;
    if (voltage < VOLTAGE_MIN) voltage = VOLTAGE_MIN;
    
    gm6020_voltage_buffer[id] = voltage;
}

//发送CAN帧（带超时保护）
static uint8_t CAN_Send_With_Timeout(CanTxMsg *tx_msg, uint32_t timeout_ms)
{
    uint8_t mailbox = CAN_Transmit(CAN1, tx_msg);
    
    if (mailbox == CAN_TxStatus_NoMailBox) {
        return 0; // 无可用邮箱
    }
    
    uint32_t timeout_cnt = timeout_ms * 1000; // 粗略计数
    while (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Pending) {
        if (--timeout_cnt == 0) {
            CAN_CancelTransmit(CAN1, mailbox); // 超时取消发送
            return 0;
        }
    }
    
    return (CAN_TransmitStatus(CAN1, mailbox) == CAN_TxStatus_Ok) ? 1 : 0;
}


 //发送所有电机控制帧
 //一次性发送0x1FF和0x2FF两帧，确保所有电机同时更新
void GM6020_Send_All_Voltage(void)
{
    CanTxMsg tx_msg;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_Data;
    tx_msg.DLC = 8;
    
    /* 发送0x1FF帧（控制ID 1-4） */
    tx_msg.StdId = 0x1FF;
    tx_msg.Data[0] = (gm6020_voltage_buffer[1] >> 8) & 0xFF;
    tx_msg.Data[1] = gm6020_voltage_buffer[1] & 0xFF;
    tx_msg.Data[2] = (gm6020_voltage_buffer[2] >> 8) & 0xFF;
    tx_msg.Data[3] = gm6020_voltage_buffer[2] & 0xFF;
    tx_msg.Data[4] = (gm6020_voltage_buffer[3] >> 8) & 0xFF;
    tx_msg.Data[5] = gm6020_voltage_buffer[3] & 0xFF;
    tx_msg.Data[6] = (gm6020_voltage_buffer[4] >> 8) & 0xFF;
    tx_msg.Data[7] = gm6020_voltage_buffer[4] & 0xFF;
    
    CAN_Send_With_Timeout(&tx_msg, 5);
    
    /* 发送0x2FF帧（控制ID 5-7） */
    tx_msg.StdId = 0x2FF;
    tx_msg.Data[0] = (gm6020_voltage_buffer[5] >> 8) & 0xFF;
    tx_msg.Data[1] = gm6020_voltage_buffer[5] & 0xFF;
    tx_msg.Data[2] = (gm6020_voltage_buffer[6] >> 8) & 0xFF;
    tx_msg.Data[3] = gm6020_voltage_buffer[6] & 0xFF;
    tx_msg.Data[4] = (gm6020_voltage_buffer[7] >> 8) & 0xFF;
    tx_msg.Data[5] = gm6020_voltage_buffer[7] & 0xFF;
    tx_msg.Data[6] = 0;
    tx_msg.Data[7] = 0;
    
    CAN_Send_With_Timeout(&tx_msg, 5);
}

//立即发送
void GM6020_Send_Voltage_ByID(uint8_t id, int32_t voltage)
{
    GM6020_Set_Voltage(id, voltage);
    // 不在这里发送，由主循环统一发送
}

void GM6020_Parse_Feedback(CanRxMsg *rx_msg)
{
    if (rx_msg->StdId < 0x205 || rx_msg->StdId > 0x20B)
        return;

    uint8_t id = rx_msg->StdId - 0x204;
    if (id < 1 || id > 7)
        return;

    uint16_t mechanical_angle = (rx_msg->Data[0] << 8) | rx_msg->Data[1];
    int16_t  speed_rpm        = (rx_msg->Data[2] << 8) | rx_msg->Data[3];
    int16_t  current_raw      = (rx_msg->Data[4] << 8) | rx_msg->Data[5];
    uint8_t  temperature      = rx_msg->Data[6];

    int16_t diff = mechanical_angle - gm6020_last_angle[id];
    if (diff > 4096)       gm6020_round_cnt[id]--;
    else if (diff < -4096) gm6020_round_cnt[id]++;

    gm6020_last_angle[id] = mechanical_angle;

    gm6020_total_angle[id] =
        (gm6020_round_cnt[id] * 8192 + mechanical_angle) * 360.0f / 8192.0f;//ai提供

    gm6020_status[id].mechanical_angle = mechanical_angle;
    gm6020_status[id].speed_rpm        = speed_rpm;
    gm6020_status[id].actual_current   = current_raw * 3.0f / 16384.0f;//ai提供
    gm6020_status[id].temperature      = temperature;
    gm6020_status[id].is_connected     = 1;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CanRxMsg rx_msg;
        CAN_Receive(CAN1, CAN_FIFO0, &rx_msg);
        GM6020_Parse_Feedback(&rx_msg);
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}
