#include "MyCAN.h"
#include "Motor.h"
//MyCAN.c
CanRxMsg RxMsg;
CanTxMsg TxMsg;

/* CAN GPIO + NVIC + Filter + Init */
void CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef  CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* PA11 RX  PA12 TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    /* 1Mbps @36MHz */
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
    CAN_InitStructure.CAN_Prescaler = 3;

    CAN_Init(CAN1, &CAN_InitStructure);

    /* 接收所有 ID */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow  = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    /* ⚠️ F103C8 正确中断 */
    NVIC_InitStructure.NVIC_IRQChannel =  USB_LP_CAN1_RX0_IRQn;/*!< USB Device Low Priority or CAN1 RX0 Interrupts */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* 发送电流指令 */
void CAN1_Send_Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    // ========== 第一部分：配置CAN帧的基础属性（按C620协议要求） ==========
    TxMsg.StdId = 0x201;  // 1. 设置CAN帧的标准ID为0x200
                          // 关键：C620电调规定，接收电机控制指令的CAN ID就是0x200，改了电调就识别不了
    TxMsg.IDE   = CAN_Id_Standard;  // 2. 声明这是「标准帧」（11位ID）
                                    // 注意：标准库正确宏是CAN_ID_STD，这里可能是拼写笔误，功能等价
    TxMsg.RTR   = CAN_RTR_Data;     // 3. 声明这是「数据帧」（不是远程帧）
                                    // 解释：数据帧是“主动发数据”，远程帧是“请求别人发数据”，控制电机用数据帧
    TxMsg.DLC   = 8;                // 4. 设置数据长度为8字节
                                    // 关键：C620要求控制指令必须传8字节（4路电机，每路2字节）

    // ========== 第二部分：拆分电流值为高低字节（CAN数据是按字节传输的） ==========
    // iq1~iq4是int16_t类型（2字节），但CAN数据帧的Data数组是uint8_t（1字节），所以要拆成高低字节
    TxMsg.Data[0] = iq1 >> 8;  // 5. 取iq1的高8位（把数值右移8位，只保留高位）
    TxMsg.Data[1] = iq1;       // 6. 取iq1的低8位（直接赋值，低8位会自动保留）
    TxMsg.Data[2] = iq2 >> 8;  // 7. 同理，iq2的高8位
    TxMsg.Data[3] = iq2;       // 8. iq2的低8位
    TxMsg.Data[4] = iq3 >> 8;  // 9. iq3的高8位
    TxMsg.Data[5] = iq3;       // 10. iq3的低8位
    TxMsg.Data[6] = iq4 >> 8;  // 11. iq4的高8位
    TxMsg.Data[7] = iq4;       // 12. iq4的低8位
    // 举例：如果iq1=8000（int16_t），二进制是0001 1111 0100 0000
    // 高8位：0001 1111（对应十进制31），低8位：0100 0000（对应十进制64）
    // 电调收到后会把高低字节拼回去，得到8000，识别为对应电流指令

    // ========== 第三部分：发送CAN帧 ==========
    CAN_Transmit(CAN1, &TxMsg);  // 13. 调用STM32标准库函数，把配置好的帧通过CAN1发送出去
}


void CAN1_RX0_IRQHandler(void)  // 1. CAN1 FIFO0中断服务函数，收到数据就能自动进入中断
{
    // 2. 检查CAN1的FIFO0里是否有待处理的报文（防止空读）
    if (CAN_MessagePending(CAN1, CAN_FIFO0))  
    {
        // 3. 从FIFO0读取收到的CAN报文，存入RxMsg结构体
        CAN_Receive(CAN1, CAN_FIFO0, &RxMsg);  
			
			CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//清除中断标志位

        // 4. 筛选：只处理M3508电机1的反馈报文（ID=0x201）
        if (RxMsg.StdId == 0x201)   // 电机 1  
        {
            // 5. 调用自定义解析函数，解析报文里的电机数据
            M3508_Parse(RxMsg.Data);  
        }
    }
}

