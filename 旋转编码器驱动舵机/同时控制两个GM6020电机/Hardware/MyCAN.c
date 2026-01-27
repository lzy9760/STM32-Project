#include "MyCAN.h"
#include "stm32f10x.h"                  // Device header


// 全局变量定义（与原代码一致）
moto_info_t motor_yaw_info = {0}; // 初始化电机信息
uint16_t can_cnt = 0;             // 计数变量（原代码未使用，保留定义）

// -------------------------- 私有函数声明（内部初始化用） --------------------------
static void CAN_GPIO_Init(void);    // CAN引脚初始化（PB8=RX，PB9=TX）
static void CAN_NVIC_Init(void);    // CAN中断控制器配置
static void CAN_Controller_Init(void); // CAN控制器时序初始化（1Mbps波特率）

// -------------------------- 公有函数实现 --------------------------
/**
 * @brief  CAN初始化入口（整合GPIO、控制器、过滤器、中断）
 * @note   适配GM6020要求：1Mbps波特率，接收所有标准帧，解析ID=0x205反馈
 */
void can_filter_init(void) {
    // 1. 初始化GPIO（CAN收发引脚）
    CAN_GPIO_Init();
    
    // 2. 初始化CAN控制器（时序+模式）
    CAN_Controller_Init();
    
    // 3. 配置CAN过滤器（接收所有标准帧，关联FIFO0）
    CAN_FilterInitTypeDef CAN_FilterInitStruct;
    
    // 过滤器0配置（对应原代码FilterBank0）
    CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
    CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask  ; // ID掩码模式
    CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit  ; // 32位宽
    CAN_FilterInitStruct.CAN_FilterIdHigh = 0x0000; // 过滤ID高8位（接收所有）
    CAN_FilterInitStruct.CAN_FilterIdLow = 0x0000;  // 过滤ID低8位
    CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x0000; // 掩码高8位（不限制）
    CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;  // 掩码低8位
    CAN_FilterInitStruct.CAN_FilterNumber = 0; // 过滤器组0
    CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; // 关联FIFO0
    CAN_FilterInit(&CAN_FilterInitStruct);
    
    // 过滤器13配置（F1仅支持0~13，修正为13）
//    CAN_FilterInitStruct.CAN_FilterNumber = 13; // 过滤器组13
//    CAN_FilterInit(&CAN_FilterInitStruct);
    
    // 4. 配置NVIC中断（使能RX_FIFO0中断）
    CAN_NVIC_Init();
    
    
    // 6. 使能CAN RX_FIFO0非空中断（触发中断的关键）
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/**
 * @brief  GM6020电机电压控制指令发送
 * @param  v1: 电压指令（范围：-25000~25000，对应文档协议）
 * @note   控制ID=0x1FF（单电机，占DATA[0-1]），DLC=8字节
 */
void set_GM6020_motor_voltage(int16_t v1) {
    CanTxMsg CAN_TxHeader; // 发送帧头结构体
    uint8_t tx_data[8] = {0};         // 发送数据（初始化全0）
    uint8_t tx_mailbox;               // 发送邮箱号
    
    // 1. 配置发送帧头（符合GM6020 CAN协议）
    CAN_TxHeader.StdId = 0x1FF;       // 标准ID=0x1FF（控制ID1~4电机）
    CAN_TxHeader.ExtId = 0x00;        // 无扩展ID
    CAN_TxHeader.RTR = CAN_RTR_Data;  // 数据帧
    CAN_TxHeader.IDE = CAN_ID_STD;    // 标准帧格式
    CAN_TxHeader.DLC = 8;             // 数据长度=8字节
    CAN_TxHeader. = DISABLE; // 不携带时间戳
    
    // 2. 填充电压指令（高8位在前，低8位在后）
    tx_data[0] = (v1 >> 8) & 0xFF;    // 电压高8位
    tx_data[1] = v1 & 0xFF;           // 电压低8位
    // 其余6字节默认0（不控制其他电机）
    
    // 3. 等待发送邮箱空，发送数据
    while (CAN_Transmit(CAN1, &CAN_TxHeader, tx_data, &tx_mailbox) == CAN_TxStatus_NoMailBox);
}

/**
 * @brief  CAN1 RX_FIFO0中断服务函数（解析GM6020反馈）
 * @note   反馈ID=0x205（ID1电机），数据格式：角度(0-1)、转速(2-3)、电流(4-5)、温度(6)
 */
void CAN1_RX0_IRQHandler(void) {
    CAN_RxHeaderTypeDef CAN_RxHeader; // 接收帧头结构体
    uint8_t rx_data[8] = {0};         // 接收数据缓冲区
    
    // 1. 检查是否为RX_FIFO0非空中断
    if (CAN_GetITStatus(CAN1, CAN_IT_RX_FIFO0_NOT_EMPTY) != RESET) {
        // 2. 读取接收数据（自动清除中断标志）
        CAN_Receive(CAN1, CAN_FIFO0, &CAN_RxHeader, rx_data);
        
        // 3. 解析GM6020反馈（仅处理ID=0x205的标准帧）
        if (CAN_RxHeader.StdId == 0x205 && CAN_RxHeader.IDE == CAN_ID_STD) {
            motor_yaw_info.rotor_angle = (rx_data[0] << 8) | rx_data[1];    // 机械角度（0~8191）
            motor_yaw_info.rotor_speed = (rx_data[2] << 8) | rx_data[3];    // 转速（rpm，有正负）
            motor_yaw_info.torque_current = (rx_data[4] << 8) | rx_data[5]; // 转矩电流（有正负）
            motor_yaw_info.temp = rx_data[6];                               // 电机温度（℃）
            
            can_cnt++; // 计数+1（原代码保留变量）
        }
    }
}

// -------------------------- 私有函数实现（内部初始化） --------------------------
/**
 * @brief  CAN GPIO初始化（PB8=RX，PB9=TX，适配GM6020接线）
 */
static void CAN_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 1. 使能时钟（GPIO B + AFIO，CAN需复用功能）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); // CAN1挂载APB1
    
    // 2. 配置PB8（CAN_RX）：上拉输入（防止总线空闲时电平不稳定）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 3. 配置PB9（CAN_TX）：复用推挽输出（CAN复用功能）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  CAN控制器时序初始化（关键：满足GM6020的1Mbps波特率）
 * @note   APB1时钟=36MHz，时序参数：Prescaler=3，BS1=9TQ，BS2=2TQ，SJW=1TQ
 *         波特率计算：36MHz / (3 * (9+2+1)) = 36/(3*12) = 1Mbps
 */
static void CAN_Controller_Init(void) {
    CAN_InitTypeDef CAN_InitStruct;
    
    // 1. 配置CAN工作模式
    CAN_InitStruct.CAN_TTCM = DISABLE; // 关闭时间触发通信模式
    CAN_InitStruct.CAN_ABOM = DISABLE; // 关闭自动离线管理
    CAN_InitStruct.CAN_AWUM = DISABLE; // 关闭自动唤醒模式
    CAN_InitStruct.CAN_NART = ENABLE;  // 使能自动重发（确保指令可靠）
    CAN_InitStruct.CAN_RFLM = DISABLE; // 关闭接收FIFO锁定
    CAN_InitStruct.CAN_TXFP = DISABLE; // 关闭发送FIFO优先级
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal; // 正常模式（非回环）
    
    // 2. 配置时序参数（1Mbps波特率关键）
    CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;       // 同步跳变宽度=1TQ
    CAN_InitStruct.CAN_BS1 = CAN_BS1_9tq;       // 时间段1=9TQ（采样点前）
    CAN_InitStruct.CAN_BS2 = CAN_BS2_2tq;       // 时间段2=2TQ（采样点后）
    CAN_InitStruct.CAN_Prescaler = 3;           // 预分频系数=3
    
    // 3. 初始化CAN控制器
    CAN_Init(CAN1, &CAN_InitStruct);
}

/**
 * @brief  CAN NVIC中断配置（确保中断能正常触发）
 * @note   NVIC分组=2（与用户系统初始化一致），抢占优先级=2，子优先级=0
 */
static void CAN_NVIC_Init(void) {
    NVIC_InitTypeDef NVIC_InitStruct;
    
    // 1. 配置CAN1 RX0中断
    NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX0_IRQn ; // CAN1 RX0中断通道
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级2
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;       // 子优先级0
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;          // 使能中断
    NVIC_Init(&NVIC_InitStruct);
}

