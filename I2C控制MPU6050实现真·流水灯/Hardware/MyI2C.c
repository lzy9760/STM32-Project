#include "stm32f10x.h"

/**
 * @brief  I2C1初始化配置函数
 * @note   配置PB6(SCL)、PB7(SDA)为I2C复用开漏引脚，I2C1工作在主模式（标准速率100kHz）
 * @param  无
 * @retval 无
 */
void I2C_Init_Config(void)
{
    // 定义GPIO初始化结构体（配置引脚）、I2C初始化结构体（配置I2C外设）
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef  i2c;

    // 1. 开启时钟：GPIOB（PB6/PB7）挂在APB2总线，I2C1挂在APB1总线
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 开启GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);  // 开启I2C1时钟

    // 2. 配置GPIO引脚：PB6=SCL（时钟线），PB7=SDA（数据线）
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;              // 选择PB6和PB7
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;                     // 复用开漏输出（I2C必须用开漏！因为SDA/SCL是双向总线）
    gpio.GPIO_Speed = GPIO_Speed_50MHz;                   // 引脚速率50MHz
    GPIO_Init(GPIOB, &gpio);                              // 应用GPIO配置

    // 3. 配置I2C1核心参数（主模式，和从设备通信）
    i2c.I2C_Mode = I2C_Mode_I2C;                          // 工作模式：标准I2C模式（不是SMBus）
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;                  // 时钟占空比：Tlow/Thigh=2（标准模式推荐）
    i2c.I2C_OwnAddress1 = 0x00;                           // 自身地址（主模式下不用，设为0即可）
    i2c.I2C_Ack = I2C_Ack_Enable;                         // 使能应答（I2C通信必须开Ack，除非最后一个字节）
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 地址长度：7位（绝大多数I2C设备用7位地址，比如MPU6050）
    i2c.I2C_ClockSpeed = 100000;                          // I2C时钟速度：100kHz（标准模式，高速模式是400kHz）

    // 4. 应用I2C配置，然后使能I2C1外设
    I2C_Init(I2C1, &i2c);    // 初始化I2C1
    I2C_Cmd(I2C1, ENABLE);   // 开启I2C1外设（相当于打开总开关）
}

/**
 * @brief  I2C单字节写函数：向指定设备的指定寄存器写入1个字节
 * @note   流程：起始信号→发设备地址（写）→发寄存器地址→发数据→停止信号
 * @param  dev_addr：I2C设备7位地址（比如MPU6050是0x68）
 * @param  reg_addr：设备内部寄存器地址（比如MPU6050的PWR_MGMT_1寄存器是0x6B）
 * @param  data：要写入寄存器的字节数据
 * @retval 无
 */
void I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    // 1. 生成I2C起始信号（SCL高电平时，SDA从高变低），开启通信
    I2C_GenerateSTART(I2C1, ENABLE);
    // 等待：I2C1进入主模式（起始信号发送成功）
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    // 2. 发送设备7位地址 + 写方向（I2C_Direction_Transmitter=0）
    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Transmitter);
    // 等待：从设备应答，I2C1进入主发送模式
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // 3. 发送要写入的寄存器地址（告诉从设备：要写哪个寄存器）
    I2C_SendData(I2C1, reg_addr);
    // 等待：寄存器地址发送完成，且从设备应答
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // 4. 发送要写入寄存器的具体数据
    I2C_SendData(I2C1, data);
    // 等待：数据发送完成，且从设备应答
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // 5. 生成I2C停止信号（SCL高电平时，SDA从低变高），结束通信
    I2C_GenerateSTOP(I2C1, ENABLE);
}

/**
 * @brief  I2C多字节读函数：从指定设备的指定寄存器读取多个字节到缓冲区
 * @note   流程：起始信号→发设备地址（写）→发寄存器地址→重复起始信号→发设备地址（读）→读数据→停止信号
 * @param  dev_addr：I2C设备7位地址
 * @param  reg_addr：设备内部寄存器地址
 * @param  buf：接收数据的缓冲区指针（读取的数据存在这里）
 * @param  len：要读取的字节数
 * @retval 无
 */
void I2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint8_t len)
{
    // --------------------- 第一步：先写寄存器地址（告诉从设备要读哪个寄存器） ---------------------
    // 1. 生成起始信号
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    // 2. 发送设备地址 + 写方向（因为要先告诉从设备读哪个寄存器，所以先写）
    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // 3. 发送要读取的寄存器地址
    I2C_SendData(I2C1, reg_addr);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // --------------------- 第二步：切换为读模式（重复起始信号，不发停止！） ---------------------
    // 4. 生成重复起始信号（Repeated START）：切换读写方向时不能发停止，否则通信中断
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    // 5. 发送设备地址 + 读方向（I2C_Direction_Receiver=1）
    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    // --------------------- 第三步：循环读取指定长度的字节 ---------------------
    while(len) // 只要还有字节没读，就循环
    {
        // 最后1个字节：禁用Ack（I2C协议规定：最后1个字节主设备不发Ack，告诉从设备停止发送）
        if(len == 1)
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);  // 禁用应答
        }

        // 检测：是否接收到1个字节（从设备发过来的数据）
        if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            *buf++ = I2C_ReceiveData(I2C1); // 读取接收到的字节，存入缓冲区，指针后移
            len--;                          // 剩余读取长度减1
            // 最后1个字节读完：生成停止信号，结束通信
            if(len == 0)
            {
                I2C_GenerateSTOP(I2C1, ENABLE);  // 发送停止信号
            }
        }
    }
    // 恢复Ack使能（方便下次读操作，避免每次都要开）
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}
