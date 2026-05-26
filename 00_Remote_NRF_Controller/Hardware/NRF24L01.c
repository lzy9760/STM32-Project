
/***************************************************************************************
  * 本程序由江协科技创建并免费开源共享
  * 你可以任意查看、使用和修改，并应用到自己的项目之中
  * 程序版权归江协科技所有，任何人或组织不得将其据为己有
  * 
  * 程序名称：				NRF24L01无线通信模块驱动程序
  * 程序创建时间：			2025.6.9
  * 当前程序版本：			V1.0
  * 当前版本发布时间：		2024.6.9
  * 
  * 江协科技官方网站：		jiangxiekeji.com
  * 江协科技官方淘宝店：	jiangxiekeji.taobao.com
  * 程序介绍及更新动态：	jiangxiekeji.com/tutorial/nrf24l01.html
  * 
  * 如果你发现程序中的漏洞或者笔误，可通过邮件向我们反馈：feedback@jiangxiekeji.com
  * 发送邮件之前，你可以先到更新动态页面查看最新程序，如果此问题已经修改，则无需再发邮件
  ***************************************************************************************
  */

#include "main.h"
#include "NRF24L01_Define.h"

/*全局变量*********************/

/*发送部分*/
uint8_t NRF24L01_TxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};		//发送地址，固定5字节
#define NRF24L01_TX_PACKET_WIDTH		4							//发送数据包宽度，范围：1~32字节
uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH];				//发送数据包
uint8_t NRF24L01_PoseTxAddress[5] = {0x66, 0x77, 0x88, 0x99, 0xAA};
uint8_t NRF24L01_PoseRxAddress[5] = {0x66, 0x77, 0x88, 0x99, 0xAA};

/*接收部分*/
uint8_t NRF24L01_RxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};		//接收通道0地址，固定5字节
#define NRF24L01_RX_PACKET_WIDTH		4							//接收通道0数据包宽度，范围：1~32字节
uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];				//接收数据包

#define NRF24L01_RF_CHANNEL_VALUE		0x4C
#define NRF24L01_RF_SETUP_VALUE			0x27
#define NRF24L01_SETUP_RETR_VALUE		0x15						//ARD=500us, ARC=5: short retry for basic Auto ACK
#define NRF24L01_TX_TIMEOUT_MS			8U							//Shortened: fail fast so main loop stays responsive
#define NRF24L01_TX_TIMEOUT_POLLS		1000U
#define NRF24L01_FEATURE_ACK_PAYLOAD	0x06						//EN_ACK_PAY | EN_DPL
#define NRF24L01_DYNPD_P0				0x01						//Dynamic payload on pipe 0
#define NRF24L01_ENABLE_ACK_PAYLOAD		0
#define NRF24L01_STATUS_RX_DR			0x40
#define NRF24L01_STATUS_TX_DS			0x20
#define NRF24L01_STATUS_MAX_RT			0x10
#define NRF24L01_STATUS_TX_FLAGS		(NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT)
#define NRF24L01_FIFO_RX_EMPTY			0x01

/**
  * 提示：设备A和设备B进行通信
  * A发B收时，A的发送地址、发送数据包宽度要与B的接收地址、接收数据包宽度对应相同
  * B发A收时，B的发送地址、发送数据包宽度要与A的接收地址、接收数据包宽度对应相同
  * 通常情况下，可以将A和B的发送地址、接收地址全设置一样，A和B的发送数据包宽度、接收数据包宽度也全设置一样
  * 这样A和B可以使用完全一样的模块程序，操作更加方便，也不容易搞混
  * 
  */

/*********************全局变量*/


/*引脚配置*********************/

/**
  * 函    数：NRF24L01写CE高低电平
  * 参    数：要写入CE的电平值，范围：0/1
  * 返 回 值：无
  * 说    明：当上层函数需要写CE时，此函数会被调用
  *           用户需要根据参数传入的值，将CE置为高电平或者低电平
  *           当参数传入0时，置CE为低电平，当参数传入1时，置CE为高电平
  */
void NRF24L01_W_CE(uint8_t BitValue)
{
	/*根据BitValue的值，将CE置高电平或者低电平*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * 函    数：NRF24L01写CSN高低电平
  * 参    数：要写入CSN的电平值，范围：0/1
  * 返 回 值：无
  * 说    明：当上层函数需要写CSN时，此函数会被调用
  *           用户需要根据参数传入的值，将CSN置为高电平或者低电平
  *           当参数传入0时，置CSN为低电平，当参数传入1时，置CSN为高电平
  */
void NRF24L01_W_CSN(uint8_t BitValue)
{
	/*根据BitValue的值，将CSN置高电平或者低电平*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * 函    数：NRF24L01写SCK高低电平
  * 参    数：要写入SCK的电平值，范围：0/1
  * 返 回 值：无
  * 说    明：当上层函数需要写SCK时，此函数会被调用
  *           用户需要根据参数传入的值，将SCK置为高电平或者低电平
  *           当参数传入0时，置SCK为低电平，当参数传入1时，置SCK为高电平
  */
void NRF24L01_W_SCK(uint8_t BitValue)
{
	/*根据BitValue的值，将SCK置高电平或者低电平*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * 函    数：NRF24L01写MOSI高低电平
  * 参    数：要写入MOSI的电平值，范围：0/1
  * 返 回 值：无
  * 说    明：当上层函数需要写MOSI时，此函数会被调用
  *           用户需要根据参数传入的值，将MOSI置为高电平或者低电平
  *           当参数传入0时，置MOSI为低电平，当参数传入1时，置MOSI为高电平
  */
void NRF24L01_W_MOSI(uint8_t BitValue)
{
	/*根据BitValue的值，将MOSI置高电平或者低电平*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * 函    数：NRF24L01读MISO高低电平
  * 参    数：无
  * 返 回 值：读取得到MISO的电平值，范围：0/1
  * 说    明：当上层函数需要读MISO时，此函数会被调用
  *           用户需要读取MISO引脚，返回此引脚的高低电平状态
  *           当MISO为高电平时，返回1，当MISO为低电平时，返回0
  */
uint8_t NRF24L01_R_MISO(void)
{
	/*取MISO引脚的高低电平并返回*/
	return (uint8_t)(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET);
}

/**
  * 本代码使用查询的方式获取设备状态，因此不需要使用IRQ引脚
  */

/**
  * 函    数：NRF24L01引脚初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：当上层函数需要初始化时，此函数会被调用
  *           用户需要将CSN、CE、MISO、SCK引脚初始化为推挽输出模式，MISO引脚初始化为上拉输入模式
  */
void NRF24L01_GPIO_Init(void)
{
	/*开启GPIO时钟*/
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	/*将CE、CSN、SCK、MOSI引脚初始化为推挽输出模式*/
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*将MISO引脚初始化为上拉输入模式*/
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Pin = GPIO_PIN_14;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*置引脚初始化后的默认电平*/
	NRF24L01_W_CE(0);		//CE默认为0，退出收发模式
	NRF24L01_W_CSN(1);		//CSN默认为1，不选中从机
	NRF24L01_W_SCK(0);		//SCK默认为0，对应SPI模式0
	NRF24L01_W_MOSI(0);		//MOSI默认电平随意，1和0均可
}

/*********************引脚配置*/


/*通信协议*********************/

/**
  * 函    数：SPI交换一个字节
  * 参    数：Byte 要发送的一个字节数据，范围：0x00~0xFF
  * 返 回 值：接收得到的一个字节数据，范围：0x00~0xFF
  */
uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte)
{
	uint8_t i;
	
	/*此处使用SPI模式0进行通信*/
	/*循环8次，主机依次移出和移入数据的每一位*/
	for (i = 0; i < 8; i ++)
	{
		/*SPI为高位先行，因此移出高位至MOSI引脚*/
		if (Byte & 0x80)			//判断Byte的最高位
		{
			NRF24L01_W_MOSI(1);		//如果为1，则给MOSI输出1
		}
		else
		{
			NRF24L01_W_MOSI(0);		//如果为0，则给MOSI输出0
		}
		Byte <<= 1;					//Byte左移一位，最低位空出来用于接收数据位
		
		/*产生SCK上升沿*/
		NRF24L01_W_SCK(1);
		
		/*从MISO引脚移入数据，存入Byte的最低位*/
		if (NRF24L01_R_MISO())		//读取MISO引脚
		{
			Byte |= 0x01;			//如果为1，则给Byte最低位置1
		}							//如果为0，则不做任何操作，因为左移后低位默认补0
		
		/*产生SCK下降沿*/
		NRF24L01_W_SCK(0);
	}
	
	/*返回Byte数据，此时的Byte为SPI交换接收得到的一个字节数据*/
	return Byte;
}

/*********************通信协议*/


/*指令实现*********************/

/**
  * 函    数：NRF24L01读取寄存器（一个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 返 回 值：指定寄存器的数据，范围：0x00~0xFF
  */
uint8_t NRF24L01_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress);
	
	/*发送读寄存器指令后，开始交换接收，得到指定地址的数据*/
	Data = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
	
	/*返回读到的一个字节数据*/
	return Data;
}

/**
  * 函    数：NRF24L01读取寄存器（多个字节）
  * 参    数：RegAddress 指定寄存器的地址，范围：0x00~0x1F
  * 参    数：DataArray 读取得到的数据数组，输出参数
  * 参    数：Count 指定读取的数量，范围：0~5
  * 返 回 值：无
  */
void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_REGISTER | RegAddress);
	
	/*发送读寄存器指令后，开始交换接收，循环接收多次，得到指定地址下的多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将接收到的数据写入到输出参数DataArray中*/
		DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01写入寄存器（一个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 参    数：Data 要写入的一个字节数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress);
	
	/*发送写寄存器指令后，开始交换发送，在指定地址下写入数据*/
	NRF24L01_SPI_SwapByte(Data);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01写入寄存器（多个字节）
  * 参    数：RegAddress 指定寄存器地址，范围：0x00~0x1F
  * 参    数：DataArray 要写入的数据数组，输入参数
  * 参    数：Count 指定写入的数量，范围：0~5
  * 返 回 值：无
  */
void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写寄存器（低5位为寄存器地址）*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_REGISTER | RegAddress);
	
	/*发送写寄存器指令后，开始交换发送，循环发送多次，在指定地址下写入多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将输入参数DataArray的数据写入到指定地址中*/
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01读取Rx有效载荷
  * 参    数：DataArray 读取得到的数据数组，输出参数
  * 参    数：Count 指定读取的数量，范围：0~32
  * 返 回 值：无
  */
void NRF24L01_ReadRxPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，读取Rx有效载荷*/
	NRF24L01_SPI_SwapByte(NRF24L01_R_RX_PAYLOAD);
	
	/*发送读取Rx有效载荷指令后，开始交换接收，循环接收多次，得到多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将读取的数据写入到输出参数DataArray中*/
		DataArray[i] = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01写入Tx有效载荷
  * 参    数：DataArray 要写入的数据数组，输入参数
  * 参    数：Count 指定写入的数量，范围：0~5
  * 返 回 值：无
  */
void NRF24L01_WriteTxPayload(uint8_t *DataArray, uint8_t Count)
{
	uint8_t i;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);
	
	/*交换发送一个字节，通信开始的第一个字节为指令码，写入Tx有效载荷*/
	NRF24L01_SPI_SwapByte(NRF24L01_W_TX_PAYLOAD);
	
	/*发送写入Tx有效载荷指令后，开始交换发送，循环发送多次，写入多个数据*/
	for (i = 0; i < Count; i ++)
	{
		/*将输入参数DataArray的数据写入到Tx有效载荷中*/
		NRF24L01_SPI_SwapByte(DataArray[i]);
	}
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01清空Tx FIFO的所有数据
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_FlushTx(void)
{
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，清空Tx FIFO*/
	NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_TX);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

/**
  * 函    数：NRF24L01清空Rx FIFO的所有数据
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_FlushRx(void)
{
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，清空Rx FIFO*/
	NRF24L01_SPI_SwapByte(NRF24L01_FLUSH_RX);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
}

static void NRF24L01_ActivateFeatures(void)
{
	NRF24L01_W_CSN(0);
	NRF24L01_SPI_SwapByte(NRF24L01_ACTIVATE);
	NRF24L01_SPI_SwapByte(0x73);
	NRF24L01_W_CSN(1);
}

static void NRF24L01_DisableAckPayload(void)
{
	NRF24L01_WriteReg(NRF24L01_FEATURE, 0x00);
	NRF24L01_WriteReg(NRF24L01_DYNPD, 0x00);
	if ((NRF24L01_ReadReg(NRF24L01_FEATURE) != 0x00) ||
	    (NRF24L01_ReadReg(NRF24L01_DYNPD) != 0x00))
	{
		NRF24L01_ActivateFeatures();
		NRF24L01_WriteReg(NRF24L01_FEATURE, 0x00);
		NRF24L01_WriteReg(NRF24L01_DYNPD, 0x00);
	}
}

#if (NRF24L01_ENABLE_ACK_PAYLOAD != 0)
static void NRF24L01_EnableAckPayload(void)
{
	NRF24L01_WriteReg(NRF24L01_FEATURE, NRF24L01_FEATURE_ACK_PAYLOAD);
	if (NRF24L01_ReadReg(NRF24L01_FEATURE) != NRF24L01_FEATURE_ACK_PAYLOAD)
	{
		NRF24L01_ActivateFeatures();
		NRF24L01_WriteReg(NRF24L01_FEATURE, NRF24L01_FEATURE_ACK_PAYLOAD);
	}
	NRF24L01_WriteReg(NRF24L01_DYNPD, NRF24L01_DYNPD_P0);
}
#endif

/**
  * 函    数：NRF24L01读取状态寄存器
  * 参    数：无
  * 返 回 值：状态寄存器的值，范围：0x00~0xFF
  */
uint8_t NRF24L01_ReadStatus(void)
{
	uint8_t Status;
	
	/*CSN置低，通信开始*/
	NRF24L01_W_CSN(0);

	/*交换发送一个字节，通信开始的第一个字节为指令码，空指令*/
	/*第一个字节发送任意指令，都可以交换得到状态寄存器的值*/
	Status = NRF24L01_SPI_SwapByte(NRF24L01_NOP);
	
	/*CSN置高，通信结束*/
	NRF24L01_W_CSN(1);
	
	/*返回状态寄存器的值*/
	return Status;
}

/*********************指令实现*/


/*功能函数*********************/

/**
  * 函    数：NRF24L01进入掉电模式（CE = 0，PWR_UP = 0）
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_PowerDown(void)
{
	uint8_t Config;
	
	/*CE置0，退出收发模式*/
	NRF24L01_W_CE(0);
	
	/*读-改-写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config &= ~0x02;								//配置寄存器位1（PWR_UP）置0
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
}

/**
  * 函    数：NRF24L01进入待机模式1（CE = 0，PWR_UP = 1）
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_StandbyI(void)
{
	uint8_t Config;
	
	/*CE置0，退出收发模式*/
	NRF24L01_W_CE(0);
	
	/*读-改-写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x02;									//配置寄存器位1（PWR_UP）置1
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
}

/**
  * 函    数：NRF24L01进入接收模式（CE = 1，PWR_UP = 1，PRIM_RX = 1）
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_Rx(void)
{
	uint8_t Config;
	
	/*CE置0，退出收发模式*/
	NRF24L01_W_CE(0);
	
	/*读-改-写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x03;									//配置寄存器位1（PWR_UP）和位0（PRIM_RX）都置1
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
	
	/*CE置1，进入收发模式，因为PRIM_RX为1，所以进入接收模式*/
	NRF24L01_W_CE(1);
}

/**
  * 函    数：NRF24L01进入发送模式（CE = 1，PWR_UP = 1，PRIM_RX = 0）
  * 参    数：无
  * 返 回 值：无
  */
void NRF24L01_Tx(void)
{
	uint8_t Config;
	
	/*CE置0，退出收发模式*/
	NRF24L01_W_CE(0);
	
	/*读-改-写操作流程，单独修改配置寄存器的某些位而不影响其他位*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);		//读取配置寄存器
	if (Config == 0xFF) {return;}					//配置寄存器全为1，出错，退出函数
	Config |= 0x02;									//配置寄存器位1（PWR_UP）置1
	Config &= ~0x01;								//配置寄存器位0（PRIM_RX）置0
	NRF24L01_WriteReg(NRF24L01_CONFIG, Config);		//写回配置寄存器
	
	/*CE置1，进入收发模式，因为PRIM_RX为0，所以进入发送模式*/
	NRF24L01_W_CE(1);
}

/**
  * 函    数：NRF24L01初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：使用前，需要调用此初始化函数
  */
void NRF24L01_Init(void)
{
	/*先调用底层的端口初始化*/
	NRF24L01_GPIO_Init();
	
	/*初始化配置一系列寄存器，寄存器值的意义需参考手册中的寄存器描述*/
	/*以下配置通信双方必须保持一致，否则无法进行通信*/
	NRF24L01_WriteReg(NRF24L01_CONFIG, 0x08);		//配置寄存器，不屏蔽中断，使能CRC，CRC为1字节，PWR_UP = 0，PRIM_RX = 0
	NRF24L01_WriteReg(NRF24L01_EN_AA, 0x3F);		//Basic Auto ACK only; ACK payload remains disabled
	NRF24L01_WriteReg(NRF24L01_EN_RXADDR, 0x01);	//使能接收通道，只开启接收通道0
	NRF24L01_WriteReg(NRF24L01_SETUP_AW, 0x03);		//设置地址宽度，地址宽度为5字节
	NRF24L01_WriteReg(NRF24L01_SETUP_RETR, NRF24L01_SETUP_RETR_VALUE);
	NRF24L01_WriteReg(NRF24L01_RF_CH, NRF24L01_RF_CHANNEL_VALUE);
	NRF24L01_WriteReg(NRF24L01_RF_SETUP, NRF24L01_RF_SETUP_VALUE);
	
	/*接收通道0的数据包宽度，设置为宏定义NRF24L01_RX_PACKET_WIDTH指定的值*/
	NRF24L01_WriteReg(NRF24L01_RX_PW_P0, NRF24L01_RX_PACKET_WIDTH);
	
	NRF24L01_WriteRegs(NRF24L01_TX_ADDR, NRF24L01_TxAddress, 5);

	/*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);

#if (NRF24L01_ENABLE_ACK_PAYLOAD != 0)
	NRF24L01_EnableAckPayload();
#else
	NRF24L01_DisableAckPayload();
#endif
	
	/*清空Tx FIFO的所有数据*/
	NRF24L01_FlushTx();
	
	/*清空Rx FIFO的所有数据*/
	NRF24L01_FlushRx();
	
	/*给状态寄存器的位4（MAX_RT）、位5（TX_DS）和位6（RX_DR）写1，清标志位*/
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
	
	/*初始化配置完成，芯片默认进入接收模式*/
	NRF24L01_Rx();
}

/**
  * 函    数：NRF24L01发送数据包
  * 参    数：无
  * 返 回 值：发送标志位，方便用户了解发送状态
  * 			1：发送成功，无错误
  * 			2：达到了最大重发次数仍未收到应答，可能是收发双方配置不一致、接收方不存在、接收FIFO已满或者多个发送数据包碰撞
  * 			3：状态寄存器的值不合法，可能是设备不存在、断路、短路或者引脚配置不正确
  * 			4：发送超时，可能是设备未初始化、断路、短路或者引脚配置不正确
  * 说    明：调用此函数前，直接修改全局数组NRF24L01_TxAddress和NRF24L01_TxPacket来设置发送的地址和数据
  */
uint8_t NRF24L01_Send(void)
{
	uint8_t Status;
	uint8_t SendFlag;
	uint16_t PollCount;
	uint32_t StartTick;

	SendFlag = 4;
	PollCount = 0U;
	NRF24L01_StandbyI();
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
	NRF24L01_FlushTx();

	/*发送地址，设置为全局数组NRF24L01_TxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_TX_ADDR, NRF24L01_TxAddress, 5);
	
	/*接收通道0地址，此处必须也设置为发送地址，用于接收应答*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_TxAddress, 5);
	
	/*写发送有效载荷，写入全局数组NRF24L01_TxPacket指定的数据，数据宽度为NRF24L01_TX_PACKET_WIDTH*/
	NRF24L01_WriteTxPayload(NRF24L01_TxPacket, NRF24L01_TX_PACKET_WIDTH);
	
	/*发送的地址和有效载荷写入完成，进入发送模式，开始发送数据*/
	NRF24L01_Tx();
	
	/*指定超时时间，即循环读取状态寄存器的次数，具体值可以实测确定*/
	StartTick = HAL_GetTick();
	
	/*循环读取状态寄存器*/
	while (1)
	{
		/*读取状态寄存器，保存至Status变量*/
		Status = NRF24L01_ReadStatus();
		
		/*超时计次*/
		/*根据状态寄存器的值，判断发送状态*/
		if ((Status & 0x30) == 0x30)		//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
		{
			SendFlag = 3;			//状态寄存器的值不合法，置标志位为3
			break;					//跳出循环
		}
		else if ((Status & 0x10) == 0x10)	//状态寄存器位4（MAX_RT）为1
		{
			SendFlag = 2;			//达到了最大重发次数仍未收到应答，置标志位为2
			break;					//跳出循环
		}
		else if ((Status & 0x20) == 0x20)	//状态寄存器位5（TX_DS）为1
		{
			SendFlag = 1;			//发送成功，无错误，置标志位为1
			break;					//跳出循环
		}
		else if (((HAL_GetTick() - StartTick) >= NRF24L01_TX_TIMEOUT_MS) ||
		         (++PollCount >= NRF24L01_TX_TIMEOUT_POLLS))
		{
			SendFlag = 4;
			break;
		}
	}
	
	NRF24L01_W_CE(0);

	/*给状态寄存器的位4（MAX_RT）和位5（TX_DS）写1，清标志位*/
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x30);
	
	/*清空Tx FIFO的所有数据*/
	NRF24L01_FlushTx();
	
	/*发送完成后，恢复接收通道0原来的地址*/
	/*如果发送地址和接收通道0地址设置相同，则可不执行这一句*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
	
	/*发送完成，芯片恢复为接收模式*/
	NRF24L01_Rx();
		
	/*返回发送标志位*/
	return SendFlag;
}

/**
  * 函    数：NRF24L01接收数据包
  * 参    数：无
  * 返 回 值：接收标志位，方便用户了解接收状态
  * 			0：未接收到数据包
  * 			1：成功接收到一个数据包
  * 			2：状态寄存器的值不合法，可能是设备不存在、断路、短路或者引脚配置不正确
  * 			3：设备仍处于掉电模式，可能是设备未初始化、曾经断电过、断路、短路或者引脚配置不正确
  * 说    明：如果收到了数据包，则可直接从全局数组NRF24L01_RxPacket取数据
  */
uint8_t NRF24L01_Receive(void)
{
	uint8_t Status, Config;
	uint8_t ReceiveFlag;
	
	/*读取状态寄存器，保存至Status变量*/
	Status = NRF24L01_ReadStatus();
	
	/*读取配置寄存器，保存至Config变量*/
	Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
	
	/*根据配置寄存器和状态寄存器的值，判断接收状态*/
	if ((Config & 0x02) == 0x00)		//配置寄存器位1（PWR_UP）为0
	{
		ReceiveFlag = 3;				//设备仍处于掉电模式，置标志位为3
		NRF24L01_Init();				//接收出错，重新初始化一次设备，这样有助于设备从错误中恢复正常
	}
	else if ((Status & 0x30) == 0x30)	//状态寄存器位4（MAX_RT）和位5（TX_DS）同时为1
	{
		ReceiveFlag = 2;				//状态寄存器的值不合法，置标志位为2
		NRF24L01_Init();				//接收出错，重新初始化一次设备，这样有助于设备从错误中恢复正常
	}
	else if ((Status & 0x40) == 0x40)	//状态寄存器位6（RX_DR）为1
	{
		ReceiveFlag = 1;				//接收到数据，置标志位为1
		
		/*读接收有效载荷，存放在全局数组NRF24L01_RxPacket中，数据宽度为NRF24L01_RX_PACKET_WIDTH*/
		NRF24L01_ReadRxPayload(NRF24L01_RxPacket, NRF24L01_RX_PACKET_WIDTH);
		
		/*给状态寄存器的位6（RX_DR）写1，清标志位*/
		NRF24L01_WriteReg(NRF24L01_STATUS, 0x40);

		/*清空Rx FIFO的所有数据*/
		NRF24L01_FlushRx();
	}
	else
	{
		ReceiveFlag = 0;				//未接收到数据，置标志位为0
	}
	
	/*返回接收标志位*/
	return ReceiveFlag;
}

uint8_t NRF24L01_ReadAckPayload(void)
{
	uint8_t Status;
	uint8_t FifoStatus;

	Status = NRF24L01_ReadStatus();
	FifoStatus = NRF24L01_ReadReg(NRF24L01_FIFO_STATUS);
	if ((FifoStatus & NRF24L01_FIFO_RX_EMPTY) != 0U)
	{
		if ((Status & NRF24L01_STATUS_RX_DR) != 0U)
		{
			NRF24L01_WriteReg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR);
		}
		return 0U;
	}

	/* Read payload directly; dynamic-payload-width check is skipped
	   because some NRF24L01 clones report incorrect width for ACK payloads */
	NRF24L01_ReadRxPayload(NRF24L01_RxPacket, NRF24L01_RX_PACKET_WIDTH);
	NRF24L01_WriteReg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR);

	return 1U;
}

uint8_t NRF24L01_SendPayload(uint8_t *Address, uint8_t *DataArray, uint8_t Count)
{
	uint8_t Status;
	uint8_t SendFlag;
	uint16_t PollCount;
	uint32_t StartTick;

	if ((Address == NULL) || (DataArray == NULL) || (Count == 0U) || (Count > 32U))
	{
		return 4U;
	}

	SendFlag = 4U;
	PollCount = 0U;
	NRF24L01_StandbyI();
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteRegs(NRF24L01_TX_ADDR, Address, 5);
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, Address, 5);
	NRF24L01_WriteTxPayload(DataArray, Count);
	NRF24L01_Tx();
	StartTick = HAL_GetTick();
	while (1)
	{
		Status = NRF24L01_ReadStatus();
		if ((Status & 0x20) == 0x20)
		{
			SendFlag = 1U;
			break;
		}
		else if ((Status & 0x10) == 0x10)
		{
			SendFlag = 2U;
			break;
		}
		else if (((HAL_GetTick() - StartTick) >= NRF24L01_TX_TIMEOUT_MS) ||
		         (++PollCount >= NRF24L01_TX_TIMEOUT_POLLS))
		{
			SendFlag = 4U;
			break;
		}
	}
	NRF24L01_W_CE(0);
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x30);
	NRF24L01_FlushTx();
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
	NRF24L01_Rx();

	return SendFlag;
}

/**
  * 函    数：NRF24L01更新接收地址
  * 参    数：无
  * 返 回 值：无
  * 说    明：如果想在运行时动态修改接收地址，则可先向全局数组NRF24L01_RxAddress写入修改的地址
  * 		  然后再调用此函数，使修改的接收地址生效
  */
void NRF24L01_UpdateRxAddress(void)
{
	/*接收通道0地址，设置为全局数组NRF24L01_RxAddress指定的地址，地址宽度固定为5字节*/
	NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RxAddress, 5);
}

/*********************功能函数*/


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
