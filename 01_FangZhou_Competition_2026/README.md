# 2026 方舟计划竞技机器人双端固件

本目录保存一套配套使用的 STM32 遥控底盘工程，包括 F407 车端和 F103 遥控端。代码来自本地 `finished` 工程快照，上传时未改动控制逻辑和方向参数。

> 本项目位于独立分支 `fangzhou-competition-2026`。仓库 `main` 分支中的自动导航项目保持不变。

## 目录结构

| 路径 | 平台 | 用途 |
| --- | --- | --- |
| `chassis_f407/` | STM32F407ZGT6，168 MHz | 四轮麦克纳姆底盘、NRF 接收、步进电机、舵机、OLED、视觉串口和 FreeRTOS |
| `remote_f103/` | STM32F103C8T6，72 MHz | 四轴摇杆采集、按键消抖和 NRF 遥控数据发送 |

两个目录都保留了 STM32CubeMX `.ioc` 文件、HAL/CMSIS 依赖和 Keil 工程文件，可以分别独立打开和编译。

## 主要功能

- 四轮麦克纳姆底盘遥控，带摇杆中心校准、死区和输出平滑。
- NRF24L01 固定 5 字节数据包，包含 4 路摇杆和 1 字节按键状态。
- 遥控失联后车端停止输出，超时时间为 300 ms。
- K2/K6 控制步进电机上升/下降，未按键时停止脉冲。
- K4/K7 控制舵机到 7.5%/2.5% 占空比位置。
- OLED 显示启动和视觉识别颜色信息。
- USART3 接收视觉端 `QR`/`CLR` 文本协议。
- 车端使用 FreeRTOS，遥控、上层机构、显示任务优先级依次降低。

## NRF 配对参数

两端必须保持以下参数一致：

| 参数 | 数值 |
| --- | --- |
| 地址 | `11 22 33 44 55` |
| 频道 `RF_CH` | `0x4C` |
| 射频配置 `RF_SETUP` | `0x27` |
| 自动重传 `SETUP_RETR` | `0x15` |
| 固定包长 | 5 字节 |
| 数据布局 | `[LV, LH, RV, RH, buttons]` |

## 遥控按键

| 按键 | 遥控端引脚 | 数据位 | 车端动作 |
| --- | --- | --- | --- |
| K2 | PA7 | `0x01` | 步进电机正转，丝杆上升 |
| K6 | PB10 | `0x02` | 步进电机反转，丝杆下降 |
| K4 | PB1 | `0x04` | 舵机 7.5% 占空比 |
| K7 | PA11 | `0x08` | 舵机 2.5% 占空比 |

冲突按键同时按下时，对应功能会被清零。遥控端和车端均保留按键稳定处理。

## 车端主要引脚

| 模块 | 信号 | STM32 引脚 |
| --- | --- | --- |
| 电机 1 | PWM / IN1 / IN2 | PC6 / PF10 / PE6 |
| 电机 2 | PWM / IN1 / IN2 | PC7 / PG0 / PG1 |
| 电机 3 | PWM / IN1 / IN2 | PD12 / PF2 / PE5 |
| 电机 4 | PWM / IN1 / IN2 | PD13 / PG2 / PG3 |
| NRF24L01 | CE / CSN / SCK / MISO / MOSI | PA8 / PA9 / PA10 / PB13 / PB14 |
| OLED | SCL / SDA | PB3 / PB5 |
| 步进驱动 | PUL / DIR / ENA | PC8 / PF14 / PF15 |
| 舵机 | PWM | PC9 |
| 视觉串口 | USART3 TX / RX | PB10 / PB11 |

车端 NRF 采用软件 SPI。OLED 采用软件 I2C，地址为 `0x78`。

## 遥控端 NRF 引脚

| 信号 | STM32F103 引脚 |
| --- | --- |
| CE | PB11 |
| CSN | PB12 |
| SCK | PB13 |
| MISO | PB14 |
| MOSI | PB15 |

## FreeRTOS 任务

| 任务 | 相对优先级 | 周期/职责 |
| --- | --- | --- |
| `RemoteTask` | `idle + 3` | 2 ms，接收遥控并更新底盘 |
| `UpperTask` | `idle + 2` | 5 ms，处理步进电机和舵机 |
| `DisplayTask` | `idle + 1` | 200 ms 刷新 OLED，500 ms 翻转状态 LED |

## 视觉串口

- 串口：USART3，PB10/PB11
- 参数：115200 baud，8N1
- 支持帧：`QR|...\n` 和 `CLR|...\n`
- USB 转 TTL：TTL TX 接 PB11，TTL RX 接 PB10，GND 共地
- 使用 3.3 V TTL 电平，避免 5 V TX 或在主控断电时从串口反向供电

## 编译

### 车端

1. 使用 Keil MDK 打开 `chassis_f407/MDK-ARM/fangzhou_competition_alternative.uvprojx`。
2. 选择工程目标并执行 Rebuild。
3. 下载到 STM32F407ZGT6。

### 遥控端

1. 使用 Keil MDK 打开 `remote_f103/MDK-ARM/NRF_Remote.uvprojx`。
2. 选择工程目标并执行 Rebuild。
3. 下载到 STM32F103C8T6。

仓库不提交 `Objects`、`Listings`、`AXF`、`HEX`、编译日志及个人 Keil 配置。烧录前请从当前源码重新生成固件，确保烧录文件与源码一致。

## 上电注意事项

- 主控、NRF、OLED、USB-TTL 和各驱动板必须可靠共地。
- 电机电源和逻辑电源按驱动板要求接线，不要用 GPIO 给驱动板供电。
- 主控断电时，不要让外部模块继续向 STM32 信号引脚输出高电平。
- 首次调试建议断开电机负载，先确认 NRF 数据、OLED 状态和各路 PWM/方向信号。
