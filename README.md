<div align="center">

# 2026 方舟计划竞技机器人

**STM32F407 车端 + STM32F103 NRF 遥控端双端固件**

![MCU](https://img.shields.io/badge/Car-STM32F407ZGT6-2f80ed)
![Remote](https://img.shields.io/badge/Remote-STM32F103C8T6-16a085)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-22a06b)
![Link](https://img.shields.io/badge/Link-NRF24L01-orange)
![Branch](https://img.shields.io/badge/Branch-fangzhou--competition--2026-lightgrey)

</div>

本分支保存方舟计划竞技机器人的配套车端和遥控端工程。仓库 `main` 分支中的自动导航项目保持不变，本项目只发布在独立分支 `fangzhou-competition-2026`。

## 工程入口

| 工程 | 平台 | 说明 |
| --- | --- | --- |
| [chassis_f407](01_FangZhou_Competition_2026/chassis_f407) | STM32F407ZGT6，168 MHz | 四轮麦克纳姆底盘、NRF 接收、步进电机、舵机、OLED、视觉串口和 FreeRTOS |
| [remote_f103](01_FangZhou_Competition_2026/remote_f103) | STM32F103C8T6，72 MHz | 四轴摇杆采集、按键消抖和 NRF 遥控数据发送 |
| [详细项目文档](01_FangZhou_Competition_2026/README.md) | README | 完整引脚表、通信协议、任务优先级、接线和编译说明 |

## 主要功能

- NRF24L01 遥控四轮麦克纳姆底盘，包含摇杆中心校准、死区和输出平滑。
- 固定 5 字节数据包传输 4 路摇杆和按键状态。
- 车端 300 ms 失联保护，超时后停止底盘和上层机构输出。
- K2/K6 控制步进电机带动丝杆上升和下降。
- K4/K7 控制舵机到 7.5% 和 2.5% 占空比位置。
- OLED 显示启动状态和视觉识别颜色结果。
- USART3 接收视觉端 `QR`/`CLR` 文本协议。
- FreeRTOS 分离遥控、上层机构和显示任务。

## NRF 配对参数

| 参数 | 数值 |
| --- | --- |
| 地址 | `11 22 33 44 55` |
| 频道 `RF_CH` | `0x4C` |
| 射频配置 `RF_SETUP` | `0x27` |
| 自动重传 `SETUP_RETR` | `0x15` |
| 包长 | 5 字节 |
| 数据布局 | `[LV, LH, RV, RH, buttons]` |

## 遥控按键

| 按键 | 遥控端引脚 | 数据位 | 车端动作 |
| --- | --- | --- | --- |
| K2 | PA7 | `0x01` | 步进电机正转，丝杆上升 |
| K6 | PB10 | `0x02` | 步进电机反转，丝杆下降 |
| K4 | PB1 | `0x04` | 舵机输出 7.5% 占空比 |
| K7 | PA11 | `0x08` | 舵机输出 2.5% 占空比 |

## 车端接口摘要

| 模块 | 引脚 |
| --- | --- |
| 电机 PWM | PC6、PC7、PD12、PD13 |
| NRF24L01 | PA8、PA9、PA10、PB13、PB14 |
| OLED | PB3、PB5 |
| 步进驱动 PUL/DIR/ENA | PC8、PF14、PF15 |
| 舵机 PWM | PC9 |
| 视觉 USART3 TX/RX | PB10、PB11 |

## FreeRTOS 任务

| 任务 | 优先级 | 职责 |
| --- | --- | --- |
| `RemoteTask` | `idle + 3` | 2 ms 接收遥控并更新底盘 |
| `UpperTask` | `idle + 2` | 5 ms 处理步进电机和舵机 |
| `DisplayTask` | `idle + 1` | 刷新 OLED 和状态 LED |

## 编译

使用 Keil MDK 5：

- 车端：[fangzhou_competition_alternative.uvprojx](01_FangZhou_Competition_2026/chassis_f407/MDK-ARM/fangzhou_competition_alternative.uvprojx)
- 遥控端：[NRF_Remote.uvprojx](01_FangZhou_Competition_2026/remote_f103/MDK-ARM/NRF_Remote.uvprojx)

两个工程均已使用 ARM Compiler 5.06 执行完整 Rebuild，结果为 `0 Error(s), 0 Warning(s)`。

仓库不提交 `Objects`、`Listings`、`AXF`、`HEX`、编译日志和个人 Keil 配置。烧录前应从当前源码重新生成固件，避免源码与烧录文件不一致。
