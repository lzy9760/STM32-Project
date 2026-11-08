# Third-party notices

本工程保留源码已有版权和许可声明，未将全部文件重新标记为MIT，也未把参考算法声明为本项目原创。

| 组件 | 来源及现有声明 | 本仓库位置 |
|---|---|---|
| 编码器对齐检测适配 | Benjamin Vedder / VESC；相关适配代码声明为GPL-3.0-or-later | `Core/Src/motor1_drive.c`文件头；`telemetry/vesc_encoder_reference_20261107/COPYING.txt` |
| 对齐参考版本 | VESC commit `2392b4b8483adecc81c87ccabcaca968efc50d60`，`mcpwm_foc_encoder_detect()` | [原始源码](https://github.com/vedderb/bldc/blob/2392b4b8483adecc81c87ccabcaca968efc50d60/motor/mcpwm_foc.c) |
| TI版权声明 | `motor1_drive.c`及相关原有文件中保留Texas Instruments声明；具体条件见各文件头 | `Core/Src/motor1_drive.c`及原有头文件 |
| 速度观测器设计参考 | Microchip MCAF公开的角度跟踪环方程；本项目为独立离散适配实现，并非打包Microchip SDK | [MCAF Tracking loop](https://microchiptech.github.io/mcaf-doc/6.0.2/algorithms/qei.html#tracking-loop)；`Core/Src/foc.c`注释 |
| STM32 HAL | STMicroelectronics；保留原文件头及附带许可 | `Drivers/STM32F4xx_HAL_Driver/LICENSE.txt` |
| CMSIS | Arm及相关贡献者；保留原文件头和许可 | `Drivers/CMSIS/LICENSE.txt` |
| STM32F4器件头文件 | STMicroelectronics；保留原文件头和许可 | `Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt` |
| USB Device Library | STMicroelectronics；保留原文件头及附带许可 | `USB_Device/STM32_USB_Device_Library/LICENSE.txt` |

本项目的控制结构、离散实现、MCU调度适配、板级保护、参数组织和诊断接口，应与上述通用算法原理、第三方源代码及厂商库分别理解。再分发时请阅读适用文件的原始许可文本。

没有单独许可声明的项目自有文件，本次发布没有为其另行指定统一许可。VESC衍生模块的许可声明和GPL文本不因这一说明而改变。
