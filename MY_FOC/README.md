# MY_FOC 工程入口

完整项目介绍、控制框图、算法公式、默认参数、编译方法和验证边界见[分支首页README](../README.md)。

- Keil工程：[`MDK-ARM/XC5000_FOC.uvprojx`](MDK-ARM/XC5000_FOC.uvprojx)
- FOC、速度观测器及多圈位置环：[`Core/Src/foc.c`](Core/Src/foc.c)
- 对齐与Flash状态机：[`Core/Src/motor1_drive.c`](Core/Src/motor1_drive.c)
- 硬件、故障与调试：[bringup.md](docs/bringup.md)
- 来源与许可：[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)

默认配置会请求相对8圈位置运动。移植前先阅读启动模式和板级参数说明。
