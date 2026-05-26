# STM32 麦轮小车自动导航与 NRF 遥控项目

本仓库用于记录 STM32 嵌入式学习与机器人项目。当前最重要、最完整的项目放在仓库根目录最前面的 `00_` 开头目录中：

- `00_Car_Mecanum_Wheels_AutoNavigation/`：车端工程，麦克纳姆轮小车，支持 NRF 遥控、编码器速度闭环、里程计、MPU6050 姿态、OLED 坐标显示、自动定位导航。
- `00_Remote_NRF_Controller/`：遥控端工程，读取双摇杆 ADC，通过 NRF24L01 发送 4 字节遥控数据。
- `00_Firmware_HEX/`：当前已编译通过的 HEX 文件，方便直接烧录测试。

当前版本已完成一次关键实测：在约 3 米自动定位导航测试中，平均误差小于 5 cm。

## 项目目标

这个项目的目标是做一台基于 STM32 的四轮麦克纳姆轮小车，实现：

- NRF24L01 无线遥控。
- 车端 OLED 实时显示当前位置坐标。
- 编码器 + MPU6050 的相对定位里程计。
- 基于当前位置反馈的自动导航。
- 支持手动遥控接管自动导航。
- 保持遥控链路稳定，不再使用 NRF 坐标回传到遥控器 OLED。

## 当前目录结构

```text
STM32-Project/
├── 00_Car_Mecanum_Wheels_AutoNavigation/
│   ├── Core/
│   ├── Drivers/
│   ├── Middlewares/
│   └── MDK-ARM/
│       ├── Mecanum_Wheels_Car(new).uvprojx
│       ├── UserApp/Core/Src/
│       │   ├── app.c
│       │   ├── app_remote.c
│       │   ├── app_nrf_remote.c
│       │   ├── app_nrf24l01.c
│       │   ├── app_odometry.c
│       │   ├── app_navigation.c
│       │   ├── app_path_planner.c
│       │   ├── app_oled.c
│       │   ├── app_mpu6050.c
│       │   ├── app_encoder.c
│       │   └── app_motor.c
│       ├── pid.c
│       └── pid.h
│
├── 00_Remote_NRF_Controller/
│   ├── Core/
│   ├── Drivers/
│   ├── Hardware/
│   ├── System/
│   └── MDK-ARM/
│       └── NRF_Remote.uvprojx
│
└── 00_Firmware_HEX/
    ├── car_mecanum_auto_navigation.hex
    └── remote_nrf_controller.hex
```

## 硬件组成

### 车端

- STM32F407 系列主控。
- 四轮麦克纳姆底盘。
- 4 路直流电机驱动。
- 4 路编码器。
- MPU6050。
- NRF24L01。
- SSD1306 OLED，软件 I2C。
- FreeRTOS。

### 遥控端

- STM32F103C8 系列主控。
- 双摇杆，4 路 ADC。
- NRF24L01。
- 遥控端当前不再使用 OLED 显示坐标，只负责稳定发送遥控数据。

## 关键接线

### 车端 NRF24L01

车端 NRF 引脚在 `00_Car_Mecanum_Wheels_AutoNavigation/MDK-ARM/UserApp/Core/Src/app_nrf24l01.c` 中定义：

```c
#define NRF_CE_Pin        GPIO_PIN_8   // PA8
#define NRF_CE_GPIO_Port  GPIOA
#define NRF_CSN_Pin       GPIO_PIN_8   // PB8
#define NRF_CSN_GPIO_Port GPIOB
#define NRF_IRQ_Pin       GPIO_PIN_9   // PB9
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_SCK_Pin       GPIO_PIN_3   // PB3
#define NRF_MISO_Pin      GPIO_PIN_4   // PB4
#define NRF_MOSI_Pin      GPIO_PIN_5   // PB5
```

推荐接线：

| NRF24L01 | 车端 STM32 |
| --- | --- |
| VCC | 3.3V |
| GND | GND |
| CE | PA8 |
| CSN/CS | PB8 |
| SCK | PB3 |
| MISO | PB4 |
| MOSI | PB5 |
| IRQ | PB9，可接可不接，当前代码主要轮询接收 |

注意：NRF24L01 只能使用 3.3V 供电，不要接 5V。建议在 NRF 模块 VCC/GND 旁边并联 10uF 到 47uF 电容。

### 车端 OLED

车端 OLED 使用软件 I2C，在 `app_oled.c` 中定义：

```c
#define OLED_SCL_PIN        GPIO_PIN_8   // PC8
#define OLED_SCL_GPIO_PORT  GPIOC
#define OLED_SDA_PIN        GPIO_PIN_9   // PC9
#define OLED_SDA_GPIO_PORT  GPIOC
```

推荐接线：

| OLED | 车端 STM32 |
| --- | --- |
| VCC | 3.3V 或模块允许的供电电压 |
| GND | GND |
| SCL | PC8 |
| SDA | PC9 |

OLED 默认地址使用 8 位地址 `0x78`，对应常见 SSD1306 的 7 位地址 `0x3C`。

### 遥控端 NRF24L01

遥控端 NRF 引脚在 `00_Remote_NRF_Controller/Hardware/NRF24L01.c` 中定义：

| NRF24L01 | 遥控端 STM32 |
| --- | --- |
| VCC | 3.3V |
| GND | GND |
| CE | PB11 |
| CSN/CS | PB12 |
| SCK | PB13 |
| MISO | PB14 |
| MOSI | PB15 |

### 遥控端摇杆 ADC

遥控端主循环读取 4 路 ADC：

```c
#define JOY_LV_CHANNEL ADC_CHANNEL_0
#define JOY_LH_CHANNEL ADC_CHANNEL_1
#define JOY_RV_CHANNEL ADC_CHANNEL_2
#define JOY_RH_CHANNEL ADC_CHANNEL_3
```

对应 PA0、PA1、PA2、PA3。

## 编译与烧录

### 车端工程

Keil 工程：

```text
00_Car_Mecanum_Wheels_AutoNavigation/MDK-ARM/Mecanum_Wheels_Car(new).uvprojx
```

当前已编译通过：

```text
0 Error(s), 0 Warning(s)
```

直接烧录 HEX：

```text
00_Firmware_HEX/car_mecanum_auto_navigation.hex
```

### 遥控端工程

Keil 工程：

```text
00_Remote_NRF_Controller/MDK-ARM/NRF_Remote.uvprojx
```

当前已编译通过：

```text
0 Error(s), 0 Warning(s)
```

直接烧录 HEX：

```text
00_Firmware_HEX/remote_nrf_controller.hex
```

## 当前核心功能

### 1. NRF 遥控

遥控端每 10 ms 发送 4 字节摇杆数据：

```c
NRF24L01_TxPacket[0] = left vertical
NRF24L01_TxPacket[1] = left horizontal
NRF24L01_TxPacket[2] = right vertical
NRF24L01_TxPacket[3] = right horizontal
```

车端只接收遥控数据，不再通过 NRF 把坐标回传到遥控端 OLED。

当前 NRF 策略：

- 保留普通 Auto ACK 和短重发，提高遥控稳定性。
- 关闭 ACK Payload 和 DPL，避免双向通信导致遥控卡顿。
- 固定 4 字节遥控包。

### 2. OLED 坐标显示

车端 OLED 显示：

```text
C:1 R:1
X: ...
Y: ...
A: ...
```

含义：

- `C:1`：车端 NRF 初始化成功。
- `C:0`：车端 NRF 初始化失败，优先检查 NRF 接线和供电。
- `R:1`：最近 500 ms 内收到遥控包。
- `R:0`：没有收到遥控包。
- `X`：当前相对 X 坐标，单位 mm。
- `Y`：当前相对 Y 坐标，单位 mm。
- `A`：当前相对 yaw 角，单位 degree。

### 3. 遥控防抖和空闲保护

为了解决摇杆不动时轮子偶发抽动的问题，车端遥控逻辑加入了：

- 平移死区。
- 右摇杆 yaw 单轴死区。
- 平移命令连续 3 个新 NRF 包确认后才输出。
- yaw 命令连续 3 个新 NRF 包确认后才输出。

这样单个异常包不会立刻驱动车轮。

### 4. 里程计

车端里程计位于：

```text
00_Car_Mecanum_Wheels_AutoNavigation/MDK-ARM/UserApp/Core/Src/app_odometry.c
```

当前关键参数：

```c
#define ODOM_WHEEL_DIAMETER_MM        65.0f
#define ODOM_ENCODER_COUNTS_PER_REV   4320.0f
#define ODOM_FORWARD_SCALE            0.962f
#define ODOM_STRAFE_SCALE             0.960f
```

当前里程计使用：

- 四轮编码器距离。
- MPU6050 yaw。
- 前进比例标定。
- 侧移比例标定。

注意：这是相对定位，不是绝对定位。断电后坐标从当前启动点重新开始。

### 5. 自动导航

自动导航入口在：

```text
00_Car_Mecanum_Wheels_AutoNavigation/MDK-ARM/UserApp/Core/Src/app.c
```

当前默认配置：

```c
#define APP_AUTO_NAV_ENABLE       1U
#define APP_AUTO_NAV_TARGET_X_MM  10000.0f
#define APP_AUTO_NAV_TARGET_Y_MM  0.0f
#define APP_AUTO_NAV_SPEED_MM_S   300.0f
```

含义：

- 上电后自动导航使能。
- 目标点为 `X = 10000 mm, Y = 0 mm`。
- 最大目标速度为 `300 mm/s`。

如果只想手动遥控，把它改成：

```c
#define APP_AUTO_NAV_ENABLE       0U
```

如果想测试 3 米：

```c
#define APP_AUTO_NAV_TARGET_X_MM  3000.0f
#define APP_AUTO_NAV_TARGET_Y_MM  0.0f
```

如果想测试 1 米：

```c
#define APP_AUTO_NAV_TARGET_X_MM  1000.0f
#define APP_AUTO_NAV_TARGET_Y_MM  0.0f
```

上电自动导航期间，只要遥控器有明显手动输入，车端会停止自动导航并切回手动遥控。

## 标定方法

### 前进 X 方向标定

1. 关闭自动导航，使用遥控让小车直行。
2. 实际测量走过的距离，例如 800 mm。
3. 读取 OLED 上的 X 坐标，例如 742 mm。
4. 按公式修正：

```text
新 ODOM_FORWARD_SCALE = 当前 ODOM_FORWARD_SCALE * 实际距离 / OLED显示距离
```

例如：

```text
当前比例 = 0.910
实际距离 = 800
OLED显示 = 756.5
新比例 = 0.910 * 800 / 756.5 ≈ 0.962
```

### 侧移 Y 方向标定

1. 关闭自动导航，使用遥控让小车侧移。
2. 实际测量侧移距离，例如 800 mm。
3. 读取 OLED 上的 Y 坐标，取绝对值，例如 855 mm。
4. 按公式修正：

```text
新 ODOM_STRAFE_SCALE = 当前 ODOM_STRAFE_SCALE * 实际距离 / abs(OLED显示距离)
```

如果 OLED 的 Y 是负值，不一定是错误，只代表当前坐标系把这个侧移方向定义为负方向。标定比例时先取绝对值。

### 建议标定顺序

1. 前进 800 mm 或 1000 mm，测 3 次取平均。
2. 后退 800 mm 或 1000 mm，确认符号和比例。
3. 左右侧移各测 3 次。
4. 原地旋转 90 度、180 度，检查 yaw。
5. 最后再打开自动导航测试 1000 mm、3000 mm、5000 mm。

不要把车架空后用 OLED 坐标标定距离。架空时轮子没有真实地面接触，无法反映打滑、负载、轮胎变形和起停惯性。

## 长距离导航说明

当前软件上可以给定几十米目标，例如：

```c
#define APP_AUTO_NAV_TARGET_X_MM  30000.0f
#define APP_AUTO_NAV_TARGET_Y_MM  0.0f
```

但是当前定位方式是轮式里程计 + MPU6050，属于航位推算。距离越长，误差会累积。

经验判断：

- 1 m 到 3 m：经过标定后可以做到比较好的效果。
- 5 m 到 10 m：需要更认真地控制速度、地面、起停和 yaw 漂移。
- 几十米：只靠当前传感器无法保证高精度，建议加入绝对位置修正。

进一步提升精度的方向：

- 更细致的轮径、轮距、四轮独立增益标定。
- 降低速度和加速度，减少麦轮打滑。
- 分段导航，不一次性跑完整几十米。
- 加 AprilTag、UWB、视觉或激光雷达等绝对位置参考。

## 常见问题

### OLED 显示 `C:0 R:0`

车端 NRF 初始化失败。检查：

- NRF 是否使用 3.3V。
- CE、CSN、SCK、MISO、MOSI 是否接反。
- GND 是否共地。
- NRF 旁边是否有电容。

### OLED 显示 `C:1 R:0`

车端 NRF 初始化成功，但没收到遥控包。检查：

- 遥控端是否烧录了最新 HEX。
- 遥控端 NRF 接线是否正确。
- 两端地址、通道、速率是否保持一致。
- 两个 NRF 模块是否距离过近或供电不稳。

### 不拨摇杆轮子偶发动一下

当前版本已经加入新包确认和右摇杆独立死区。如果仍然出现：

- 检查摇杆中位 ADC 是否漂移太大。
- 适当增大 `RC_MOVE_DEADZONE`。
- 适当增大 `RC_YAW_AXIS_DEADZONE`。
- 检查遥控端摇杆供电是否稳定。

### 自动导航上电就跑

这是因为：

```c
#define APP_AUTO_NAV_ENABLE 1U
```

需要纯手动遥控时，改成：

```c
#define APP_AUTO_NAV_ENABLE 0U
```

### 为什么不再让遥控器 OLED 显示坐标

之前尝试过使用 NRF ACK Payload 或双向通信把坐标回传到遥控器 OLED，但实测会和实时遥控争用 NRF 时序，导致小车走走停停或遥控不灵敏。

当前版本选择更稳妥的架构：

- NRF 只做遥控。
- 坐标显示放在车端 OLED。
- 自动导航和里程计都在车端完成。

这样遥控链路更稳定，也更容易排查问题。

## 当前版本状态

- 车端工程：Keil ARMCC 编译通过，0 error / 0 warning。
- 遥控端工程：Keil ARMCC 编译通过，0 error / 0 warning。
- 车端 OLED 坐标显示正常。
- NRF 遥控恢复正常。
- 空闲轮子误动问题已处理。
- 3 米自动定位导航实测平均误差小于 5 cm。

## 后续计划

- 继续完善前进、侧移、旋转标定。
- 增加分段导航任务列表。
- 增加更稳定的加减速规划。
- 后续考虑加入 AprilTag 或 UWB 做绝对位置修正，解决长距离累计误差问题。
