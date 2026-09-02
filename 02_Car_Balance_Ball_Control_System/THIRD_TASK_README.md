# 第三题球杆控制：接线、协议与调试顺序

## 当前状态

`Core/Inc/ball_config.h` 中 `BALL_CONTROL_MODE` 默认是
`BALL_MODE_SENSOR_MONITOR`。该模式会强制失能步进驱动，只采集传感器，
因此首次烧录不会让摆杆突然运动。第二题循迹代码和原参数均保留；
`Core/Inc/app_config.h` 中只用 `APP_ACTIVE_TASK` 选择运行第二题或第三题。

## 接线

- WDD35D4 角位移传感器：1、3 端接 3.3 V/GND，2 端（滑动端）接 PB0。
  若转角方向反了，交换 1、3 端或修改 `BALL_ANGLE_SENSOR_SIGN`，二选一。
- D36A：PC6 -> STEP，PC8 -> DIR，PB14 -> EN，STM32 GND 与驱动器信号地共地。
  电机功率由驱动器外部电源提供，不从开发板取电。
- MaixCAM：队友代码所用 `/dev/ttyS1` 对应的 UART1_TX -> STM32
  PD2/UART5_RX，并共地。只使用 3.3 V TTL。
- VOFA+ 仍接 USART3（PB10/PB11），OLED 仍接 I2C1（PB6/PB7）。

## MaixCAM 到 STM32 的字符串协议

队友端发送：

```python
serial = uart.UART('/dev/ttyS1', 115200)
serial.write(f"{pos_mm}\r\n".encode())
```

STM32逐行接收普通ASCII浮点数，例如`0.0\r\n`、`48.25\r\n`或
`-50.0\r\n`。`pos_mm`必须以毫米为单位、管道中心为0，并带有正确正负方向。
数值必须位于-125～+125mm。连续150ms收不到合法数字时，STM32会失能
步进驱动并进入错误态。该协议没有识别有效位，因此视觉丢球时应停止发送，
不能继续发送旧位置，也不要用0冒充丢球。

MaixCAM 发送的是标定后的带符号位置，而不是针对某一个固定目标的误差。
因为第三题目标会由 +50 mm 切换到 -50 mm，目标状态放在 STM32 中更容易保证
时序一致；STM32 内部仍按 `目标位置 - 实际位置` 计算位置误差。

## 必须按顺序调试

1. `BALL_MODE_SENSOR_MONITOR`
   - 摆杆放水平，在 Keil Watch 观察 `g_ball_debug.angle_adc_raw`，把稳定值写入
     `BALL_ANGLE_ADC_ZERO`。
   - 在 MaixCAM 的 Find Blobs 工具中取得球的 LAB 阈值，修改
     `MaixCAM/maixcam_ball.py` 的 `BALL_LAB` 与 `ROI`。
   - 球分别放在 -5 cm、+5 cm 刻度，填写 `POINT_NEG_5CM`、`POINT_POS_5CM`。
   - Watch 中 `g_ball_debug.ball_position_mm` 应分别接近 -50、+50，
     `vision_valid` 应持续为 1，`vision_parse_errors`应基本不增长。
2. `BALL_MODE_STEPPER_DIRECTION_TEST`
   - 上电 3 s 后只以 +200 Hz 运行 250 ms。
   - 正指令必须使 `angle_feedback_adc` 增大；否则只翻转
     `BALL_STEPPER_POSITIVE_DIR_LEVEL`。
3. `BALL_MODE_ANGLE_LOOP`
   - 上电 3 s 后只运行角度内环，目标为水平。
   - 先只调 `g_ball_angle_kp`；摆杆应快速回水平且不持续振荡。
4. `BALL_MODE_CASCADE`
   - 有持续有效视觉帧后自动使能，先水平稳定 0.5 s，再执行
     `0 -> +50 mm -> -50 mm`，最终保持在 -50 mm。
   - 先调位置 `Kp`，再加 `Kd` 抑制往复振荡；`Ki` 初期保持 0。

## VOFA+ 通道

- ch0：位置目标（mm）
- ch1：小球实际位置（mm）
- ch2：角度目标（ADC 相对量）
- ch3：角度实际值（ADC 相对量）

Keil Watch 可直接调整六个 `volatile float`：
`g_ball_position_kp/ki/kd`、`g_ball_angle_kp/ki/kd`。

代码没有加入 `HAL_Delay` 或毫秒级忙等待。ADC2 每个控制周期进行 8 次快速
硬件转换并平均；STEP 脉冲由 TIM3 中断产生，控制循环不会阻塞脉冲输出。
