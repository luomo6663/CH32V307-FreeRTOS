# CH32V307 FreeRTOS 机器人固件 v1.0.0

CH32V307VCT6 四轮麦轮底盘、舵机和单轴步进转台的正式版固件。工程使用
FreeRTOS 和 WCH RISC-V 工具链，正常上电后电机保持滑行、舵机输出 1500 us
中位、转台停机，等待 USART3 命令。

## 功能

- 四路 TMI8140 H 桥电机驱动，支持双 PWM 或 PWM + DIR 接法
- 四路正交编码器、独立 PI(D) 速度环、堵转和反馈反接保护
- 麦轮正逆运动学、比例限速、里程计位姿积分
- 100 Hz 固定周期控制任务，独立控制台和遥测任务
- TBS-K20 舵机控制，支持固定脉宽和手动扫描
- STEP/DIR 步进转台，支持角度、连续旋转、线性加减速和位置清零
- CSV 或 VOFA+ FireWater 遥测
- 保留接线标定和台架维护命令，不包含一次性排线与引脚探测固件

## 硬件

| 项目 | 连接 |
| --- | --- |
| 主控 | CH32V307VCT6，96 MHz |
| 控制台 | USART3，PB10 TX / PB11 RX，115200-8-N-1 |
| 电机 | W1 PA2/PA3，W2 PA15/PB3，W3 PD12/PD13，W4 PD14/PD15 |
| 编码器 | W1 PC6/PC7，W2 PD1/PD3，W3 PA8/PA9，W4 PD9/PD11 |
| 舵机 | PB5，TIM3_CH2，50 Hz |
| 转台 | PA0 STEP/TIM5_CH1，PA1 DIR |

电机、舵机和步进电机必须使用各自匹配的电源，并与控制板共地。不要让电机或
舵机负载从 MCU 的 3.3 V 电源取电。

## 安全启动

- 四路电机上电滑行，不自动转动。
- 舵机初始化后固定输出 1500 us 中位，不自动扫描。
- 步进转台保持停止，STEP 引脚停在低电平。
- `SERVO SWEEP` 只用于显式请求的手动检查，正式使用时应避免舵机带负载扫动。
- DRIVE 模式命令超时、车轮缺反馈、堵转或反馈方向错误时自动停机并锁存故障。

运行期的 PID、方向、编码器符号和转台 PPR 修改只保存在 RAM，复位后恢复
`User/config/robot_config.h` 中的默认值。

## 构建

MounRiver Studio 可以直接导入工程并使用 `obj/` 构建目录。命令行构建不修改
MounRiver 工程，产物写入 `build/`：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1
```

如果工具链不在默认路径，可以显式指定：

```powershell
.\tools\build.ps1 -ToolchainDir "C:\path\to\toolchain\bin"
```

成功后生成：

```text
build/CH32V307-FreeRTOS.elf
build/CH32V307-FreeRTOS.hex
build/CH32V307-FreeRTOS.map
```

当前 v1.0.0 命令行构建体积：

```text
text   data    bss    dec
56732    600  15848  73180
```

## 烧录

使用 MounRiver Studio 或 WCH-Link 工具烧录
`build/CH32V307-FreeRTOS.hex`。烧录后连接 USART3，115200 波特率，发送
`H` 查看命令台帮助。默认设置使用 256 K Flash / 32 K SRAM 下限。

## 指令表

命令不区分大小写，参数使用空格分隔。底盘坐标系为：

- `vx > 0` 向前，`vx < 0` 向后
- `vy > 0` 向左，`vy < 0` 向右
- `wz > 0` 左转，`wz < 0` 右转
- `V` 指令必须在 1 秒内重复发送，否则底盘会自动停车

### 小车运动

| 动作 | 指令 | 说明 |
| --- | --- | --- |
| 前进 | `V 150 0 0` | 以 150 mm/s 向前 |
| 后退 | `V -150 0 0` | 以 150 mm/s 向后 |
| 左移 | `V 0 150 0` | 向左横移 150 mm/s |
| 右移 | `V 0 -150 0` | 向右横移 150 mm/s |
| 原地左转 | `V 0 0 1.0` | 左转 1.0 rad/s |
| 原地右转 | `V 0 0 -1.0` | 右转 1.0 rad/s |
| 左前斜行 | `V 100 100 0` | 向左前方运动 |
| 右前斜行 | `V 100 -100 0` | 向右前方运动 |
| 左后斜行 | `V -100 100 0` | 向左后方运动 |
| 右后斜行 | `V -100 -100 0` | 向右后方运动 |
| 边走边转 | `V 100 0 0.5` | 前进同时左转 |
| 停车 | `S` / `STOP` / `X` | 目标清零并让电机滑行 |
| 刹车 | `B` | 四轮主动刹车 |
| 状态 | `P` / `STATUS` | 查看模式、故障、RPM、目标和占空比 |
| 帮助 | `H` / `?` / `HELP` | 打印控制台帮助 |

### 标定与维护

| 指令 | 说明 |
| --- | --- |
| `W <n> <duty>` | 单轮开环，`n=1..4`，`duty=-1000..1000` |
| `M <d1> <d2> <d3> <d4>` | 四轮分别开环控制 |
| `SPD <n\|ALL> <rpm>` | 单轮或四轮闭环速度标定 |
| `PID <n\|ALL> [kp ki kd]` | 查询或设置速度环增益 |
| `OL <n> <gain>` | 设置缺反馈轮的开环占空比/转速比 |
| `RAMP <n>` | 每个控制周期的占空比变化上限，`0` 为关闭 |
| `INV <n> <0\|1>` | 电机方向标定 |
| `MODE <0\|1>` | `0` 为双 PWM，`1` 为 PWM + DIR |
| `E` | 查看四路编码器计数、RPM 和电平变化 |
| `E R` | 清零编码器计数、诊断标志和里程计 |
| `SIGN <n> <1\|-1>` | 编码器方向标定 |
| `POSE` / `POSE R` | 查看或清零里程计位姿 |
| `CLR` | 清除锁存故障 |
| `CFG` | 查看时钟、PWM、控制周期和限幅 |
| `TELEM <on\|off\|ms\|csv>` | 控制 CSV 遥测 |
| `VOFA <n\|ALL\|OFF> [ms]` | 输出 VOFA+ FireWater 调参数据 |

### 舵机与转台

| 指令 | 说明 |
| --- | --- |
| `SERVO` | 查看舵机脉宽、扫描状态和定时器寄存器 |
| `SERVO <500..2500>` | 设置舵机脉宽，`1500` 为中位 |
| `SERVO SWEEP [ms]` | 手动启动 1000–2000 us 扫描 |
| `SERVO HOLD` | 停止扫描并保持当前脉宽 |
| `TURN` | 查看转台位置、目标、方向和速率 |
| `TURN <deg> [rpm]` | 转台相对移动指定角度 |
| `TURN RUN <CW\|CCW> [rpm]` | 转台连续旋转 |
| `TURN STOP` | 停止转台并拉低 STEP |
| `TURN ZERO` | 把当前位置设为零 |
| `TURN PPR <n>` | 设置转台每圈脉冲数 |
| `TURN PULSES <n> [hz]` | 按指定频率直接发送脉冲 |

完整硬件说明、参数和安全设计见
[docs/MOTOR_CONTROL.md](docs/MOTOR_CONTROL.md)。

## 工程结构

```text
User/main.c              启动入口
User/config/             引脚、机械、PID 和安全参数
User/control/            电机、编码器、麦轮、底盘、舵机、步进
User/app/                任务、控制台、通信、遥测和日志
tools/build.ps1          命令行构建
tools/step_move.ps1      转台串口移动示例
docs/                    架构、底盘、PID 和转台文档
```

## 文档

- [固件架构](docs/ARCHITECTURE.md)
- [底盘、硬件和命令](docs/MOTOR_CONTROL.md)
- [VOFA+ PID 调试](docs/VOFA_PID.md)
- [步进转台](docs/STEPPER_TURNTABLE.md)

## 上板验收

1. 上电后电机不自走，舵机停在 1500 us，转台不转。
2. 启动横幅显示 `v1.0.0`，`H` 能打印完整命令帮助。
3. 架空车轮执行 `V`、`S`、`W`、`SPD` 和 `E`，确认方向、反馈和停车正常。
4. 执行 `TURN 90`、`TURN STOP`，确认转台方向、角度和安全停止正常。
5. 执行 `SERVO 1500` 和 `SERVO SWEEP`，确认舵机中位与手动扫描正常。

## 已知限制

- 里程计轮径和轮距尚未实测标定，长距离行驶会有累积误差。
- 转台位置为开环计数，没有原点开关，断电后位置清零。
- 转台使用线性加减速；重载时可降低 `ROBOT_STEPPER_ACCEL_HZ_PER_S` 和
  `ROBOT_STEPPER_DECEL_HZ_PER_S`。
- 底盘低速转向可能在地面静摩擦阶段触发 0.5 s 堵转保护；实测
  `V 0 0 1.0` 正常，更低速度需要重新评估堵转阈值或降低机械阻力。
- 遥测与控制台共用 USART3，开启高速遥测后命令响应速度会下降。
