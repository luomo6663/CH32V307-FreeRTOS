# 步进转台说明

当前版本：`v1.0.0`

本文档说明单轴 STEP/DIR 步进转台的接线、参数、控制命令和故障排查。

## 1. 接线

| MCU 引脚 | 信号 | 定时器/功能 |
| --- | --- | --- |
| `PA0` | `STEP` | `TIM5_CH1` |
| `PA1` | `DIR` | 普通 GPIO |

驱动器的逻辑地必须与 MCU 地连接。电机应由独立电源供电，不能使用 MCU 的
3.3 V 电源驱动步进电机。

固件当前不使用 `ENABLE` 引脚。驱动器的使能输入应由外部硬件固定在允许运行
的状态。

## 2. 参数配置

在 `User/config/robot_config.h` 中设置：

```c
#define ROBOT_STEPPER_STEPS_PER_REV     3200U
#define ROBOT_STEPPER_DEFAULT_RPM       5.0f
#define ROBOT_STEPPER_MAX_RPM           30.0f
#define ROBOT_STEPPER_DIR_INVERT        0U
#define ROBOT_STEPPER_START_RATE_HZ     20U
#define ROBOT_STEPPER_ACCEL_HZ_PER_S    500U
#define ROBOT_STEPPER_DECEL_HZ_PER_S    500U
```

`ROBOT_STEPPER_STEPS_PER_REV` 是转台旋转一圈所需的驱动脉冲数，已经包含
驱动细分和机械减速比：

```text
每圈脉冲数 = 电机整步数 * 驱动器细分 * 转台减速比
```

示例：

```text
200 整步/圈 * 16 细分 * 1 = 3200 脉冲/圈
200 整步/圈 * 16 细分 * 4 = 12800 脉冲/圈
```

也可以在运行时修改：

```text
TURN PPR 12800
```

## 3. TMC2209 和 STH-42C2009-01

`STH-42C2009-01` 的步距角为 0.9 度，因此电机每圈有 400 个整步。

| TMC2209 细分 | 电机每圈脉冲数 |
| --- | --- |
| 1/8 | 3200 |
| 1/16 | 6400 |

固件默认值 `3200` 对应 1/8 细分。若 TMC2209 的 MS1/MS2 设置为 1/16，
应执行：

```text
TURN PPR 6400
```

## 4. 常见接线问题

如果电机只振动或原地摆动，依次检查：

- TMC2209 `VIO` 是否连接到 3.3 V。
- `EN` 是否拉低，TMC2209 的使能信号低有效。
- MCU、TMC2209 和电机电源是否共地。
- `VM` 是否使用 12–24 V 独立电源。
- 两相电机线圈是否分别接到 A1/A2 和 B1/B2。
- 同一线圈的两根线是否接到同一个输出对。
- TMC2209 电流是否按电机额定相电流设置。

STH-42C2009-01 单相电阻约为 18.5 欧姆。相线接错时，典型现象是电机振动
但不连续旋转。

## 5. 低速检查

首次调试先使用低转速：

```text
TURN STOP
TURN RUN CW 0.5
```

在 3200 脉冲/圈时，`0.5 rpm` 对应约 27 Hz。转向相反时，把
`ROBOT_STEPPER_DIR_INVERT` 改为 `1` 并重新构建。

如果低速能转、提高速度后只振动，通常是缺少加减速、扭矩不足或驱动电流
设置不当。

## 6. 加减速

固件启动时先以 `ROBOT_STEPPER_START_RATE_HZ`（默认 20 Hz）发脉冲，再按
`ROBOT_STEPPER_ACCEL_HZ_PER_S` 向目标频率加速；接近目标位置时按
`ROBOT_STEPPER_DECEL_HZ_PER_S` 减速。当前为线性斜坡，控制任务每 10 ms
更新一次频率。

`TURN RUN CW` 默认的 5 rpm 在 3200 脉冲/圈时目标频率约为 267 Hz，默认
参数下约 0.5 s 完成加速。如果带负载仍然失步，可以把加速度和减速度都
降低到 `100–300` 再试。

## 7. 控制台命令

| 指令 | 说明 |
| --- | --- |
| `TURN` | 查看位置、目标、方向和速率 |
| `TURN 90` | 顺时针移动 90 度 |
| `TURN -45 3` | 逆时针移动 45 度，速度 3 rpm |
| `TURN RUN CW 5` | 顺时针连续运行 5 rpm |
| `TURN RUN CCW 2` | 逆时针连续运行 2 rpm |
| `TURN STOP` | 停止并将 STEP 拉低 |
| `TURN ZERO` | 把当前位置设为零 |
| `TURN PPR 12800` | 设置每圈脉冲数 |
| `TURN PULSES 3200 1000` | 以 1000 Hz 发送 3200 个脉冲 |

正角度、`CW` 和正脉冲数表示同一个逻辑方向。

## 8. PowerShell 移动示例

工程保留一个串口移动脚本：

```powershell
.\tools\step_move.ps1 -Port COM8 -Degrees 360 -Rpm 5
```

脚本会依次执行 `TURN ZERO`、`TURN <角度> <转速>`，等待移动完成后读取
`TURN` 状态，并在退出时发送 `TURN STOP`。

## 9. 位置和限制

- 转台位置为开环计数。
- 计数器从复位或 `TURN ZERO` 开始。
- 没有原点开关时，系统不知道绝对机械位置。
- 断电后位置清零。
- 当前使用线性加减速，重载时可降低加速度参数。
- 最大脉冲频率由 `ROBOT_STEPPER_MAX_RATE_HZ` 限制，默认 20 kHz。
- RPM 会限制在 `ROBOT_STEPPER_MIN_RPM` 和 `ROBOT_STEPPER_MAX_RPM` 之间。

## 10. 实机验证记录

以下是加入加减速之前的历史记录：

- `TURN 10 1`：实测位置约 `10.013` 度。
- `TURN -10 1`：回到 `0.000` 度。
- `TURN RUN CW 1`：连续运行约 7.5 度后停机。
- `TURN STOP`：状态回到 `idle`，STEP 输出关闭。

加入线性加减速后，需要用 `TURN 10 1`、`TURN -10 1` 和
`TURN RUN CW 0.5` 重新上板确认。
