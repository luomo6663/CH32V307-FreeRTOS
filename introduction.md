# CH32V307-FreeRTOS 主工程简介

本工程是运行在 CH32V307VCT6 上的机器人控制固件，使用 FreeRTOS 调度四轮麦克纳姆底盘、舵机机构和单轴步进转台。上位机通过 USART3 发送运动和调试命令，也可以从同一串口读取状态或采集遥测数据。

## 主要功能

| 模块 | 功能 |
| --- | --- |
| 麦轮底盘 | 将车体前后、横移和旋转速度换算成四轮目标转速，根据正交编码器反馈做独立闭环控制，并估算底盘位姿。 |
| 舵机机构 | 支持左右侧切换、分关节限位、平滑移动、预设动作、保持输出和关闭 PWM。 |
| 步进转台 | 通过 STEP/DIR 控制相对角度或连续旋转，支持线性加减速和位置清零。位置由脉冲计数估算。 |
| 串口控制台 | 支持运动指令、接线标定、PID 调参和故障查询，可输出 CSV 或 VOFA+ FireWater 遥测。 |

上电后，四轮电机保持滑行，舵机输出 1500 µs 中位脉宽，转台保持停止。底盘闭环控制具备命令超时、堵转和反馈方向异常保护。

## 固件如何运行

[User/main.c](User/main.c) 初始化时钟、串口控制台和应用模块，创建任务后启动 FreeRTOS 调度器。控制台任务解析串口命令，控制任务每 10 ms 更新一次编码器反馈、底盘输出、舵机和转台状态，遥测任务按配置周期输出数据。

以底盘速度命令 `V vx vy wz` 为例，控制任务先通过麦轮逆运动学计算四轮目标转速，再用各轮的编码器反馈调整电机 PWM。实测轮速同时用于里程计积分。

## 从哪里读代码

| 路径 | 内容 |
| --- | --- |
| [User/app/](User/app/) | 任务创建、串口通信、命令解析、遥测和日志。 |
| [User/control/](User/control/) | 电机、编码器、速度环、麦轮运动学、舵机和步进转台控制。 |
| [User/config/robot_config.h](User/config/robot_config.h) | 引脚、机械尺寸、控制周期、PID、限幅和保护阈值。 |
| [User/FreeRTOSConfig.h](User/FreeRTOSConfig.h) | FreeRTOS 内核配置。 |
| [FreeRTOS/](FreeRTOS/) | FreeRTOS 内核和移植代码。 |

## 构建与首次连接

1. 在 MounRiver Studio 中导入 [CH32V307-FreeRTOS.wvproj](CH32V307-FreeRTOS.wvproj)，或在工程根目录执行下方命令。
2. 命令行构建完成后，使用 MounRiver Studio 或 WCH-Link 工具烧录 `build/CH32V307-FreeRTOS.hex`。
3. 连接 USART3（PB10 为 TX，PB11 为 RX），串口设置为 115200、8-N-1。发送 `H` 查看帮助，发送 `STATUS` 查看底盘状态。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1
```

完整命令和接线见 [README](README.md)。任务和控制数据流见 [固件架构](docs/ARCHITECTURE.md)，外设操作见 [舵机控制](docs/SERVO_CONTROL.md) 和 [步进转台](docs/STEPPER_TURNTABLE.md)。
