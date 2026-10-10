# 固件架构说明

当前版本：`v1.0.0`

本文档说明固件的分层结构、FreeRTOS 任务、控制数据流、实时约束和资源占用。
硬件连接和完整命令表见 [MOTOR_CONTROL.md](MOTOR_CONTROL.md)。

## 1. 总体设计

固件按三层组织：

- `config/`：集中保存引脚、机械尺寸、限幅、PID 和周期参数。
- `control/`：实现电机、编码器、舵机、步进、麦轮运动学和底盘控制。
- `app/`：负责启动、任务、串口命令、遥测和日志。

实时控制由 `control` 任务统一执行。控制台只负责解析命令和调用公开接口，
不直接修改电机定时器或编码器寄存器。这样可以把实时控制与串口阻塞、文本
格式化隔离开。

## 2. 启动流程

`User/main.c` 按以下顺序启动：

1. 设置 NVIC 优先级分组。
2. 更新系统时钟并初始化延时。
3. 初始化日志互斥和 USART3 控制台。
4. 初始化遥测、底盘、舵机和步进转台。
5. 创建控制、控制台和遥测任务。
6. 启动 FreeRTOS 调度器。

启动阶段不会自动驱动电机。舵机固定输出 `1500 us` 中位，转台保持停止。

## 3. 目录结构

```text
User/
  main.c                  启动入口
  FreeRTOSConfig.h        FreeRTOS 配置
  ch32v30x_it.c           基础异常和中断入口
  config/
    robot_config.h        全部可调参数和固件版本
  control/
    motor.[ch]            H 桥 PWM、方向和刹车
    encoder.[ch]          正交编码器计数和 RPM
    pid.[ch]              通用 PI(D) 控制器
    wheel_control.[ch]    单轮速度环和故障保护
    mecanum.[ch]          麦轮运动学和里程计
    chassis.[ch]          模式机、命令锁存和状态汇总
    servo.[ch]            舵机 PWM 和扫描
    stepper.[ch]          STEP/DIR 转台控制
  app/
    robot_app.[ch]        任务创建、启动横幅和 RTOS 钩子
    console.[ch]          控制台命令解析
    comm.[ch]             USART3 中断接收和 FIFO
    telemetry.[ch]        CSV 和 VOFA+ 遥测
    log.[ch]              printf 互斥
tools/
  build.ps1               命令行构建
  step_move.ps1           转台串口移动示例
```

## 4. 任务模型

| 任务 | 优先级 | 栈（word） | 周期 | 职责 |
| --- | --- | --- | --- | --- |
| `control` | 5 | 448 | 10 ms，固定周期 | 编码器采样、速度环、PWM、舵机和步进更新 |
| `console` | 3 | 448 | 5 ms 轮询 | 读取串口行、解析命令、下发控制指令 |
| `telemetry` | 2 | 512 | 可配置 | 输出 CSV 或 VOFA+ 数据 |
| `IDLE` | 0 | 256 | 空闲 | FreeRTOS 空闲任务 |

任务之间的约束：

- 只有 `control` 任务执行周期性实时控制。
- 底盘运动命令通过 `CHASSIS_Command*()` 锁存，由 `control` 任务在下一周期应用。
- 状态通过 `CHASSIS_GetStatus()` 整体拷贝，避免读取到半更新状态。
- USART3 中断只写接收 FIFO，不调用 FreeRTOS API。
- 控制台输出由 `LOG_Lock()` / `LOG_Unlock()` 串行化。

## 5. 底盘模式

| 模式 | 含义 |
| --- | --- |
| `IDLE` | 目标为零，电机滑行，安全默认状态 |
| `TEST` | 单轮或四轮开环占空比，用于接线和台架检查 |
| `SPEED` | 单轮或四轮闭环 RPM，用于速度环标定 |
| `DRIVE` | 使用 `vx/vy/wz` 的麦轮闭环运动 |

`DRIVE` 和 `SPEED` 都有命令超时保护。运动故障会锁存，需使用 `CLR` 清除。

## 6. 运动控制数据流

正常底盘运动的数据流如下：

```text
V vx vy wz
  -> console 任务
  -> CHASSIS_CommandTwist()
  -> control 任务，每 10 ms
     -> 检查命令超时和车轮反馈状态
     -> Mecanum_TwistToMotorsLimited()
     -> 目标 RPM
     -> WHEEL_CONTROL_SetTargets()
     -> ENCODER_Update()
     -> WHEEL_CONTROL_Update()
     -> MOTOR_SetDuty() / MOTOR_Update()
     -> 正运动学和里程计积分
```

`TEST` 模式跳过速度环，直接把开环占空比写入电机。`SPEED` 模式把四个目标
RPM 送入单轮速度环。`IDLE` 模式清除目标并让电机滑行。

## 7. 单轮速度环

- 控制周期为 `10 ms`，即 `100 Hz`。
- 编码器读取 16 位定时器计数，使用有符号差值自然处理溢出。
- RPM 由本周期增量、每圈计数和控制周期换算。
- 每轮独立使用 PI(D)，默认参数集中在 `robot_config.h`。
- 输出限幅与积分抗饱和同时生效。
- 目标非零但转速持续过低时判为堵转。
- 目标与反馈方向持续相反时判为反接。
- 缺少反馈且没有配置开环增益时，拒绝进入 `DRIVE`。

## 8. 麦轮运动学

逻辑轮序为 `FL, FR, RL, RR`，与电机通道的映射如下：

| 逻辑轮 | 电机通道 | 物理位置 |
| --- | --- | --- |
| `FL` | M2 / W3 | 左前 |
| `FR` | M1 / W2 | 右前 |
| `RL` | M3 / W4 | 左后 |
| `RR` | M0 / W1 | 右后 |

逆运动学：

```text
FL = vx - vy - k * wz
FR = vx + vy + k * wz
RL = vx + vy - k * wz
RR = vx - vy + k * wz

k = 半轴距 + 半轮距
```

当任一轮超过速度上限时，四个轮目标按相同比例缩小，保持整车运动方向不变。
正运动学使用同一组车轮方向符号，输出 `vx/vy/wz` 并积分位姿。

## 9. 控制台与遥测

控制台使用 USART3：

- `PB10`：TX
- `PB11`：RX
- `115200-8-N-1`

接收中断把字节写入 256 字节环形 FIFO。控制台任务按行读取、统一转为大写并
使用 `strtok` 拆分参数。

遥测与控制台共用 USART3，支持：

- CSV：周期输出模式、四轮 RPM、目标、占空比、位姿和故障。
- VOFA+：FireWater 格式，可查看单轮 PID 分量或四轮目标/实际/输出。

开启高速遥测会占用串口带宽，命令响应速度可能下降。

## 10. 资源占用

`tools/build.ps1` 的命令行构建结果：

```text
text   data    bss    dec
53784    592  15840  70216
```

其中 BSS 包含 FreeRTOS 堆和静态状态。链接脚本预留：

- 256 K Flash
- 32 K SRAM 下限
- 2 K 中断栈

使用 32 K SRAM 下限可以兼容不同 CH32V307 Option Bytes 分区。

## 11. 关键设计约定

- 所有可调参数优先放入 `robot_config.h`。
- 实时控制模块不解析命令行，也不执行阻塞打印。
- 运动命令必须可超时停车。
- 车轮故障在 `DRIVE` 模式下停止整车，在 `SPEED` 模式下只影响故障轮。
- 上电、复位和停止命令后的安全默认状态均为电机滑行。
