# TFLM Robot Firmware

这是一个基于 Zephyr RTOS 的机器人控制固件仓库。当前代码不只是底盘和遥控，还已经包含了 IMU 采集、姿态 EKF、IMU 加热控温、`zbus` 话题分发，以及一个预留的 TFLM 线程入口。

这份 README 以当前源码为准，重点说明真实入口、线程组织、话题流向，以及 IMU 这条核心链路。

## 先看什么

如果你第一次读这个仓库，建议按下面顺序看：

1. `src/main.c`
2. `projects/apps/System_startup.cpp`
3. `projects/thread/*`
4. `modules/*`
5. `topic/*`
6. `algorithm/*`

入口关系很直接：

```text
main()
  -> System_Bsp_Init()
  -> System_Modules_Init()
  -> System_Thread_Start()
```

其中：

- `System_Bsp_Init()` 负责较底层的板级线程初始化，目前会先初始化 CAN 发送线程。
- `System_Modules_Init()` 负责各功能线程的 `thread_init()`。
- `System_Thread_Start()` 统一启动各线程，并给出优先级。

## 当前线程组织

`projects/apps/System_startup.cpp` 中目前已经接入了这些线程入口：

| 线程 | 入口命名空间 | 作用 |
| --- | --- | --- |
| Remote | `thread::remote` | 接收遥控器数据并发布 `remote_to` |
| IMU | `thread::imu` | IMU 读数、加热、静态校准、姿态解算、发布 `imu_to` |
| CAN_TX | `thread::can` | 消费控制结果并发送 CAN |
| Chassis | `thread::chassis` | 底盘控制 |
| Gimbal | `thread::gimbal` | 云台控制 |
| GPIO/Output | `thread::output` | 输出状态指示 |
| TFLM | `thread::ml` | TFLM 推理线程入口，当前默认不启动 |

默认是否编译这些线程由 `projects/thread/Kconfig` 决定，不同板级 `.conf` 可以进一步打开或关闭对应功能。

## 系统数据流

这个仓库的模块之间主要通过 `zbus` 解耦，而不是互相直接耦合调用。

典型数据流如下：

```text
Remote driver/thread
  -> pub_remote_to
  -> Chassis / Gimbal

IMU source
  -> IMU thread
  -> Quaternion EKF
  -> pub_imu_to
  -> 其他姿态消费者

Chassis / Gimbal / other control threads
  -> pub_to_can_tx
  -> CAN TX thread
  -> motor bus
```

目前已能直接看到的 topic 包括：

- `topic/remote_to`
- `topic/imu_to`
- `topic/to_can_tx`

## IMU 链路

当前 IMU 路径是这套仓库里比较完整的一条链：

```text
具体 IMU 驱动
  -> Source::Read(sample)
  -> thread::imu::ImuManager
  -> attitude::Processor
  -> alg::attitude::QuaternionEkf
  -> topic::imu_to::Message
  -> zbus_chan_pub(pub_imu_to)
```

### 1. 底层驱动抽象

`modules/imu/imu.hpp` 定义了统一的 `Source` 接口，底层驱动需要提供：

- `Init()`
- `Read(Sample& sample)`
- `Calibrate()`
- `PeriodMs()`

`Sample` 是上层统一消费的工程量数据，包含：

- `gyro[3]`
- `accel[3]`
- `temperature`
- `dt`

当前 `ImuManager::SelectSource()` 通过 Kconfig 选择具体驱动：

- `CONFIG_MOD_DEV_IMU_ICM42688P`
- `CONFIG_MOD_DEV_IMU_BMI088`

### 2. IMU 线程职责

`modules/imu/imu.cpp` 里的 `thread::imu::ImuManager` 负责三件事：

1. 选择并初始化底层 IMU 数据源
2. 预热并执行一次静态校准
3. 周期读取 IMU，完成姿态解算并发布 `imu_to`

初始化阶段会先：

```text
SelectSource()
-> source_->Init()
-> heater_.Init()
-> PrepareCalibration()   // 默认 thread_init() 中开启
-> attitude_.Init()
```

其中 `PrepareCalibration()` 会先等待温度稳定，再触发一次 `source_->Calibrate()`。

### 3. 姿态解算

姿态解算位于 `algorithm/filter/quaternion_ekf/quaternion_ekf.cpp`。

这套 EKF 的核心特点：

- 状态量以四元数为主，同时估计陀螺零偏
- 角速度用于预测，重力方向用于观测修正
- 对加速度做低通，减小机械振动影响
- 用卡方检验抑制异常观测
- 在静止条件成立时，对 yaw 漂移做额外零偏缓慢修正
- 最终输出四元数和标准 ZYX 欧拉角

代码里还保留了 `YawTotalAngle`，用于跨过 `+/-180` 度后的连续角度累计，方便连续角度控制。

### 4. `imu_to` 发布内容

`topic/imu_to/imu_to.hpp` 里当前发布消息为：

- `quaternion[4]`
- `gyro[3]`
- `temperature`
- `roll`
- `pitch`
- `yaw`
- `yaw_total`

也就是说，IMU 线程对外发布的是“已经过姿态解算整理后的结果”，不是裸寄存器原始值。

## Remote 链路

遥控相关代码在 `modules/remotes`。

当前 `remote_to::Message` 已经是语义化后的控制输入，字段包括：

- `chassisx`
- `chassisy`
- `yaw`
- `pitch`
- `chassis_mode`
- `shoot_ctrl`
- `reload_ctrl`
- `autoaim_ctrl`
- `supercap_ctrl`

也就是说，上层控制线程消费的是统一控制语义，不需要直接关心底层遥控协议细节。

## 目录结构

```text
src/            Zephyr 应用入口
projects/       系统启动、线程组织、板级 overlay/conf/board.cmake
modules/        设备或功能模块，如 IMU、遥控、电机
algorithm/      控制、滤波、辨识、TFLM 相关算法
drivers/        通信和设备驱动封装
topic/          zbus topic 定义
cmd/            shell 命令
doc/            设计说明、移植记录、分析文档
```

如果你是为了看某条功能链路，建议这样找：

- 看系统启动：`src/main.c`、`projects/apps/System_startup.cpp`
- 看 IMU：`modules/imu/*`、`algorithm/filter/quaternion_ekf/*`、`topic/imu_to/*`
- 看遥控：`modules/remotes/*`、`topic/remote_to/*`
- 看线程控制：`projects/thread/*`

## 构建方式

项目通过 Zephyr 的 `west build` 构建，并通过 `BOARD_CFG` 选择项目内的板级配置分组：

```bash
west build -b <board> -- -DBOARD_CFG=<config>
```

如果没有显式传入 `BOARD_CFG`，顶层 `CMakeLists.txt` 会默认把 `BOARD` 当作配置组名使用。

板级配置文件位于：

```text
projects/boards/*/<BOARD_CFG>/
```

这一层通常包含：

- `<board>.overlay`
- `<board>.conf`
- `board.cmake`

## SDK glue

顶层 `CMakeLists.txt` 默认会从下面这个路径引入 HPMicro 的 board/soc/dts 补充内容：

```text
D:/Zephyr_HPMicro/sdk_glue
```

也可以通过环境变量覆盖：

```bash
SDK_GLUE_DIR=<path>
```

这会影响：

- `BOARD_ROOT`
- `SOC_ROOT`
- `DTS_ROOT`
- `ZEPHYR_EXTRA_MODULES`

## 开发约定

- 模块之间优先通过 `topic/*` 和 `zbus` 通信，减少直接耦合。
- 线程入口统一使用 `thread_init()` / `thread_start()` 风格。
- 板级连接关系尽量放在 `projects/boards/*` 下的 overlay/conf 中，不把项目专用连接写死到共享 board 文件里。
- 读源码时，先确认数据是“原始驱动数据”、还是“模块整理后的工程量”、还是“topic 对外发布语义”。

## 相关文件

- IMU topic: `topic/imu_to/imu_to.hpp`
- IMU 线程: `modules/imu/imu.cpp`
- 姿态 EKF: `algorithm/filter/quaternion_ekf/quaternion_ekf.cpp`
- 遥控 topic: `topic/remote_to/remote_to.hpp`
- 系统启动: `projects/apps/System_startup.cpp`
- 板级总入口: `CMakeLists.txt`
