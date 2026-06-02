# NN 替代 PID 控制实现分析

本文基于当前工程源码，说明 `mc_nn_control` 如何用神经网络接管 PX4 多旋翼传统 PID 控制链路，并列出对应文件的相对目录。

## 1. 结论概览

本项目里真正实现“NN 替代 PID”的主模块是：

- `src/modules/mc_nn_control/`

它不是只替换某一个 PID 环，而是把多旋翼从位置误差、姿态、线速度、角速度到电机输出的控制链路压缩成一次前馈神经网络推理：

```text
trajectory_setpoint
vehicle_local_position
vehicle_attitude
vehicle_angular_velocity
        |
        v
15 维 observation
        |
        v
TensorFlow Lite Micro 模型推理
        |
        v
4 维网络输出
        |
        v
RescaleActions() 物理映射
        |
        v
actuator_motors.control[0..3]
```

传统 PX4 多旋翼控制大致是：

```text
位置控制 mc_pos_control
  -> 姿态控制 mc_att_control
  -> 角速度控制 mc_rate_control
  -> 控制分配 control_allocator
  -> actuator_motors
```

`mc_nn_control` 进入自己的外部飞行模式后，会声明不使用 PX4 的多旋翼位置控制和控制分配，网络直接发布 `actuator_motors`，也就是直接给 4 个电机控制量。

## 2. 模块如何接入 PX4

### 2.1 编译开关

模块由 Kconfig 控制：

- `src/modules/mc_nn_control/Kconfig`

核心开关是：

```text
CONFIG_MODULES_MC_NN_CONTROL=y
CONFIG_LIB_TFLM=y
```

已有的 neural 板级配置：

- `boards/px4/sitl/neural.px4board`
- `boards/px4/fmu-v6c/neural.px4board`
- `boards/mro/pixracerpro/neural.px4board`

模块 CMake：

- `src/modules/mc_nn_control/CMakeLists.txt`

这里把 `mc_nn_control.cpp`、`control_net.cpp` 编进模块，并链接 `tensorflow_lite_micro`。

### 2.2 启动条件

自启动脚本在：

- `ROMFS/px4fmu_common/init.d/rc.mc_apps`

当参数 `MC_NN_EN == 1` 时启动：

```sh
mc_nn_control start
```

参数定义在：

- `src/modules/mc_nn_control/mc_nn_control_params.c`

关键参数：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `MC_NN_EN` | `1` | 是否开机启动 `mc_nn_control` |
| `MC_NN_MAX_RPM` | `22000` | 电机最大 RPM，用于输出归一化 |
| `MC_NN_MIN_RPM` | `1000` | 电机最小 RPM |
| `MC_NN_THRST_COEF` | `1.2` | 推力系数，代码里会除以 `100000` |
| `MC_NN_MANL_CTRL` | `1` | 是否允许遥控手动生成轨迹 setpoint |

### 2.3 外部飞行模式注册

`mc_nn_control` 启动后注册名为 `Neural Control` 的外部飞行模式：

- `src/modules/mc_nn_control/mc_nn_control.cpp`
  - `RegisterNeuralFlightMode()`
  - `CheckModeRegistration()`
  - `ConfigureNeuralFlightMode()`

注册成功后，只有当 `vehicle_status.nav_state == _mode_id` 时，`_use_neural` 才会变成 `true`，控制器才真正输出电机控制量。

## 3. 它如何绕开传统 PID

核心在：

- `src/modules/mc_nn_control/mc_nn_control.cpp`
  - `ConfigureNeuralFlightMode()`

该函数发布 `config_control_setpoints`，关键标志如下：

```cpp
config_control_setpoints.flag_multicopter_position_control_enabled = false;
config_control_setpoints.flag_control_position_enabled = false;
config_control_setpoints.flag_control_allocation_enabled = false;
config_control_setpoints.flag_control_manual_enabled = _param_manual_control.get();
config_control_setpoints.flag_control_climb_rate_enabled = true;
config_control_setpoints.flag_control_termination_enabled = true;
```

含义：

- 关闭 PX4 多旋翼位置控制链路。
- 关闭 PX4 控制分配器对该模式的接管。
- 保留必要的模式、手动输入和安全相关控制标志。
- 网络最后直接发布 `actuator_motors`。

所以这个实现不是“PID 后面加一个 NN 补偿”，也不是“NN 调 PID 参数”，而是一个端到端控制器：输入状态和目标，输出电机控制量。

## 4. 控制循环触发方式

核心入口：

- `src/modules/mc_nn_control/mc_nn_control.cpp`
  - `init()`
  - `Run()`

`init()` 把模块注册为 `vehicle_angular_velocity` 的回调工作项：

```cpp
_angular_velocity_sub.registerCallback()
```

因此控制循环由角速度消息触发。也就是说，NN 控制的运行频率跟随陀螺仪/角速度话题更新频率，而不是另开固定定时器。

`Run()` 中只在进入 `Neural Control` 模式后继续执行控制逻辑：

```cpp
if (!_use_neural) {
    return;
}
```

然后在每次 `vehicle_angular_velocity` 更新时：

1. 读取最新姿态 `vehicle_attitude`。
2. 读取最新本地位置和速度 `vehicle_local_position`。
3. 读取或生成轨迹目标 `trajectory_setpoint`。
4. 填充 15 维输入张量。
5. 调用 TFLM `Invoke()`。
6. 把网络输出映射成电机控制量。
7. 发布 `actuator_motors`。
8. 发布 `neural_control` 调试话题。

## 5. 15 维网络输入如何构造

核心函数：

- `src/modules/mc_nn_control/mc_nn_control.cpp`
  - `PopulateInputTensor()`

源码注释写明输入布局：

```text
[pos_err(3), lin_vel(3), att(6), ang_vel(3)]
```

实际写入顺序是：

| 索引 | 内容 | 来源 |
| ---: | --- | --- |
| `0..2` | 位置误差 `trajectory_setpoint_local - position_local` | `trajectory_setpoint`、`vehicle_local_position` |
| `3..8` | 姿态旋转矩阵前两行 | `vehicle_attitude` 四元数转换 |
| `9..11` | 线速度 | `vehicle_local_position.vx/vy/vz` |
| `12..14` | 角速度 | `vehicle_angular_velocity.xyz` |

注意：函数开头注释写的是 `[pos_err(3), lin_vel(3), att(6), ang_vel(3)]`，但代码实际填充顺序是 `[pos_err(3), att(6), lin_vel(3), ang_vel(3)]`。调试日志 `neural_control.observation` 也按代码实际顺序发布。

### 5.1 坐标变换

`PopulateInputTensor()` 内部定义了两个矩阵：

- `frame_transf`
- `frame_transf_2`

这些矩阵把 PX4 的本地 NED/机体系相关量变换到模型训练时使用的坐标系。位置、目标位置、线速度、姿态矩阵、角速度都会经过相应变换后再送入网络。

### 5.2 为什么姿态用 6 维

姿态不是直接用欧拉角，也不是直接用四元数，而是取旋转矩阵前两行共 6 个数：

```text
R00 R01 R02 R10 R11 R12
```

这对神经网络更友好：

- 避免欧拉角奇异性。
- 避免四元数 `q` 和 `-q` 表示同一姿态带来的不连续。
- 旋转矩阵前两行已经能约束完整姿态，第三行可由正交关系得到。

## 6. 网络模型如何加载和推理

模型文件：

- `src/modules/mc_nn_control/control_net.cpp`
- `src/modules/mc_nn_control/control_net.hpp`

`control_net.cpp` 里是编译进固件的 TFLite FlatBuffer 字节数组：

```cpp
alignas(16) const unsigned char control_net_tflite[] = { ... };
```

`control_net.hpp` 中记录模型大小：

```cpp
constexpr unsigned int control_net_tflite_size = 15088;
```

加载和初始化在：

- `src/modules/mc_nn_control/mc_nn_control.cpp`
  - `InitializeNetwork()`

实现步骤：

1. `tflite::GetModel(control_net_tflite)` 从 C 数组读取模型。
2. 创建 `MicroMutableOpResolver<3>`。
3. 注册 3 类算子：
   - `FullyConnected`
   - `Relu`
   - `Add`
4. 分配静态 tensor arena：
   - `kTensorArenaSize = 10 * 1024`
5. 创建 `tflite::MicroInterpreter`。
6. `AllocateTensors()`。
7. 取得输入 tensor。

推理调用在 `Run()` 中：

```cpp
TfLiteStatus invoke_status = _interpreter->Invoke();
```

随后通过：

```cpp
_output_tensor = _interpreter->output(0);
```

读取 4 维输出。

## 7. 网络输出如何变成电机控制量

核心函数：

- `src/modules/mc_nn_control/mc_nn_control.cpp`
  - `RescaleActions()`
  - `PublishOutput()`

网络输出是 4 个浮点数，对应 4 个电机。`RescaleActions()` 对每个输出执行同样映射：

1. 截断到 `[-1, 1]`。
2. 加 `1`，变为 `[0, 2]`，作为抽象推力量。
3. 用推力系数换算到 RPS：
   ```cpp
   rps = sqrt(output / thrust_coeff);
   ```
4. RPS 转 RPM：
   ```cpp
   rpm = rps * 60.0f;
   ```
5. 按 `MC_NN_MIN_RPM` 和 `MC_NN_MAX_RPM` 归一化。
6. 做一个二次曲线映射：
   ```cpp
   a = 0.8f;
   b = 0.2f;
   ```

这说明网络本身没有直接输出 PX4 actuator 的最终归一化值，而是先输出一个近似推力域的控制动作，再由代码结合电机物理参数映射到 PX4 电机控制量。

最后 `PublishOutput()` 发布：

```cpp
actuator_motors.control[0] = command_actions[0];
actuator_motors.control[1] = command_actions[1];
actuator_motors.control[2] = command_actions[2];
actuator_motors.control[3] = command_actions[3];
actuator_motors.control[4..11] = -NAN;
```

输出话题：

- `actuator_motors`

这一步没有再经过 PX4 的 `control_allocator` 混控矩阵，所以 NN 必须自己学会“状态/目标 -> 四个电机”的完整映射。

## 8. 手动控制和 Offboard setpoint

`MC_NN_MANL_CTRL=1` 时，模块会根据遥控输入生成位置目标：

- `src/modules/mc_nn_control/mc_nn_control.cpp`
  - `generate_trajectory_setpoint()`
  - `check_setpoint_validity()`
  - `reset_trajectory_setpoint()`

手动输入来源：

- `manual_control_setpoint`

逻辑是把 pitch/roll/throttle 转成世界系速度增量，然后积分成 `_trajectory_setpoint.position`。

当 `MC_NN_MANL_CTRL=0` 时，模块使用外部 `trajectory_setpoint`。只有三个位置分量都是 finite 时才接受该 setpoint。

## 9. 调试和日志

调试消息定义：

- `msg/NeuralControl.msg`

字段：

```text
float32[15] observation
float32[4] network_output
int32 controller_time
int32 inference_time
```

发布位置：

- `src/modules/mc_nn_control/mc_nn_control.cpp`
  - `Run()` 末尾

默认 logger 话题：

- `src/modules/logger/logged_topics.cpp`

其中添加了：

```cpp
add_topic("neural_control", 50);
```

消息构建列表：

- `msg/CMakeLists.txt`

其中包含：

```text
NeuralControl.msg
```

调试时重点看：

- `neural_control.observation[0..14]`
- `neural_control.network_output[0..3]`
- `neural_control.inference_time`
- `actuator_motors.control[0..3]`

## 10. 与传统 PID 的关键差异

| 对比项 | 传统 PX4 PID 链路 | `mc_nn_control` |
| --- | --- | --- |
| 控制结构 | 多级串联控制器 | 一个前馈神经网络 |
| 中间量 | 位置、速度、姿态、角速度 setpoint 层层传递 | 中间层由网络隐式表达 |
| 输出 | torque/thrust 经控制分配到电机 | 直接输出 4 个电机控制量 |
| 参数 | 多组 PID 和限幅参数 | 主要是模型权重 + RPM/推力系数 |
| 状态记忆 | PID 可有积分项 | 当前模型接口是前馈输入，无显式积分状态 |
| 安全边界 | 各级控制器限幅、anti-windup、控制分配约束 | 输出截断、RPM 映射、PX4 模式/arming 检查 |
| 调参方式 | 改 PID 参数 | 换模型或调输出映射参数 |

## 11. 相关文件相对目录

### NN 控制主实现

| 相对路径 | 作用 |
| --- | --- |
| `src/modules/mc_nn_control/mc_nn_control.cpp` | 主控制逻辑：模式注册、输入构造、TFLM 推理、输出发布 |
| `src/modules/mc_nn_control/mc_nn_control.hpp` | 类定义、uORB 订阅/发布、参数句柄 |
| `src/modules/mc_nn_control/control_net.cpp` | TFLite 模型字节数组，编译进固件 |
| `src/modules/mc_nn_control/control_net.hpp` | 模型数组声明和大小 |
| `src/modules/mc_nn_control/mc_nn_control_params.c` | `MC_NN_*` 参数定义 |
| `src/modules/mc_nn_control/CMakeLists.txt` | 模块编译和 TFLM 链接 |
| `src/modules/mc_nn_control/Kconfig` | 模块 Kconfig 开关 |

### uORB 消息和日志

| 相对路径 | 作用 |
| --- | --- |
| `msg/NeuralControl.msg` | NN 控制调试消息定义 |
| `msg/CMakeLists.txt` | 把 `NeuralControl.msg` 加入消息生成 |
| `src/modules/logger/logged_topics.cpp` | 默认记录 `neural_control` 话题 |

### 启动和板级配置

| 相对路径 | 作用 |
| --- | --- |
| `ROMFS/px4fmu_common/init.d/rc.mc_apps` | 多旋翼应用启动脚本，按 `MC_NN_EN` 启动模块 |
| `boards/px4/sitl/neural.px4board` | SITL neural 构建配置 |
| `boards/px4/fmu-v6c/neural.px4board` | FMU v6C neural 构建配置 |
| `boards/mro/pixracerpro/neural.px4board` | Pixracer Pro neural 构建配置 |

### 对照：传统控制链路

| 相对路径 | 作用 |
| --- | --- |
| `src/modules/mc_pos_control/` | 多旋翼位置控制 |
| `src/modules/mc_att_control/` | 多旋翼姿态控制 |
| `src/modules/mc_rate_control/` | 多旋翼角速度 PID 控制 |
| `src/modules/control_allocator/` | 控制分配器 |

### 另一路 NN/RL 控制：RAPTOR

本工程还包含 `mc_raptor`：

| 相对路径 | 作用 |
| --- | --- |
| `src/modules/mc_raptor/` | RAPTOR/RL 控制模块 |
| `src/modules/mc_raptor/README.md` | RAPTOR 使用说明 |
| `src/modules/mc_raptor/blob/policy.tar` | RAPTOR 策略文件 |
| `boards/px4/sitl/raptor.px4board` | SITL RAPTOR 构建配置 |
| `boards/px4/fmu-v6c/raptor.px4board` | FMU v6C RAPTOR 构建配置 |
| `msg/versioned/RaptorInput.msg` | RAPTOR 输入日志消息 |
| `msg/versioned/RaptorStatus.msg` | RAPTOR 状态日志消息 |

`mc_raptor` 和 `mc_nn_control` 都属于神经网络/强化学习控制方向，但本文分析的“NN 替代 PID 并直接输出电机”的主路径是 `mc_nn_control`。

## 12. 阅读源码建议

建议按以下顺序读：

1. `src/modules/mc_nn_control/mc_nn_control_params.c`
2. `ROMFS/px4fmu_common/init.d/rc.mc_apps`
3. `src/modules/mc_nn_control/mc_nn_control.hpp`
4. `src/modules/mc_nn_control/mc_nn_control.cpp`
   - 先看 `ConfigureNeuralFlightMode()`
   - 再看 `Run()`
   - 再看 `PopulateInputTensor()`
   - 最后看 `RescaleActions()` 和 `PublishOutput()`
5. `msg/NeuralControl.msg`
6. `src/modules/mc_nn_control/control_net.cpp`

这样能先理解它如何接管 PX4 控制链路，再看 NN 输入输出细节。
