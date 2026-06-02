# NN 替代底盘行进轮 PID 规划

## 1. 目标与边界

本文规划如何在当前 Zephyr 底盘工程中，用神经网络替代 chassis 行进轮的 PID 控制链。

替换目标只包含行进轮 drive motor：

```text
目标行进速度 + 当前行进状态
        |
        v
      NN 推理
        |
        v
  行进目标电流 current_ref
        |
        v
  DrivePwrCtrl 功率限制
        |
        v
      CAN 电流帧
```

不在第一阶段替换以下部分：

- 舵向电机 steer 的角度环和力矩环。
- `UpdateTarget()` 中的舵轮运动学和优劣弧逻辑。
- `DrivePwrCtrl` / `SteerPwrCtrl` 功率预测、功率分配和最终限流。
- CAN 组帧、遥控器解析、zbus 通信。

原因是当前底盘线程已经有清晰的分层：`ControlCalculate()` 负责产生“需要多少电流”，`PowerAlloc()` 负责决定“允许给多少电流”。NN 第一阶段应该替代前者，不越过后者。

## 2. 当前代码基线

核心文件：

| 文件 | 作用 |
| --- | --- |
| `projects/thread/chassis/trd_chassis.cpp` | 底盘主线程：遥控、运动学、控制、电源分配、CAN 发布 |
| `projects/thread/chassis/trd_chassis.hpp` | 舵轮数量、电机 ID、PID 和电机实例 |
| `algorithm/controller/pid/pid.hpp` | PID 控制器 |
| `algorithm/controller/power_ctrl/power_ctrl.hpp` | 电机功率约束层 |
| `algorithm/tflm/tflm.cpp` | TFLM 当前封装入口 |
| `scripts/train_chassis_rl.py` | 现有行进轮 RL 训练脚本雏形 |

当前底盘控制周期在 `Task()` 中固定为 1ms：

```cpp
static constexpr uint32_t kPeriodMs = 1;
```

当前行进轮控制位于 `ControlCalculate()`：

```cpp
const auto  snap = chassis_wheel[wi].drive_motor.ReadAll();
const float chassis_velocity = g_wh_target[wi].velocity * g_k_factor[wi];

wheel_pid[wi].drive_velocity.SetTarget(chassis_velocity);
wheel_pid[wi].drive_velocity.SetNow(snap.velocity);
const float torque_ref  = wheel_pid[wi].drive_velocity.Calc();
const float current_ref = wheel_pid[wi].drive_torque.Calc(torque_ref, snap.torque) / kTorqueK;
DrivePwrCtrl.SetTarget(wi, current_ref);
DrivePwrCtrl.SetMotorData(wi, snap.torque, snap.omega, wheel_pid[wi].drive_velocity.GetError());
```

等价控制链是：

```text
target_velocity
    -> drive_velocity PID
    -> torque_ref
    -> drive_torque PID
    -> / kTorqueK
    -> current_ref
    -> DrivePwrCtrl.SetTarget()
```

初始化参数：

| 控制器 | Kp | Ki | Kd |
| --- | ---: | ---: | ---: |
| `drive_velocity` | 5.0 | 0 | 0 |
| `drive_torque` | 0.5 | 0 | 0 |

当前为 P-only 串级控制，因此第一阶段 NN 并不需要学习复杂积分行为，但需要处理真机延迟、静摩擦、负载变化、速度反馈噪声和功率限制后的闭环影响。

## 3. 不能直接照搬 PX4 `mc_nn_control`

`doc/nn_replaces_pid_analysis.md` 分析的是 PX4 多旋翼 `mc_nn_control`：它进入独立飞行模式后关闭传统位置控制和控制分配，网络直接发布电机输出。

底盘这里不建议一开始做同样的端到端替换。

PX4 端到端路径：

```text
状态 + 轨迹目标 -> NN -> actuator_motors
```

本项目第一阶段推荐路径：

```text
单轮目标速度 + 单轮反馈 -> NN -> 行进目标电流 -> 功率限制 -> CAN
```

关键差异：

| 对比项 | PX4 `mc_nn_control` | 本底盘推荐方案 |
| --- | --- | --- |
| 替换范围 | 整条多旋翼控制链 | 只替换行进轮 PID |
| 输出 | 直接电机控制量 | PID 原始电流需求 |
| 安全层 | 依赖 PX4 模式和输出限幅 | 保留 `DrivePwrCtrl` 功率限制 |
| 上线风险 | 高 | 可分阶段灰度 |
| 数据需求 | 飞行状态和轨迹日志 | 单轮速度、转矩、电流、目标日志 |

因此本文不把 NN 放在 CAN 发布之后，也不让 NN 同时决定舵向角、行进速度和功率预算。

## 4. 建模对象

### 4.1 单轮被控对象

第一阶段把每个行进轮看成相同结构的速度控制对象：

```text
输入: 电调电流 i [A]
输出: 轮贡献线速度 v [m/s]
扰动: 地面摩擦、车体负载、舵向角变化、功率限流、电机反馈延迟
```

现有 DJI 电机驱动在 `DjiC6xx::ReadAll()` 中已经给出：

| 信号 | 含义 |
| --- | --- |
| `snap.velocity` | 电机反馈换算后的单轮线速度 |
| `snap.torque` | 当前转矩估计 |
| `snap.omega` | 输出轴角速度 |

`ControlCalculate()` 中的目标速度为：

```cpp
target_velocity = g_wh_target[wi].velocity * g_k_factor[wi];
```

这个目标已经包含：

- 遥控器速度指令。
- 舵轮逆运动学。
- 优劣弧导致的行进方向翻转。
- `kDriveSign[]` 安装方向补偿。

所以 NN 输入不需要再关心遥控器原始通道，也不需要重新做运动学。

### 4.2 电机简化模型

PC 训练时可以先使用一阶机械模型：

```text
tau = Kt * i
J * dv/dt = tau - B * v - tau_load
```

其中：

| 参数 | 初值 | 来源 |
| --- | ---: | --- |
| `Kt` | 0.3 N*m/A | 当前 `kTorqueK` |
| `J` | 0.028 | 现有总结中的辨识初值 |
| `B` | 0.123 | 现有总结中的辨识初值 |
| `dt` | 0.001 s | 当前底盘线程周期 |
| `max_current` | 20 A | C620/CAN 映射限幅 |
| `max_velocity` | 0.5 m/s | 当前 `KMaxMoveVelocity` |

这个模型只能作为起点。真机部署振荡时，优先怀疑模型缺少以下因素：

- 电调和 CAN 反馈的采样延迟。
- 静摩擦、库仑摩擦、地面接触变化。
- 功率限制后的目标电流被削顶。
- 速度反馈量化和 CAN 周期抖动。
- 舵向角未到位时，行进速度目标与实际地面速度不一致。

## 5. NN 接口设计

### 5.1 推荐第一版输入

建议输入 8 维：

```text
[
  target_velocity,
  current_velocity,
  velocity_error,
  current_torque,
  current_omega,
  last_current_cmd,
  target_delta,
  limit_ratio
]
```

字段说明：

| 输入 | 来源 | 说明 |
| --- | --- | --- |
| `target_velocity` | `g_wh_target[wi].velocity * g_k_factor[wi]` | 当前单轮目标速度 |
| `current_velocity` | `snap.velocity` | 当前速度反馈 |
| `velocity_error` | `target_velocity - snap.velocity` | 保留显式误差，降低网络学习难度 |
| `current_torque` | `snap.torque` | 当前转矩反馈 |
| `current_omega` | `snap.omega` | 功率模型也使用该量，可帮助区分高速和低速工况 |
| `last_current_cmd` | 上一帧 NN 或 PID 原始电流 | 给前馈网络一个近似动态记忆 |
| `target_delta` | 本帧目标速度 - 上帧目标速度 | 帮网络识别加减速阶段 |
| `limit_ratio` | 上一帧限流输出 / 原始需求 | 让网络感知功率限制是否长期介入 |

如果想先做最小版本，也可以用 3 维：

```text
[target_velocity, current_velocity, current_torque]
```

但 3 维接口很难表达延迟、目标突变、功率削顶和历史输出，真机更容易振荡。当前已经有 RL 真机振荡记录，因此不建议继续只依赖 3 维输入。

### 5.2 归一化

训练和固件必须使用同一套归一化常量。建议写进模型说明和 C++ 常量：

| 量 | 归一化建议 |
| --- | --- |
| 速度 | `/ 0.5f` |
| 速度误差 | `/ 0.5f` |
| 转矩 | `/ 6.0f`，对应约 20A * 0.3 |
| 角速度 | 按实测最大 `snap.omega` 设置 |
| 电流 | `/ 20.0f` |
| 目标变化量 | `/ 0.5f` |
| 限流比例 | 已在 `[0, 1]` 附近 |

训练脚本应导出一份 `normalization.json` 或在模型头文件中生成注释，固件侧不要手写另一套含义不同的比例。

### 5.3 输出

输出 1 维：

```text
normalized_current in [-1, 1]
```

固件侧映射：

```cpp
current_ref = normalized_current * 20.0f;
```

然后继续走：

```cpp
DrivePwrCtrl.SetTarget(wi, current_ref);
```

不要在 NN 输出后直接乘 `kCurrentScale` 填 CAN 帧。`kCurrentScale = 16384 / 20` 属于 `FramePublish()` 的 CAN 编码层。

## 6. 数据采集规划

### 6.1 第一批数据：PID 教师日志

先让现有 PID 跑，记录每个行进轮在每个控制周期的状态和 PID 输出。

建议日志字段：

```text
timestamp_ms
wheel_index
target_velocity
current_velocity
velocity_error
current_torque
current_omega
pid_torque_ref
pid_current_ref
limited_current
drive_power_pred
drive_power_budget
remote_vx
remote_vy
remote_spin
steer_angle
steer_target
```

最低必须字段：

```text
target_velocity,current_velocity,current_torque,current_omega,pid_current_ref,limited_current
```

采集动作覆盖：

- 原地静止到小速度阶跃。
- 小速度到反向小速度。
- 最大速度正反向阶跃。
- 缓慢斜坡输入。
- Spin 模式下前后轮方向相反的工况。
- 手推或轻微阻挡造成的外扰。
- 低电压或功率限制容易介入的工况。

注意：1ms 周期里不要高频 `printk("%f")`。更稳妥的做法是：

- 用环形缓冲区缓存二进制或定点日志。
- 降频输出，例如每 10ms 或 20ms 输出一行。
- 只在数据采集固件里打开日志，正常控制固件关闭。

### 6.2 第二批数据：系统辨识日志

为了让仿真更接近真机，需要单独做电机模型辨识。

建议注入电流序列：

- PRBS 伪随机二值电流，小幅度开始。
- 多组阶跃电流：`2A, 4A, 8A, 12A`。
- 低速保持电流，识别静摩擦死区。
- 正反向对称测试，识别安装方向和摩擦非对称。

记录：

```text
current_cmd, limited_current, velocity, omega, torque, timestamp
```

拟合模型从简单到复杂：

1. 一阶线性：`J * dv/dt = Kt*i - B*v`
2. 加库仑摩擦：`- Fc * sign(v)`
3. 加静摩擦死区：低速小电流无运动
4. 加延迟：输入电流延迟 `1~3` 个控制周期

只有当模型能复现实测阶跃响应，才进入 RL 或仿真闭环训练。否则仿真策略很容易在真机上学到过激控制。

## 7. 训练路线

### 7.1 阶段 A：监督学习拟合现有 PID

目标不是最终性能，而是打通数据、模型、TFLM、固件替换链路。

训练数据：

```text
X = NN 输入
y = pid_current_ref / 20.0
```

损失函数：

```text
loss = mse(pred_current, pid_current)
     + 0.02 * mse(delta_pred_current, delta_pid_current)
```

这样先得到一个“行为类似 PID”的模型，用来验证：

- TFLite 导出是否成功。
- TFLM 推理结果是否和 PC 一致。
- 固件侧归一化是否正确。
- 1ms 周期内推理时间是否可接受。

上线方式：影子模式，只计算 NN，不控制电机，同时记录 `nn_current_ref` 和 `pid_current_ref`。

通过标准：

- NN 与 PID 输出平均误差小于 `1A`。
- 最大误差在目标突变时可解释。
- 推理耗时稳定小于 `0.3ms`。
- 不影响底盘线程周期。

### 7.2 阶段 B：在 PID 数据上做改良监督学习

PID 当前是 P-only，会有稳态误差和抗扰不足。第二阶段可以不完全照抄 PID，而是基于数据和简单模型生成更好的标签。

可用标签策略：

```text
improved_current =
    pid_current_ref
  + Kv * velocity_error
  + Ka * target_delta
  + Kf * sign(velocity_error)  // 克服静摩擦
```

也可以把 PID 输出作为初始策略，再用真机影子日志评估 NN 在相同状态下会给多少电流。

这一阶段不要直接追求“比 PID 快很多”，重点是：

- 输出更平滑。
- 小误差时不抖。
- 反向时不过冲。
- 静止附近能克服死区但不来回振荡。

### 7.3 阶段 C：仿真闭环训练

使用辨识后的单轮模型训练闭环控制策略。相比现有 `scripts/train_chassis_rl.py`，需要补齐以下内容：

- 延迟模型：动作延迟、观测延迟各随机 `0~3ms`。
- 摩擦模型：粘性摩擦、库仑摩擦、静摩擦死区随机化。
- 负载扰动：随机外力矩和地面阻力。
- 目标生成：阶跃、斜坡、正弦、随机保持。
- 观测噪声：速度和转矩加入实测级别噪声。
- 输出惩罚：电流幅值、变化率、反向尖峰。
- 限流模型：模拟 `DrivePwrCtrl` 削顶后的闭环效果。

奖励函数建议：

```text
reward =
  - 4.0 * abs(target_velocity - current_velocity)
  - 0.03 * abs(current_cmd)
  - 0.15 * abs(current_cmd - last_current_cmd)
  - 0.50 * overshoot_penalty
  - 1.00 * oscillation_penalty
```

现有 PPO 可以保留，但第一版网络不宜过大。建议结构：

```text
input_dim -> Dense(16, tanh) -> Dense(16, tanh) -> Dense(1, tanh)
```

如果 16x16 不够，再升到 32x32。不要一开始用 64x64，因为 MCU 端调试成本更高，而且大模型更容易学到仿真细节。

### 7.4 阶段 D：真机安全微调

真机上不要让未验证 RL 策略一上来全权控制。建议用混合输出逐步放权：

```text
current_ref = (1 - alpha) * pid_current_ref + alpha * nn_current_ref
```

放权步骤：

| 阶段 | `alpha` | 目的 |
| --- | ---: | --- |
| 影子模式 | 0.0 | 只记录，不控制 |
| 小比例混合 | 0.1 | 看 NN 是否引入振荡 |
| 半控制 | 0.3 ~ 0.5 | 评估响应和功率限制 |
| 全控制 | 1.0 | 替代行进 PID |

任何阶段出现持续振荡、反向尖峰、线程超时或功率限制长期饱和，都回退到上一个 `alpha`。

## 8. 训练脚本设计

建议新增或重构为：

```text
scripts/chassis_nn/
  collect_log_format.md
  train_supervised.py
  identify_motor_model.py
  train_rl.py
  export_tflite.py
  eval_policy.py
```

### 8.1 `train_supervised.py`

输入：

```text
data/chassis_logs/*.csv
```

输出：

```text
algorithm/tflm/models/chassis/chassis_model.tflite
algorithm/tflm/models/chassis/chassis_model_data.h
algorithm/tflm/models/chassis/chassis_model_data.cc
algorithm/tflm/models/chassis/normalization.json
```

必须包含的检查：

- 数据范围统计。
- 训练集/验证集按时间段切分，不能随机打散全部样本后造成泄漏。
- 输出电流饱和比例。
- PC TFLite 推理误差。
- 随机抽样打印输入、标签、模型输出。

### 8.2 `identify_motor_model.py`

输入真机辨识日志，输出模型参数：

```json
{
  "J": 0.028,
  "B": 0.123,
  "Fc": 0.02,
  "dead_current": 0.8,
  "action_delay_steps": 2,
  "obs_delay_steps": 1
}
```

### 8.3 `train_rl.py`

在现有 `scripts/train_chassis_rl.py` 基础上改：

- 观测维度从 3 维扩展到 8 维。
- 环境中加入延迟队列。
- `reset()` 随机化 `J/B/Fc/dead_current/delay/load`。
- `step()` 中加入动作变化率惩罚。
- 导出时保存同一份归一化参数。

### 8.4 `export_tflite.py`

导出时固定：

- float32 模型优先，先保证正确性。
- 算子只使用 TFLM 已编译的 `FullyConnected` 和 `Tanh`。
- 生成 C 数组文件时使用 `alignas(16)`。
- 输出头文件只声明数组和大小，不放训练脚本生成的临时信息。

## 9. 固件集成规划

### 9.1 Kconfig

当前 `algorithm/tflm/Kconfig` 只有 `TFLM` 和 `TFLM_ARENA_SIZE`，但 `algorithm/tflm/CMakeLists.txt` 已经引用了 `CONFIG_TFLM_MODEL_CHASSIS`。需要补齐模型开关：

```text
config TFLM_MODEL_CHASSIS
    bool "Chassis drive motor NN model"
    depends on TFLM
    default n
```

`projects/thread/Kconfig` 的 `TRD_CHASSIS` 后续可以增加：

```text
select TFLM
select TFLM_MODEL_CHASSIS
```

但建议先加一个独立开关：

```text
config TRD_CHASSIS_NN_DRIVE
    bool "Use NN for chassis drive motor control"
    depends on TRD_CHASSIS
    select TFLM
    select TFLM_MODEL_CHASSIS
    default n
```

这样默认仍使用 PID，只有显式打开才启用 NN。

### 9.2 TFLM 封装 API

建议在 `algorithm/tflm/tflm.hpp` 增加面向 chassis 的窄接口：

```cpp
namespace tflm::chassis {

struct DriveInput {
    float target_velocity;
    float current_velocity;
    float velocity_error;
    float current_torque;
    float current_omega;
    float last_current_cmd;
    float target_delta;
    float limit_ratio;
};

bool init();
bool predict(const DriveInput& in, float& current_ref);

}
```

实现要点：

- `init()` 只初始化一次模型、resolver、interpreter 和 tensor arena。
- `predict()` 内部完成归一化、填 tensor、`Invoke()`、反归一化。
- 推理失败时返回 `false`，调用侧回退 PID。
- tensor arena 使用静态数组，不动态分配。
- 输入输出 tensor 维度启动时检查，不匹配则禁用 NN。

### 9.3 底盘控制替换点

在 `ControlCalculate()` 的行进段落中保留 PID 计算作为回退和影子对照：

```cpp
const float pid_torque_ref  = wheel_pid[wi].drive_velocity.Calc();
const float pid_current_ref = wheel_pid[wi].drive_torque.Calc(pid_torque_ref, snap.torque) / kTorqueK;

float nn_current_ref = pid_current_ref;
bool nn_ok = false;

#if CONFIG_TRD_CHASSIS_NN_DRIVE
nn_ok = tflm::chassis::predict(input, nn_current_ref);
#endif

const float current_ref = nn_ok
    ? Blend(pid_current_ref, nn_current_ref, g_nn_alpha)
    : pid_current_ref;

DrivePwrCtrl.SetTarget(wi, current_ref);
DrivePwrCtrl.SetMotorData(wi, snap.torque, snap.omega, target_velocity - snap.velocity);
```

注意：

- `DrivePwrCtrl.SetMotorData()` 的误差参数仍传 `target_velocity - snap.velocity`，因为功率分配需要用误差做隶属度。NN 替代 PID 后不应让功率控制失去误差信息。
- 即使全 NN 控制，也建议保留 `drive_velocity` PID 的目标和当前值更新，用于 shell 查看和回退。
- `last_current_cmd` 应记录进入功率控制前的原始需求，而不是限流后的输出；另存 `limit_ratio` 表达限流效果。

### 9.4 运行时开关

建议支持以下变量或 shell 命令：

```text
chassis nn off
chassis nn shadow
chassis nn alpha 0.3
chassis nn on
chassis nn status
```

第一版如果不做 shell，也至少在代码中保留：

```cpp
static float g_nn_alpha = 0.0f;
static bool  g_nn_enable = false;
```

上线前通过编译期开关和固定 `alpha` 控制。

## 10. 验证计划

### 10.1 PC 离线验证

每次训练输出以下报告：

- 训练集和验证集 loss。
- 电流误差 MAE/RMSE/max。
- 不同速度区间的误差。
- 目标突变片段的曲线图。
- TFLite 与 Keras/PyTorch 原模型输出差异。

最低通过标准：

| 指标 | 要求 |
| --- | --- |
| TFLite 与原模型误差 | `< 0.05A` |
| 监督模型验证 MAE | `< 1A` |
| 输出范围 | 不出现长期贴边 `+-20A` |
| 电流变化率 | 不明显大于 PID 标签 |

### 10.2 固件影子验证

固件仍由 PID 控制，NN 只推理并记录：

```text
target_velocity
current_velocity
pid_current_ref
nn_current_ref
limited_current
inference_us
loop_elapsed_ms
```

观察：

- NN 输出方向是否和 PID 一致。
- 零速附近是否抖动。
- 目标反向时是否给过大尖峰。
- `DrivePwrCtrl` 限流比例是否异常增大。
- 1ms 周期是否被推理挤爆。

### 10.3 架空轮测试

车辆离地，逐步增大 `alpha`：

1. `alpha = 0.1`
2. `alpha = 0.3`
3. `alpha = 0.5`
4. `alpha = 1.0`

只测试：

- 小速度正反转。
- 阶跃目标。
- Spin 模式。
- 零速保持。

如果离地就振荡，不进入落地测试。

### 10.4 落地低速测试

地面低速测试，先限制 `KMaxMoveVelocity` 或目标速度比例。

检查：

- 起步是否克服静摩擦。
- 停车是否残留推力。
- 转向未完全到位时行进电流是否异常。
- 电机和电调温度。
- 功率限制是否频繁介入。

### 10.5 对比测试

同一遥控输入脚本或人工动作，分别跑 PID 和 NN：

| 指标 | 说明 |
| --- | --- |
| 速度误差积分 | `sum(abs(target - velocity))` |
| 峰值电流 | 不能显著高于 PID |
| 电流变化率 | 反映平滑度 |
| 到达时间 | 阶跃响应快慢 |
| 超调量 | 反向和停车重点看 |
| 功率限制次数 | 越少越好 |

NN 只有在至少不差于 PID 的平滑性和安全性后，才值得追求更快响应。

## 11. 安全与回退

必须保留以下回退条件：

- TFLM 初始化失败：自动使用 PID。
- 推理返回失败：本帧使用 PID。
- NN 输出非有限值：本帧使用 PID。
- NN 输出超过限幅：截断到 `[-20A, 20A]` 并计数。
- 连续 N 帧线程超时：关闭 NN。
- 速度误差持续变大：降低 `alpha` 或关闭 NN。
- 遥控器失联或进入安全模式：不允许 NN 继续输出非零电流。

建议限幅：

```cpp
current_ref = clamp(current_ref, -20.0f, 20.0f);
current_ref = slew_limit(current_ref, last_current_ref, max_delta_per_ms);
```

`slew_limit` 的第一版可以只用于 NN 输出，避免模型在真机上给尖峰。

## 12. 分阶段里程碑

### M1：文档和日志格式确认

产物：

- 本文档。
- 日志字段定义。
- 采集固件开关设计。

通过标准：

- 明确 NN 替换点。
- 明确保留功率控制和 PID 回退。

### M2：PID 影子模型

产物：

- `train_supervised.py`
- `chassis_model_data.cc/h`
- `tflm::chassis::predict()`
- 固件影子日志

通过标准：

- 固件能 1ms 内完成推理。
- NN 输出与 PID 大体一致。
- 不控制电机时不会影响原控制。

### M3：混合控制

产物：

- `alpha` 混合控制。
- NN/PID 对比日志。
- 架空轮测试结果。

通过标准：

- `alpha <= 0.5` 不振荡。
- 输出方向、限幅、回退可靠。

### M4：改良模型

产物：

- 电机辨识参数。
- 改良监督或 RL 模型。
- 落地低速测试报告。

通过标准：

- 小速度起步和停车优于 PID 或至少不差。
- 外扰恢复不振荡。

### M5：全 NN 行进控制

产物：

- `CONFIG_TRD_CHASSIS_NN_DRIVE=y` 下可全权替代行进 PID。
- PID 仍可编译保留为回退。

通过标准：

- 全速度范围稳定。
- 长时间运行无线程超时。
- 功率限制介入后仍稳定。

## 13. 推荐实施顺序

1. 补齐 `CONFIG_TFLM_MODEL_CHASSIS` 和 `TRD_CHASSIS_NN_DRIVE` 开关。
2. 先做日志采集，不改控制输出。
3. 训练监督模型拟合 PID。
4. 实现 `tflm::chassis::predict()`。
5. 在 `ControlCalculate()` 中做影子推理和日志。
6. 加 `alpha` 混合控制，默认 `alpha = 0`。
7. 架空轮从 `alpha = 0.1` 开始验证。
8. 采集真机辨识数据，重训更稳的模型。
9. 落地低速测试。
10. 再考虑全 NN 控制。

这条路线的重点是把 NN 当成一个可替换的行进电流生成器，而不是一次性把底盘控制全端到端化。这样既能利用现有 TFLM 和训练脚本，也能保留当前功率控制、安全边界和 CAN 输出链路。
