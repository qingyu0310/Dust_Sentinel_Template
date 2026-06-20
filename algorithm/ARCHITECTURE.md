# algorithm/ 架构说明

## 职责

纯计算模块。提供控制器（PID、功率控制）、滤波器（LPF、HPF、Kalman、EKF）、参数辨识（RLS）等算法封装。

## 边界

| 管 | 不管 |
|----|------|
| 数学计算、状态更新、配置管理 | 不创建线程 |
| 纯数据输入输出 | 不持有硬件外设句柄（PWM、UART 等） |
| 编译期可选的独立模块 | 不依赖 `modules/`、`projects/`、`topic/` |

## 目录结构

```
algorithm/
├── controller/         
├── filter/
├── identify/
├── tflm/
├── ARCHITECTURE.md
├── CMakeLists.txt
└── Kconfig
```

controller/
    控制算法。提供 PID 控制器、功率控制器、软件定时器等功能。
    负责根据传感器反馈和目标值计算控制输出，是底盘、云台等执行机构的核心运算层。

filter/
    信号滤波。提供低通滤波、高通滤波、卡尔曼滤波、四元数姿态 EKF 等。
    对传感器原始数据进行去噪和状态估计，输出平滑可靠的测量值供控制器使用。

identify/
    参数辨识。提供递归最小二乘法（RLS）等在线辨识算法。
    用于系统模型参数的自适应估计，如功率模型的电阻和转矩系数辨识。

tflm/
    TensorFlow Lite Micro 第三方移植。提供嵌入式设备上运行轻量级神经网络推理的能力。

## 文件规范

新增算法时在对应分类下建新目录，包含：

| 文件 | 内容 | 是否必需 |
|------|------|----------|
| `xxx.hpp` | 类声明。header-only 模块放完整实现；有 cpp 的只放声明 | 是 |
| `xxx.cpp` | 非内联方法实现 | 有非内联方法时需要 |

规则：
- 不包含 `namespace thread::xxx`、不定义 `k_thread`、不持有 `static` 设备实例
- 线程无关，纯数据输入输出
- 在 `algorithm/CMakeLists.txt` 和 `algorithm/Kconfig` 中注册编译条目

## 依赖关系

```
quaternion_ekf → kalman（基类）
power_ctrl    → pid + rls + lpf
```

各模块之间通过 Kconfig `select` 表达依赖。除此之外无耦合。

## 调用方

- `projects/thread/` 中的线程代码实例化并调用算法类
- 不反向依赖线程层
