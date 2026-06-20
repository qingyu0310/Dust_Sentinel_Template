# modules/ 架构说明

## 职责

硬件设备模块的封装层。应用层直接声明该层的类使用。对于能独立工作的模块，允许配备独立 task 和 drivers 外设；对于需要多模块配合的，task 延迟放到 `projects/thread/` 处理。

## 边界

| 管 | 不管 |
|----|------|
| 封装具体设备管理器类，对外提供完整接口 | 不直接操作硬件寄存器 |
| 组合 `drivers/` 外设 + `algorithm/` 算法 | 不定义 `namespace thread::xxx` |
| 类内部维护 `ready_` 就绪状态和防呆检查 | 不管理线程生命周期 |
| 通过 Kconfig `select` 表达依赖 | 多从设备总线共享由独立通讯线程处理 |

规则：
- **独立模块**：能独立工作的模块，允许直接配备 task 线程和 drivers 外设
- **多模块配合**：需要多模块协同工作时，task 延迟到 `projects/thread/` 层处理
- **多从设备通讯**：总线上挂载多个从设备时，不允许在模块内单独添加通讯外设，需建立独立通讯线程共享资源

## 目录结构

```
modules/
├── imu/
├── motors/
├── powermeter/
├── remotes/
├── CMakeLists.txt
└── Kconfig
```

imu/
    惯性测量单元。提供采样、加热控温、姿态解算功能。

remotes/
    遥控器接收机。通过 UART DMA 接收遥控数据，支持 DR16、VT12、VT13 协议识别和解码。
    
motors/
    电机驱动。封装 DJI 和 DM 等品牌电机的通讯协议和控制接口。

powermeter/
    功率计。读取功率数据，用于底盘功率分配和控制。

## 文件规范

每个设备模块一个子目录，包含：

| 文件 | 内容 |
|------|------|
| `xxx.hpp` | 类声明 + 数据结构 |
| `xxx.cpp` | 非内联方法实现 |

规则：
- 类声明在 hpp，实现在 cpp
- 需要线程包装的类，提供 `Start()` 方法并在内部做 `ready_` 防呆检查
- 在 `modules/Kconfig` 和 `modules/CMakeLists.txt` 中注册编译条目

## 依赖关系

模块通过 Kconfig `select` 依赖 `drivers/` 和 `algorithm/` 中的组件。

## 调用方

- `projects/thread/` 中的线程代码实例化模块类并调用
- 应用层可直接声明使用该层类
