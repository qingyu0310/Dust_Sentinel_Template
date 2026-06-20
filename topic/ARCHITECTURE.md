# topic/ 架构说明

## 职责

线程间数据通道。定义 zbus 消息结构和通道声明，用于模块间解耦数据交换。

## 边界

| 管 | 不管 |
|----|------|
| 定义消息结构体（`topic::xxx::Message`） | 不包含业务逻辑 |
| 声明 zbus 通道和监听者 | 不持有设备实例 |
| 通过 Kconfig 开关控制编译 | 不创建线程 |

## 目录结构

```
topic/
├── imu_to/
├── remote_to/
├── to_can_tx/
├── CMakeLists.txt
└── Kconfig
```

imu_to/
    IMU 姿态数据通道。发布四元数、角速度、姿态角和温度。

remote_to/
    遥控器数据通道。发布遥控器通道值和开关状态。

to_can_tx/
    CAN 发送数据通道。发布待发送的 CAN 帧数据。

## 文件规范

每个通道一个子目录，包含：

| 文件 | 内容 |
|------|------|
| `xxx.hpp` | 消息结构体定义 + `ZBUS_CHAN_DECLARE` / `ZBUS_OBS_DECLARE` |
| `xxx.cpp` | `ZBUS_CHAN_DEFINE` / `ZBUS_OBS_DEFINE` |

规则：
- hpp 中定义 `namespace topic::xxx { struct Message { ... }; }`
- hpp 末尾声明 `ZBUS_CHAN_DECLARE` 和 `ZBUS_OBS_DECLARE`
- cpp 中定义 `ZBUS_CHAN_DEFINE` 和 `ZBUS_OBS_DEFINE`
- 在 `topic/Kconfig` 和 `topic/CMakeLists.txt` 中注册编译条目

## 依赖关系

数据通道可选两种机制：
- **zbus**（`imu_to`、`remote_to`）：依赖 Zephyr `CONFIG_ZBUS`，支持发布/订阅模式
- **消息队列**（`to_can_tx`）：使用 `k_msgq`，适用于点对点帧数据传递

通过 Kconfig `select ZBUS` 开启 zbus 支持。

## 调用方

- `modules/` 和 `projects/thread/` 中的代码通过 `zbus_chan_pub` 和 `zbus_chan_sub` 读写通道。
