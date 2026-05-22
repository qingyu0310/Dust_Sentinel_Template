# TFLM Robot Firmware

基于 Zephyr RTOS 的机器人控制固件，当前主要覆盖遥控接收、底盘控制、CAN 电机发送、GPIO/LED 状态指示等模块。项目同时支持普通 Zephyr 板级工程结构和 HPM5361ICB 的项目侧 overlay / SDK glue 适配。

## 总体数据流

```text
Remote(UART DMA) -> zbus(pub_remote_to) -> Chassis/Gimbal
                                         -> 控制算法
                                         -> zbus(to_can_tx)
                                         -> CAN_TX -> DJI 电机

GPIO thread -> 运行状态指示 / WS2812B
```

主要线程：

| 线程 | 触发方式 | 职责 |
| --- | --- | --- |
| Remote | UART RX 信号量唤醒 | 接收 UART 字节流、识别遥控协议、发布 `remote_to` |
| Chassis | 周期运行 | 底盘运动学、PID、功率控制、CAN 目标发布 |
| Gimbal | 周期运行 | 云台控制与遥控输入消费 |
| CAN_TX | zbus / 队列驱动 | 发送电机 CAN 帧 |
| GPIO | 周期运行 | LED / WS2812B 状态指示 |

## Remote 模块

Remote 模块位于 `modules/remotes`，对外发布稳定的 `topic::remote_to::Message`。上层消费者不需要知道具体遥控器类型。

当前支持协议：

| 协议 | 帧长 | 识别特征 |
| --- | --- | --- |
| DR16 | 18B | 通道范围 + 开关值校验，无固定帧头 |
| VT12 | 16B | `0xA5` 帧头，后续可补强长度 / cmd / crc8 |
| VT13 | 21B | `0xA9 0x53` 帧头 + 通道范围，后续可补强 crc16 |

`RemoteType::Auto` 会从 UART 字节流中自动识别协议。固定选择 `DR16` / `VT12` / `VT13` 时，只表示启动时优先锁定该协议；运行中如果连续失配或断连，仍会回到探测状态，允许中途更换遥控器。

协议模块只负责：

- `validate()`：判断当前窗口是否是合法帧，不发布数据。
- `decode()`：把合法帧写入 `topic::remote_to::Message`，不直接发布。

`Remote` 主循环负责：

- UART 碎片数据拼帧。
- 滑动窗口同步。
- 协议探测与锁定。
- 断连超时归零。
- 统一 `zbus_chan_pub()`。

## HPM5361ICB 注意事项

HPM5361ICB 的项目侧板级配置在：

```text
projects/boards/hpm/hpm5361icb/hpm5361icb.overlay
```

remote UART 使用 `uart-remote = &uart4`，当前波特率为 `100000`。UART4 + DMA + RX idle 的底层适配历史和排查记录见：

```text
doc/hpm5361icb_porting_record.md
```

在 HPM UART RX idle 没完全稳定前，Remote 的 DMA `buf_size` 不宜设置过大。当前使用小分片唤醒线程，由 `Remote::frame_buf_` 跨 chunk 拼完整帧，避免 128 字节 DMA buffer 带来的遥控延迟。

项目自有外设、别名、pinmux 优先放在项目 overlay 中，避免把应用侧连接写进共享 board DTS。

## 目录结构

```text
algorithm/      控制算法、滤波、辨识、功率控制
drivers/        项目封装的 UART / CAN / GPIO 等设备接口
modules/        遥控器、电机、LED 等功能模块
projects/       应用线程、板级 overlay/conf、项目入口组织
topic/          zbus topic 定义
doc/            移植记录、架构说明、问题分析
src/            Zephyr 应用入口
```

## 构建入口

项目通过 `BOARD_CFG` 选择项目侧板级配置：

```bash
west build -b <board> -- -DBOARD_CFG=<config>
```

如果未指定 `BOARD_CFG`，CMake 会默认使用 `BOARD` 名称作为配置分组。

HPMicro SDK glue 默认路径：

```text
D:/Zephyr_HPMicro/sdk_glue
```

也可以通过环境变量覆盖：

```bash
SDK_GLUE_DIR=<path>
```

## 开发约定

- `topic::remote_to::Message` 保持为控制语义接口，不暴露具体遥控器类型。
- 新增遥控协议时，优先新增协议模块的 `validate()` / `decode()`，再把协议加入 `remote.cpp` 的协议表。
- HPM5361ICB 的 UART4/DMA 问题优先从 overlay、`doc/hpm5361icb_porting_record.md` 和 SDK glue UART 驱动查起。
- WS2812B 在 HPM5361ICB 上走直接 HPM GPIO 快速翻转路径，不走 Zephyr PWM/GPIO 包装路径。
