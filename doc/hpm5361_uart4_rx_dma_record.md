# HPM5361 UART4 RX DMA 调试记录

日期：2026-05-21

## 背景

HPM5361ICB 上 DR16 使用 UART4，现象是接收不稳定，最开始表现为只触发一次或不触发。STM32 板子使用同一套应用层逻辑测试正常，因此问题收敛到 HPM5361 的 UART4 板级配置和 `sdk_glue` UART driver。

注意：`D:\Zephyr_HPMicro\sdk_glue\boards\hpmicro\hpm5361icb` 下的板级代码可信度较低，排查时优先参考 `temp` 下官方 SDK 和本仓库 `doc/hpm5361icb_porting_record.md` 中的移植记录。

## 当前有效改动

### `projects/boards/hpm/hpm5361icb/hpm5361icb.overlay`

UART4 保持最小标准配置，不再使用 `TRGM + GPTMR` 软件 idle 检测属性，也不再覆盖 PA17 的 BIOC pinmux。

当前 UART4 关键配置：

```dts
&uart4 {
	current-speed = <100000>;
	pinctrl-0 = <&pinmux_uart4_rx>;
	pinctrl-names = "default";
	dmas = <&hdma 2 0x1C>, <&hdma 3 0x1D>;
	dma-names = "rx", "tx";
	status = "okay";
};
```

保留 `rx + tx DMA`，没有继续使用“只开 RX DMA”的特殊路径。

### `D:\Zephyr_HPMicro\sdk_glue\drivers\serial\uart_hpmicro.c`

HPM5361 改为优先走 UART 外设自带的硬件 RX idle 检测：

```c
#if defined(CONFIG_SOC_HPM5361) && defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
#define UART_HPM_USE_HW_RX_IDLE 1
#else
#define UART_HPM_USE_HW_RX_IDLE 0
#endif
```

`TRGM + GPTMR` 相关 include 只在没有硬件 RX idle 时启用：

```c
#if defined(CONFIG_UART_ASYNC_API) && !UART_HPM_USE_HW_RX_IDLE
#include <hpm_trgmmux_src.h>
#include <hpm_trgm_drv.h>
#include <hpm_gptmr_drv.h>
#endif
```

UART 初始化时打开硬件 RX idle：

```c
uart_config.rxidle_config.detect_enable = false;
uart_config.rxidle_config.detect_irq_enable = true;
uart_config.rxidle_config.idle_cond = uart_rxline_idle_cond_state_machine_idle;
uart_config.rxidle_config.threshold = 20U;
```

当前仍保留临时调试打印：

```text
hpm uart: callback_set ...
hpm uart: rx_enable ...
hpm uart: dma rx cb ...
```

### `drivers/communication/uart/uart.cpp`

`UartDma` 的 RX buffer 处理做了收紧：

- `UART_RX_BUF_REQUEST` 使用实际 `dma_buf_size_` 回复 buffer。
- `UART_RX_DISABLED` 固定回到 `buf0` 重新 `uart_rx_enable()`。
- 重新 enable 失败时把 `ready_` 置为 `false`。
- 增加 `UART_RX_STOPPED` 分支，避免事件落到未知分支。

### `drivers/communication/uart/uart.hpp`

`UartDma` 新增实际 DMA buffer 大小记录：

```cpp
uint16_t dma_buf_size_ = 0;
```

### `modules/remotes/remote.cpp`

DR16 接收线程被唤醒后不再只读一次 `32` 字节，而是循环读空 UART 环形缓冲区：

```cpp
while (true) {
    uint16_t n = uart_->Read(tmp, sizeof(tmp));
    if (n == 0) {
        break;
    }

    ProcessChunk(tmp, n);
}
```

这一步解决了 HPM5361 侧一次 DMA 回调返回 `128` 字节时，上层只消费前 `32` 字节导致看起来“只打印一次”的问题。

当前 DR16 UART 配置：

```cpp
cfg.buf_size = 128;
cfg.rx_timeout = 200;
```

### `modules/remotes/dr16/dr16.cpp`

保留临时调试打印，用来确认 DR16 解包是否持续运行：

```cpp
printk("%d,%d\n", ch0, ch1);
```

## 测试现象

当前用户侧日志：

```text
hpm uart: callback_set serial@f0050000 cb=0x800048c4 user=0x86ab8
hpm uart: rx_enable serial@f0050000 len=128 timeout=200 rxch=2 idle=200
hpm uart: dma rx cb dev=hdma@f00c8000 ch=2 mode=1 off=0 total=128
1024,1024
1024,1024
1024,1024
1024,1024
1024,1024
1024,1024
1024,1024
```

结论：

- 应用层 `remote/dr16` 链路已经通，数据能持续接收和解析。
- 当前现象更像是 DMA buffer 满 `128` 字节后回调，而不是硬件 RX idle 中断在分帧。
- 卡顿应继续从 HPM5361 UART 底层 RX idle / DMA 回调路径查，不应靠改应用层 buffer 规避。

## 后续清理建议

确认长时间运行稳定后，可以移除这些临时打印：

- `D:\Zephyr_HPMicro\sdk_glue\drivers\serial\uart_hpmicro.c` 中的 `hpm uart: ...`
- `modules/remotes/dr16/dr16.cpp` 中的 `printk("%d,%d\n", ch0, ch1);`

本记录只保存改动位置和调试结论，不包含编译结果。
