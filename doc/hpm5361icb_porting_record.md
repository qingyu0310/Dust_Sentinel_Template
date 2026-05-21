# HPM5361ICB Zephyr 移植记录

日期：2026-05-20

本文记录 HPM5361ICB 自制核心板适配 Zephyr/HPMicro `sdk_glue` 的排查过程、遇到的问题、最终修正点，以及后续再开外设时需要注意的限制。

这次结论很明确：应用层 remote 代码本身不是第一嫌疑，真正的问题在 HPM5361ICB 的板级/芯片适配层，尤其是 UART4 + DMA + RX idle 的底层实现。

## 1. 背景

工程目录：

```text
D:\Zephyr\projects\tflm
```

HPMicro `sdk_glue` 目录：

```text
D:\Zephyr_HPMicro\sdk_glue
```

HPM5361ICB 板级目录：

```text
D:\Zephyr_HPMicro\sdk_glue\boards\hpmicro\hpm5361icb
```

项目事实：

- 原生 Zephyr 没有 HPM5361ICB 这块板。
- `D:\Zephyr_HPMicro\sdk_glue\boards\hpmicro\hpm5361icb` 最早由 AI 改写出来，可信度低。
- 同一套应用代码在 ST 和 HPM6E00 官方开发板上能跑。
- HPM6E00 EVK 是官方 SDK 适配，HPM5361ICB 是自制核心板。
- 所以排查方向最终收敛到 board/soc/dts/openocd/pinctrl/clock/irq/dma/uart 层，而不是优先改 Zephyr API 或 remote 应用逻辑。

标准构建命令：

```powershell
.\cmd\build\build.bat hpm5361icb -p
```

普通增量构建命令：

```powershell
.\cmd\build\build.bat hpm5361icb
```

烧录命令：

```powershell
west flash
```

## 2. 最初现象

### 2.1 烧录不稳定

典型错误：

```text
Error: [hpm5361.cpu0] Unable to halt. dmcontrol=0x00000001, dmstatus=0x00400ca2
Error: [hpm5361.cpu0] Fatal: Hart 0 failed to halt during examine
Error: [hpm5361.cpu0] Unsupported DTM version: -1
Error: [hpm5361.cpu0] Could not identify target type.
```

偶发 JTAG 扫链错误：

```text
JTAG scan chain interrogation failed: all ones
hpm5361.cpu: IR capture error
dtmcs.abits is zero. Check JTAG connectivity/board power
Unsupported DTM version: 15
```

观察到的规律：

- GPIO only 可以反复烧录。
- 开 remote 后，第一次可能能烧进去，第二次就经常 halt/examine 失败。
- 降速到 500 kHz 后有所改善，但没有根治。
- BOOT 全部悬空或切到 ISP/串行启动时，能恢复烧录。

这个现象容易误判为纯硬件、BOOT strap 或 JTAG 时序问题。但后面验证表明：remote 触发的 UART+DMA 底层问题会把芯片带到一种 OpenOCD 难以 halt 的状态，所以烧录失败是结果，不是根因。

### 2.2 Debug 不稳定

典型表现：

```text
Cannot insert breakpoint.
Cannot access memory at address 0x8000xxxx
```

以及：

```text
Failed to write 4-byte breakpoint instruction at 0x80003b18
can't add breakpoint: unknown reason
```

原因拆分：

- HPM5361 当前从 XIP flash 运行，普通软件断点需要改写 flash 指令，OpenOCD/GDB 很容易失败。
- 应尽量使用硬件断点 `hbreak`，并设置硬件断点数量。
- VSCode Step Over 也可能临时插入断点，因此在 XIP 区域会失败。

建议调试配置：

```gdb
set mem inaccessible-by-default off
mem 0x80000000 0x80100000 ro 32 nocache
mem 0x00080000 0x000a0000 rw 32 nocache
set remote hardware-breakpoint-limit 4
set breakpoint auto-hw on
```

### 2.3 Fatal Handler 捕获到异常

早期曾捕获到：

```text
mcause = 0x5
mtval  = 0x08080f98
```

其中：

```text
&slice_timeouts = 0x80f98
```

这说明内核 timeout/list 附近的数据被错误访问，表象像 Zephyr 调度或线程坏了。后面确认不是 Zephyr 调度器本身有问题，而是底层 UART/DMA/中断路径导致内存或状态异常。

### 2.4 线程看似卡在 idle

GDB 常停在：

```c
arch_cpu_idle()
```

或者 Halt Only 停在：

```c
gpio_hpm_port_isr()
```

这些点不能直接说明程序坏了：

- 停在 `arch_cpu_idle()` 可能只是当前没有 ready 线程。
- `mcause = 0x80000007` 是 machine timer interrupt，说明 tick 在来。
- Halt Only 只是随机打断 CPU，停在 ISR 或 idle 都可能是正常现象。

真正要看的是线程有没有继续跑、WS2812B 是否持续变色、remote 是否触发后系统还能烧录/调试。

## 3. 排查中遇到的问题

### 3.1 BOOT 模式影响烧录

现象：

- Flash boot 模式下，开 remote 后容易第二次烧录失败。
- 切到 ISP/串行启动，或者让 BOOT 悬空后，OpenOCD 更容易重新拿住芯片。

解释：

- 错误固件一上电就运行，可能很快进入异常、错误中断或外设死循环。
- OpenOCD reset 后需要 halt core，如果程序把底层状态搞乱，就会出现 halt/examine 失败。
- BOOT 进入 ISP 后，用户固件不运行，OpenOCD 更容易接管。

结论：

- BOOT 问题是恢复手段，不是最终根因。
- 最终还是要修 board/uart/dma 层。

### 3.2 OpenOCD 配置问题

曾经出现：

```text
Can't find interface/cmsis-dap.cfg
```

原因：

- 本地板级目录里手写了 OpenOCD `cmsis_dap.cfg`，但 runner 的 search path 里找不到 `interface/cmsis-dap.cfg`。
- 后面改为直接引用 HPM SDK 自带 OpenOCD 配置，而不是自己复制一份半成品 cfg。

当前策略：

- 使用 SDK 自带 OpenOCD 配置。
- `adapter speed` 固定到 500 kHz。
- 保留 `support` 目录，避免 Zephyr openocd runner 因 support path 不存在报错。

板级 runner 参数：

```cmake
macro(app_set_runner_args)
  board_runner_args(openocd "--cmd-pre-init=adapter speed 500")
endmacro()
```

### 3.3 board 重名问题

曾出现：

```text
Error finding board: hpm5361icb
Board(s): {'hpm5361icb'}, defined multiple times.
Last defined in ... hpm5361icb.ai_backup_20260520\board.yml
```

原因：

- 备份目录还放在 `boards/hpmicro` 下。
- Zephyr 会递归扫描 board root，备份里的 `board.yml` 也会被识别。

处理：

- 备份目录移出 `boards/hpmicro`，放到不会被 Zephyr 扫描的 backup 路径。

### 3.4 DTS parse error

曾出现：

```text
devicetree error: hpm5361icb.dts:31: parse error: expected number or parenthesized expression
```

原因：

- DTS 语法不符合 dtc 要求。
- AI 生成的板级 DTS 里存在多处不可靠写法。

处理：

- 重新整理 `hpm5361icb.dts` 和 `hpm5361icb-pinctrl.dtsi`。
- 对照官方 HPM SDK/Zephyr 适配写法。

### 3.5 PMA/PMP 配置错误

曾出现：

```text
implicit declaration of function 'pma_config_attributes'
undefined reference to `pma_config_attributes'
```

原因：

- HPM5361 对应特性里没有按当前方式启用 PMA。
- 盲目照搬其他 HPM 芯片的 PMA 初始化会导致编译或链接失败。

处理原则：

- HPM5361 不应强行套用不匹配的 PMA 初始化。
- 以内置 SoC feature 和官方 SDK 当前支持为准。

### 3.6 PCFG/DCDC 链接问题

曾出现：

```text
undefined reference to `pcfg_dcdc_set_voltage'
```

原因：

- `soc.c` 使用了 PCFG 相关函数，但当前 Kconfig/CMake 没有正确拉入对应 SDK 源。

处理原则：

- SoC clock/voltage 初始化必须和当前 HPM5361 SDK 源匹配。
- 不能只复制官方 board 初始化片段而不检查链接依赖。

### 3.7 `.stack` 和 `.bss` 重叠

曾出现：

```text
section .stack VMA [0000000000080300,00000000000806ff] overlaps section bss VMA [0000000000080000,00000000000810f3]
```

原因：

- RAM/linker/memory region 配置不一致。
- 之前移植过程中 DLM/ILM/ITCM/DTCM/ROM 区域和 Zephyr linker 预期没有完全对齐。

处理原则：

- HPM5361 的 `zephyr,sram` 指向 DLM。
- XIP flash 指向 `flash0`。
- `zephyr,itcm` 指向 ILM。
- 不随意改 stack 起始地址来掩盖内存布局问题。

## 4. WS2812B/GPIO 验证线

HPM5361ICB 上的 `led_r` 实际是 WS2812B，不是普通 GPIO LED。

最终采用 `modules/leds/ws2812b.hpp` 中的专用 bitbang 驱动来验证代码是否真的在线程中运行。

关键点：

- WS2812B 接在 PA10。
- DTS 中 `led_alert` 使用 `gpioa 10`。
- 有效电平为高电平。
- 使用 GPIO API 不足以稳定驱动 WS2812B，因此改为 HPM GPIO 寄存器快速翻转。
- `led_r.set(colors[phase++ % ARRAY_SIZE(colors)])` 能持续变色，说明线程调度、tick、GPIO 输出基本正常。

当前 overlay 中：

```dts
led_alert: led_alert {
    gpios = <&gpioa 10 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
    label = "LED_RED";
};
```

GPIO 线的意义：

- 如果只开 GPIO/WS2812B 能持续变色，说明内核 tick 和线程没有根本问题。
- 如果一开 remote 就不变色、烧录困难，则问题高度集中到 UART+DMA。

## 5. remote 问题的定位过程

逐步打开线程后确认：

- 只开 GPIO：可运行，可反复烧录。
- 打开其他不用 UART+DMA 的线程：基本可运行。
- 打开 remote：线程不正常，烧录也容易失败。
- `rx_timeout=0` 时现象能被绕开。
- 其他开发板上 remote 正常，说明 remote 协议和 Zephyr API 不是首要问题。

因此结论：

```text
HPM5361ICB 的 UART + DMA + RX idle 底层适配有问题。
```

`rx_timeout=0` 为什么能绕开：

- 旧实现依赖 GPTMR/TRGM 做 UART RX idle 检测。
- 当 timeout 被设成 0 或相关路径不触发时，错误的 idle 检测链路被绕开。
- 这不是修复，只是避开了坏路径。

## 6. UART4 + DMA 根因

### 6.1 旧设计的问题

旧的 `uart_hpmicro.c` 异步接收 idle 检测逻辑依赖：

- UART RX 引脚信号
- TRGM input
- GPTMR capture/compare
- 单独的 idletimer IRQ

这套方式对某些 HPM 芯片可能成立，但套在 HPM5361ICB 上不可靠。

主要问题：

- HPM5361 支持 UART 硬件 RX idle detect，不需要 GPTMR/TRGM 模拟。
- UART4 RX 使用 PA17 ALT2。
- PA17 如果作为 UART4_RXD，就不应再假设它同时作为 GPTMR/TRGM capture 输入。
- 之前 board DTS 里写了 `uart-idle-trgm-*`、`uart-idle-gptmr-*` 一类属性，本质是在强行给 UART idle 拼外部路径。
- 部分 pinmux 还混入了不存在或不适用于 HPM5361 的 BIOC 思路。

最终判断：

```text
HPM5361 的 remote UART 接收应该走 UART 外设自己的 RX idle interrupt，而不是 GPTMR/TRGM 软件 idle。
```

### 6.2 官方 SDK 依据

从 HPM5361 SDK 特性确认：

```c
#define HPM_IP_FEATURE_UART_RX_IDLE_DETECT 1
#define HPM_IP_FEATURE_UART_E00018_FIX 1
```

含义：

- UART 外设自带 RX idle 检测。
- idle flag 相关读取/清除需要按 SDK 当前 UART driver 的 E00018 fix 路径处理。

SDK 示例 `uart_hardware_rx_idle` 的思路：

- `rxidle_config.detect_enable = true`
- `rxidle_config.detect_irq_enable = true`
- idle 条件使用 RX line logic one
- threshold 设一个合理值

### 6.3 DMA request source

HPM5361 UART4 DMA request：

```text
UART4 RX = 0x1C
UART4 TX = 0x1D
```

当前 DTS：

```dts
&uart4 {
    status = "disabled";
    current-speed = <100000>;
    pinctrl-0 = <&pinmux_uart4_rx>;
    pinctrl-names = "default";
    dmas = <&hdma 2 0x1C>, <&hdma 3 0x1D>;
    dma-names = "rx", "tx";
};
```

overlay 中开启：

```dts
&hdma {
    status = "okay";
};

&uart4 {
    status = "okay";
};
```

remote alias：

```dts
aliases {
    user-can1 =  &mcan0;
    uart-remote = &uart4;
};
```

## 7. 最终关键修改

### 7.1 UART driver：HPM5361 使用硬件 RX idle

文件：

```text
D:\Zephyr_HPMicro\sdk_glue\drivers\serial\uart_hpmicro.c
```

新增选择逻辑：

```c
#if defined(CONFIG_SOC_HPM5361) && defined(HPM_IP_FEATURE_UART_RX_IDLE_DETECT) && (HPM_IP_FEATURE_UART_RX_IDLE_DETECT == 1)
#define UART_HPM_USE_HW_RX_IDLE 1
#else
#define UART_HPM_USE_HW_RX_IDLE 0
#endif
```

效果：

- HPM5361 走 UART 外设硬件 RX idle。
- 非 HPM5361 仍保留旧的 GPTMR/TRGM 软件 idle 路径。
- 避免影响 HPM6E00 等已经正常的官方板。

### 7.2 UART init：配置硬件 RX idle

HPM5361 async UART 初始化中加入：

```c
uart_config.rxidle_config.detect_enable = false;
uart_config.rxidle_config.detect_irq_enable = true;
uart_config.rxidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
uart_config.rxidle_config.threshold = 20U;
```

注意：

- init 时先不打开 detect。
- `rx_enable()` 后再清 flag 并打开 detect。
- 避免未准备好 DMA buffer 时先收到 idle IRQ。

### 7.3 UART ISR：处理硬件 RX idle

HPM5361 下 UART IRQ 中增加：

```c
if (uart_is_rxline_idle(base)) {
    uart_clear_rxline_idle_flag(base);
    uart_hpm_hw_rx_idle_handler(dev);
}
```

idle handler 做的事情：

- 判断 RX DMA 是否正在运行。
- suspend RX DMA。
- 调用 `uart_hpm_async_rx_flush()` 释放已收到的数据。
- resume RX DMA。

这就是 remote 能重新动起来的核心修改。

### 7.4 删除 DTS 中伪 idle timer 配置

从 `hpm5361icb.dts` 删除：

```text
uart-idle-trgm-*
uart-idle-gptmr-*
```

原因：

- HPM5361 不需要 GPTMR/TRGM 模拟 UART idle。
- UART4_RXD PA17 不应该再被当作 GPTMR/TRGM 输入复用。
- 错误 idle path 会引发线程异常、DMA 状态异常、烧录后难以 halt。

### 7.5 Kconfig：async UART 自动拉起 IRQ

文件：

```text
D:\Zephyr_HPMicro\sdk_glue\drivers\serial\Kconfig.hpmicro
```

关键修改：

```kconfig
select UART_INTERRUPT_DRIVEN if UART_ASYNC_API
select HAS_HPMSDK_GPTMR if (UART_ASYNC_API && !SOC_HPM5361)
select HAS_HPMSDK_DMA if (UART_ASYNC_API && DMA_HPMICRO)
select HAS_HPMSDK_DMAV2 if (UART_ASYNC_API && DMAV2_HPMICRO)
select DMA if UART_ASYNC_API
```

意义：

- UART async 接收 idle 事件需要 UART IRQ。
- HPM5361 不再为 UART async 强行拉 GPTMR。
- DMA 只在实际启用 async API 时拉起，而不是因为 driver 支持 async 就误开。

### 7.6 修正 UART DMA TX/RX 边界问题

在 `uart_hpmicro.c` 中还修了几个隐患：

- TX flush/abort 使用 `data->dma_tx.dma_dev`，不能误用 UART device 指针。
- RX flush 中如果 `dma_get_status()` 失败，不继续用未定义状态。
- RX flush 增加边界检查，避免 `pending_length` 大于剩余长度导致 unsigned underflow。

这些不是最核心根因，但会放大 UART+DMA 的不稳定。

## 8. 当前 HPM5361ICB 项目配置

文件：

```text
D:\Zephyr\projects\tflm\projects\boards\hpm\hpm5361icb\hpm5361icb.conf
```

当前策略：

```conf
CONFIG_MCAN_HPMICRO=n
CONFIG_DMAV2_HPMICRO=y
CONFIG_SERIAL=y
CONFIG_UART_ASYNC_API=y
CONFIG_UART_CONSOLE=n
CONFIG_CONSOLE=n
CONFIG_PRINTK=n
CONFIG_LOG=n
CONFIG_BOOT_BANNER=n
CONFIG_EXCEPTION_DEBUG=n
CONFIG_EXCEPTION_STACK_TRACE=n
CONFIG_THREAD_NAME=n
CONFIG_ASSERT_VERBOSE=n
CONFIG_CAN=n
CONFIG_DMA=y
CONFIG_TRD_REMOTE=y
CONFIG_TRD_CAN_TX=n
CONFIG_ROM_START_OFFSET=0x3100
CONFIG_HW_STACK_PROTECTION=n
```

说明：

- 当前只打开 remote 所需的 UART4 + DMA。
- CAN 暂时关闭，避免把 MCAN 问题和 UART 问题混在一起。
- printk/log/console 关闭，因为 UART 底层正在验证阶段，不让 console 抢 UART 或引入额外变量。

## 9. 当前 overlay 要点

文件：

```text
D:\Zephyr\projects\tflm\projects\boards\hpm\hpm5361icb\hpm5361icb.overlay
```

关键点：

```dts
aliases {
    user-can1 =  &mcan0;
    uart-remote = &uart4;
};
```

```dts
&hdma {
    status = "okay";
};

&mcan0 {
    status = "disabled";
};

&uart4 {
    status = "okay";
};
```

说明：

- `uart-remote` 已挂到 `uart4`。
- `user-can1` 先保留 alias，后续开 CAN 时应用层不用再改名字。
- `mcan0` 当前 disabled，因为本阶段只验证 remote。

## 10. 当前板级 DTS 要点

文件：

```text
D:\Zephyr_HPMicro\sdk_glue\boards\hpmicro\hpm5361icb\hpm5361icb.dts
```

UART4：

```dts
&uart4 {
    status = "disabled";
    current-speed = <100000>;
    pinctrl-0 = <&pinmux_uart4_rx>;
    pinctrl-names = "default";
    dmas = <&hdma 2 0x1C>, <&hdma 3 0x1D>;
    dma-names = "rx", "tx";
};
```

UART4 pinctrl：

```dts
pinmux_uart4_rx: pinmux_uart4_rx {
    group0 {
        pinmux = <HPMICRO_PINMUX(HPMICRO_PIN(HPMICRO_PORTA, 17), IOC_TYPE_IOC, 0, 2)>;
        input-enable;
    };
};
```

注意：

- PA17 ALT2 是 UART4_RXD。
- 不再配置伪 TRGM/GPTMR idle。
- 不再使用 BIOC 思路。

## 11. 为什么这次修完 remote 会“动了”

之前的数据流：

```text
UART4 RX -> DMA -> 旧 GPTMR/TRGM idle 检测 -> idle IRQ -> flush -> callback
```

问题是 GPTMR/TRGM idle 检测这段在 HPM5361ICB 上不成立，导致接收完成事件不可靠，甚至破坏线程/DMA/中断状态。

现在的数据流：

```text
UART4 RX -> DMA -> UART 硬件 RX idle IRQ -> flush -> callback
```

改变点：

- idle 来源从外部模拟改成 UART 外设内部硬件 flag。
- 删除了 PA17 同时承担 UART 和 TRGM/GPTMR 输入的冲突。
- async UART 自动打开 IRQ。
- DMA flush 更保守，避免异常状态下破坏内存。

因此 remote 接收开始恢复，线程也不再被旧 idle path 拖死。

## 12. 后续开 CAN 的注意事项

CAN 当前没有打开，原因是要先把 remote 单独验证干净。

后续如果打开 CAN，需要单独确认：

- MCAN clock 是否正确。
- MCAN IRQ 是否正确。
- MCAN message RAM/AHB_SRAM section 是否正确。
- 当前硬件实际使用 PA14/PA15 还是 PB0/PB1。
- overlay 里的 `pinmux_mcan0_txrx` 是否和硬件原理图一致。
- `AHB_SRAM` orphan warning 是否需要通过 linker section 正式处理。
- loopback/internal mode 是否会触发底层 MCAN 状态异常。

不要在 remote 还没完全稳定前同时打开 CAN，否则两个底层问题会互相污染。

## 13. 验证建议

建议按下面顺序验证：

1. GPIO + WS2812B 单独运行，确认持续变色。
2. 开 UART4 + DMA + remote，确认 remote 数据能动。
3. remote 开启后连续 `west flash` 多次，确认不再出现只能烧一次。
4. remote 运行一段时间后 Halt Only，确认不再进入 Fatal Handler。
5. 再考虑打开 CAN。

出现问题时优先记录：

```gdb
p/x $pc
p/x $ra
p/x $sp
p/x $mcause
p/x $mepc
p/x $mtval
p/x $mstatus
p/x $mie
p/x $mip
bt
```

如果 Fatal Handler 记录变量还在，则继续看：

```gdb
p/x g_fatal_marker
p/x g_fatal_reason
p/x g_fatal_mcause
p/x g_fatal_mepc
p/x g_fatal_mtval
p/x g_fatal_thread
p/x g_fatal_esf_ra
p/x g_fatal_esf_a0
p/x g_fatal_esf_a5
```

## 14. 这次移植得到的原则

1. 自制板优先怀疑 board/soc/dts/pinctrl/clock/irq/dma，不要先动稳定运行在其他板上的应用层。
2. AI 生成的芯片层只能当草稿，必须对照官方 SDK 和真实 SoC feature。
3. HPM5361 有 UART 硬件 RX idle，就不要强行套 GPTMR/TRGM 软件 idle。
4. pinmux 不能靠猜，尤其是 IOC/PIOC/BIOC 这种 SoC 差异。
5. `rx_timeout=0` 能绕开问题，不代表应用层 timeout 有错，只代表底层 idle path 可疑。
6. XIP flash 调试优先用硬件断点。
7. BOOT/ISP 能恢复烧录，说明错误固件运行态会影响 OpenOCD 接管。
8. 每次只打开一个外设验证，先证明 GPIO/tick/thread，再证明 UART/DMA，再开 CAN。

## 15. 当前结论

HPM5361ICB 当前最关键的修复是：

```text
UART4 remote 接收从 GPTMR/TRGM 软件 idle 检测，改为 HPM5361 UART 硬件 RX idle 检测。
```

这解释了所有关键现象：

- 为什么不开 remote 时 GPIO/WS2812B 正常。
- 为什么开 remote 后线程跑不动。
- 为什么开 remote 后第二次烧录困难。
- 为什么 `rx_timeout=0` 能绕开。
- 为什么同一套应用在 HPM6E00 官方板上没有问题。

后续主要风险点已经从 UART+DMA 转移到 CAN/MCAN 和硬件 BOOT/JTAG 稳定性验证。
