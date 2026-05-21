# HPM5361 UART 链路排查状态

日期：2026-05-21

## 当前结论

HPM5361 上 `remote -> dr16` 异常的核心问题，不在应用层协议解析，也不在 `uart.cpp` 的 Zephyr async 回调封装本身。

已经确认：

- `UART4_RX -> DMA搬运 -> DMA完成回调 -> UART_RX_RDY -> uart.cpp -> remote.cpp -> dr16.cpp`
  这条链路是通的。
- 不通的是：
  `UART4_RX -> UART RX idle 检测 -> 提前 flush DMA -> 小块 UART_RX_RDY`

也就是说，现在数据能收上来，但不是按帧/按 idle 切出来，而是经常等 DMA buffer 满了以后整块往上推。

## 已确认的现象

### 1. 应用层不是首要问题

`STM32` 板子同样的应用层能正常工作，说明：

- `modules/remotes/remote.cpp`
- `modules/remotes/dr16/dr16.cpp`
- `drivers/communication/uart/uart.cpp`

这些代码结构本身不是首要根因。

### 2. DMA 回调链是通的

调试时看到过：

- `dma_cb ch=2 status=0`
- `UART_RX_RDY off=0 len=128`
- `remote.cpp` 能持续 `Read()`
- `dr16.cpp` 能进入 `dataprocess()`

说明：

- UART4 确实在收数据
- DMA 确实在搬数据
- Zephyr async callback 确实在往上发事件

### 3. 现在的回调节奏是“满 buffer 才上报”

典型现象：

- `128 bytes, 4 reads`
- `frame_pos` 每次按 `2` 递增

这说明：

- 一次处理拿到的是完整 `128B` DMA 缓冲
- `remote.cpp` 只是因为每次 `Read()` 读 `32B`，所以变成 `4 reads`
- `frame_pos += 2` 本质上是 `128 mod 18 = 2`

也就是说，当前行为不是“18B 一帧一回调”，而是“128B 满缓冲后一次性回调”。

### 4. UART IRQ 入口会进，但 idle 不命中

排查时已经看到：

- `uart_hpm_isr()` 入口日志会出现
- 但 `idle=0`
- 没有命中 `idle-hit`

这说明：

- UART 中断入口不是完全没连上
- 但 `UART RX idle flag` 没有按预期触发
- 真正把数据推上层的仍然是 DMA 满缓冲回调

## 对 HPM5361 的关键判断

### 1. 原始 TRGM/GPTMR 软件 idle 路径不成立

已经确认板上 `UART4_RX` 只接到了 `PA17`。

虽然官方 `HPM5361` 头文件里：

- `PA17` 可以配置成 `UART4_RXD`
- `PA17` 也可以配置成 `TRGM0_P_05`

但这是同一个 PAD 的两种复用，不是同时生效。

因此：

- 当 `PA17` 配成 `UART4_RXD` 时
- `TRGM0_P5` 这条输入链路并没有同时成立

所以基于 `TRGM + GPTMR` 的软件 idle 检测，对这块板当前接法不可靠。

### 2. 已改成“硬件 RX idle 优先，软件 idle 兜底”

当前 `D:\Zephyr_HPMicro\sdk_glue\drivers\serial\uart_hpmicro.c` 已经改成：

- 如果 HPM UART IP 支持 `RX idle detect`
- 就优先走 `UART` 外设自己的 `RX idle`
- 保留旧 `TRGM/GPTMR` 路径作为兜底

这是按 HPM UART IP 能力做的通用底层适配，不是只给 `5361` 加板子分支。

## 目前卡住的位置

当前真正卡住的位置是：

`UART RX idle 检测 -> 提前 flush DMA`

具体表现为：

- idle 路径没有把 `128B` 打散成接近 `18B` 的小块回调
- 最终上层看到的仍然是 `128 bytes, 4 reads`

所以当前的根问题，不是“收不到”，而是：

**收得到，但没有按 idle/按帧边界及时切出来。**

## 当前可用的工程性绕法

如果把 DMA buffer 改成固定 `18B`，现象会变成：

- `18 bytes, 1 reads`

这说明：

- 应用层可以在固定帧长模式下恢复正常节奏
- 但这只是绕开 idle 机制，不代表底层 idle 问题已经真正解决

## 当前建议

后续排查重点应该继续盯：

- HPM UART `RX idle` 标志为什么没有按预期生效
- 为什么最终总是由 `DMA满缓冲` 触发上报，而不是 `idle` 提前触发

如果目标是先让 DR16 稳定可用，固定帧长 DMA 是短期可落地方案。

如果目标是把 HPM Zephyr UART async 底层彻底修好，就要继续解决：

`UART RX idle flag -> flush DMA`

这一步。
