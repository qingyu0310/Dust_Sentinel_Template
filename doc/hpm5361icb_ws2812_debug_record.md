# HPM5361ICB 调试与 WS2812B 修改记录

日期: 2026-05-20

本文记录 HPM5361ICB 自制核心板在 Zephyr 工程中的移植、烧录、调试、GPIO 线程、WS2812B 驱动排查过程。重点是把这次遇到的现象和判断链保留下来，避免后续 remote/CAN/UART/DMA 再打开时重复踩同样的问题。

## 背景

当前工程路径:

```text
D:\Zephyr\projects\tflm
```

HPMicro 适配层路径:

```text
D:\Zephyr_HPMicro\sdk_glue
```

板级目录:

```text
D:\Zephyr_HPMicro\sdk_glue\boards\hpmicro\hpm5361icb
```

这块板是自制 HPM5361 核心板。原生 Zephyr 没有 HPM5361 芯片和 `hpm5361icb` 板级适配，早期 `hpm5361icb` 目录由 AI 改写生成，因此芯片层、板级 DTS、OpenOCD、时钟、GPIO、UART/DMA/CAN 这些内容都需要审视。

对比基准:

- 同一套应用代码在 ST 板上正常。
- 同一套应用代码在 HPM6E00 官方开发板上正常。
- HPM6E00 EVK 是 SDK 自带板级，可信度高。
- HPM5361ICB 是自制板 + 新增板级适配，所以优先怀疑 board/soc 层，而不是 Zephyr API 或业务层。

## 最终结论

这次最终打通 WS2812B 的关键不是线程调度，也不是 `k_msleep()`，而是 WS2812B 对 GPIO 时序非常敏感。

最终有效修改是:

1. `projects/thread/gpio/trd_gpio.cpp`
   - HPM5361 下不走 Zephyr GPIO API。
   - 直接使用 `HPM_GPIO0`、port `0`、pin `10`，也就是 PA10。

2. `modules/leds/ws2812b.hpp`
   - 新增 HPM fast path。
   - 发送函数放入 `.fast` 段。
   - 使用 `mcycle` 控制 bit 周期。
   - 直接写 GPIO `DO[port].SET` / `DO[port].CLEAR` 寄存器。
   - 发送期间 `irq_lock()`，避免中断破坏 WS2812B 纳秒级时序。

最终确认: LED 能持续按线程里的颜色序列变化。

## 最终有效代码路径

### GPIO 线程

文件:

```text
projects/thread/gpio/trd_gpio.cpp
```

当前核心逻辑:

```cpp
static void Task(void*, void*, void*)
{
    uint8_t phase = 0;
    static constexpr Ws2812b::Color colors[] = {
        {32, 0, 0},
        {0, 32, 0},
        {0, 0, 32},
        {0, 0, 0},
    };

    for (;;)
    {
        led_r.set(colors[phase++ % ARRAY_SIZE(colors)]);
        k_msleep(500);
    }
}
```

HPM5361 初始化:

```cpp
void thread_init()
{
#ifdef CONFIG_SOC_HPM5361
    led_r.init(HPM_GPIO0, 0, 10);
#else
    led_r.init(GPIO_GET(led_alert));
#endif
}
```

这里 `HPM_GPIO0, 0, 10` 对应 GPIOA.10，也就是 PA10。

### WS2812B 驱动

文件:

```text
modules/leds/ws2812b.hpp
```

HPM fast path 条件:

```cpp
#if defined(CONFIG_SOC_SERIES_HPM5300) || defined(CONFIG_SOC_HPM5361)
#define WS2812B_FAST_CODE __attribute__((section(".fast"), noinline, optimize("O2")))
#else
#define WS2812B_FAST_CODE
#endif
```

HPM 初始化时保存寄存器地址:

```cpp
fast_set_ = &gpio->DO[port].SET;
fast_clear_ = &gpio->DO[port].CLEAR;
```

发送时使用:

```cpp
send_grb_fast(fast_set_, fast_clear_, fast_mask_, color.g, color.r, color.b,
              t0h_cycles_, t1h_cycles_, bit_cycles_);
```

bit 发送方式:

```cpp
*set_reg = mask;
wait_until_cycle(bit_start + (bit ? t1h_cycles : t0h_cycles));
*clear_reg = mask;
wait_until_cycle(bit_start + bit_cycles);
```

这保证了每个 bit 周期约 1.25us，高电平宽度按 0/1 分开控制。

## 为什么 Zephyr GPIO API 不行

尝试过把 HPM5361 也切到:

```cpp
led_r.init(GPIO_GET(led_alert));
```

结果线程能跑，但是颜色不变或不亮。

原因是 WS2812B 的协议不是普通 GPIO 翻转:

- 800kHz 数据速率。
- 一个 bit 周期约 1.25us。
- `0` 和 `1` 靠高电平宽度区分。
- 任意函数调用、设备 API、检查、总线等待、调试器单步、中断都可能破坏波形。

Zephyr `gpio_pin_set_dt()` 的抽象层太厚，适合验证 PA10 是否能当 GPIO 输出，不适合直接驱动 WS2812B。

## 为什么之前直接 bitbang 也不稳

早期版本已经尝试过直接写 HPM GPIO 寄存器，但仍然不稳定。主要问题是:

- 发送函数在普通 XIP flash 中执行，取指延迟不够稳定。
- 每个 bit 中调用层级较多，函数调用和 flash 取指抖动会影响高低电平宽度。
- 调试器断点或单步进入 `led_r.set()` 会直接破坏 WS2812B 时序。

最终版本把关键发送函数放进 `.fast`，并且将 bit 周期固定为 `bit_start + bit_cycles`，比“高电平 delay + 低电平 delay”的累计方式更稳。

## 调试过程中遇到的问题

### 1. OpenOCD 偶发无法 halt

典型日志:

```text
Error: [hpm5361.cpu0] Unable to halt. dmcontrol=0x00000001, dmstatus=0x00400ca2
Error: [hpm5361.cpu0] Fatal: Hart 0 failed to halt during examine
Error: [hpm5361.cpu0] Unsupported DTM version: -1
```

有时还会出现:

```text
JTAG scan chain interrogation failed: all ones
dtmcs.abits is zero. Check JTAG connectivity/board power
Unsupported DTM version: 15
```

当时判断:

- 不是单纯编译问题。
- 有硬件/JTAG/BOOT 状态因素。
- 但后来发现只要 remote 线程不开，其他线程可以反复烧录，说明应用启动后的外设配置也可能影响调试连接。

处理过的方向:

- JTAG 降速到 500kHz。
- BOOT 引脚切换到可重新烧录的模式。
- 使用 SDK 自带 OpenOCD 脚本，而不是手写 `cmsis_dap.cfg`。

### 2. BOOT 悬空后能恢复烧录

现象:

- 某次烧录后再次烧录失败。
- 把 BOOT 全部悬空后可以烧录进去。
- 第二次又可能失败。

当时判断:

- BOOT 状态会影响进入 ISP/串行启动/flash 启动。
- 如果用户程序启动后让芯片进入某种不容易 halt 的状态，JTAG examine 可能失败。
- 这不必然说明应用代码写坏 flash，更像是 boot strap、硬件连接、JTAG halt 时机、外设配置共同影响。

后续建议:

- BOOT 引脚不要悬空，必须有确定上下拉。
- 如果打开 remote/CAN 后又复现“只能烧一次”，优先查对应 UART/DMA/CAN 的 pinmux、clock、interrupt、DMA channel。

### 3. 链接 orphan section `AHB_SRAM`

曾出现:

```text
warning: orphan section `AHB_SRAM' from `zephyr/drivers/can/libdrivers__can.a(mcan_hpmicro.c.obj)' being placed in section `AHB_SRAM'
```

判断:

- 这是 linker orphan section 警告，不是当时 flash/debug 失败的直接根因。
- 但它说明 HPM CAN 驱动里有数据想放到 `AHB_SRAM` 段，板级 linker/memory region 需要正确支持。
- 当前在只调 GPIO/WS2812B 阶段，CAN 已关闭，不是主线问题。

### 4. Debug 进不去或 init flash failed

曾出现:

```text
Error: init flash failed on target: 0x2af8
Error: auto_probe failed
Connect failed.
```

处理:

- 在 VSCode `launch.json` 中对 HPM5361 调试配置增加:

```gdb
gdb_memory_map disable
set mem inaccessible-by-default off
mem 0x80000000 0x80100000 ro 32 nocache
mem 0x00080000 0x000a0000 rw 32 nocache
set remote hardware-breakpoint-limit 4
set breakpoint auto-hw on
```

原因:

- 程序运行在 XIP flash `0x80000000` 区域。
- 普通软件断点需要改写 flash 指令，OpenOCD/GDB 无法直接写入，导致断点失败。

### 5. 普通断点/Step Over 报错

典型错误:

```text
Cannot insert breakpoint.
Cannot access memory at address 0x80003834
Cannot access memory at address 0x80004368
```

原因:

- 地址在 XIP flash。
- GDB 普通行断点会尝试插软件断点。
- `next` / Step Over 也会临时插断点，所以也会失败。

正确做法:

```gdb
hbreak D:/Zephyr/projects/tflm/projects/thread/gpio/trd_gpio.cpp:53
continue
```

不要单步进入 `led_r.set()`，否则 WS2812B 波形会被调试器破坏。

### 6. Halt Only 停在 GPIO ISR 或 idle

曾看到 halt 后停在:

```c
gpio_hpm_port_isr()
```

或者:

```c
arch_cpu_idle()
```

判断:

- `Attach HPM5361ICB Halt Only` 是暂停当前 CPU，不是复位后跑到 `main`。
- CPU 当时在哪里就停在哪里。
- 停在 ISR 或 idle 并不等于异常。

后来 `Debug HPM5361ICB` 配置改成:

```gdb
monitor reset halt
thbreak main
continue
```

这样才是复位后跑到 `main`。

### 7. 曾进入 Fatal_handler

早期曾进入:

```c
arch_system_halt()
```

或者工程里的:

```text
projects/apps/Fatal_handler.cpp
```

当时读到过的关键寄存器:

```text
mcause = 0x5
mtval  = 0x8080f98
mepc   = 0x80005b50 / 0x80005b96
```

解释:

- `mcause = 0x5` 是 load access fault。
- `mtval = 0x08080f98` 这种值和 `&slice_timeouts = 0x80f98` 很像，像是指针高位被污染。
- 但后续在清理板级、GPIO ISR、时钟后，这条线没有继续作为最终根因。

当前最终阶段已经“不进 Fatal_handler”，说明最初的 fatal 不是 WS2812B 最后不变色的根因。

### 8. `arch_cpu_idle()` 不是卡死证据

曾多次停在:

```c
void arch_cpu_idle(void)
{
    sys_trace_idle();
    __asm__ volatile("wfi");
    sys_trace_idle_exit();
    irq_unlock(MSTATUS_IEN);
}
```

后面 GDB 确认:

```text
mcause = 0x80000007
```

这是 machine timer interrupt，不是异常。

同时读过 `mtime/mtimecmp`:

```text
0xe6000000 / 0xe6000008
```

`last_ticks` 也在增长，说明 mchtmr/系统 tick 在工作。

因此停在 idle 不能直接判断线程卡死。需要结合:

```gdb
p/x _kernel.cpus[0].current
p/x _kernel.ready_q.cache
x/8wx &timeout_list
```

判断线程是否在 sleep timeout 里。

### 9. 线程是否真的跑到 `led_r.set()`

最终确认过:

- 线程确实跑进:

```cpp
led_r.set(colors[phase++ % ARRAY_SIZE(colors)]);
```

- 所以应用启动链和线程调度不是最终问题。

应用启动链:

```text
main()
  -> System_Bsp_Init()
  -> System_Modules_Init()
       -> output::thread_init()
  -> System_Thread_Start()
       -> output::thread_start(6)
            -> Task()
                 -> led_r.set()
                 -> k_msleep(500)
```

### 10. GPIO API 路径验证失败

曾按验证思路把 HPM5361 改成:

```cpp
led_r.init(GPIO_GET(led_alert));
```

结果:

- 不再卡死。
- 能跑到 `led_r.set()`。
- 但是 WS2812B 颜色不变。

结论:

- GPIO API 不适合 WS2812B 精确定时。
- 这个尝试只能说明应用层和 GPIO 设备大体能跑，不能作为 WS2812B 正式驱动方案。

## HPM5361 board/soc 层排查和修改

### 板级目录重建

因为 `hpm5361icb` 早期由 AI 改写，后面将旧目录移走备份，重新按官方 HPM SDK/Zephyr glue 思路配置。

注意: 备份目录如果仍放在 `boards/hpmicro` 下，会导致 Zephyr 发现两个同名 board。

曾出现:

```text
Board(s): {'hpm5361icb'}, defined multiple times.
Last defined in ... hpm5361icb.ai_backup_20260520\board.yml
```

处理:

- 将备份移到 `D:\Zephyr_HPMicro\sdk_glue\boards_backup`。
- 避免 `boards` 搜索路径下存在重复 `board.yml`。

### DTS 关键配置

当前 `hpm5361icb.dts` 重点:

```dts
chosen {
    zephyr,sram = &dlm;
    zephyr,flash = &flash0;
    zephyr,itcm = &ilm;
    zephyr,code-partition = &slot0_partition;
    zephyr,flash-controller = &xpi0;
};
```

WS2812B 对应:

```dts
led_alert: led_alert {
    gpios = <&gpioa 10 (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH)>;
    label = "LED_ALERT";
};
```

当前仅调 GPIO/WS2812B 时:

```dts
&uart0 { status = "disabled"; };
&uart4 { status = "disabled"; };
&hdma  { status = "disabled"; };
&mcan0 { status = "disabled"; };
```

### PA10 pinctrl

当前 PA10:

```dts
pinmux_gpioa: pinmux_gpioa {
    group0 {
        pinmux = <HPMICRO_PINMUX(HPMICRO_PIN(HPMICRO_PORTA, 10), IOC_TYPE_IOC, 0, 0)>;
        bias-pull-down;
    };
};
```

可选后续优化:

```dts
bias-disable;
drive-strength = "r111";
slew-rate = "fast";
```

这不是本次最终必要条件，因为 `.fast + mcycle + SET/CLEAR` 已经能驱动成功。

### OpenOCD 配置

曾经手写过本地 `cmsis_dap.cfg`，出现:

```text
Can't find interface/cmsis-dap.cfg
```

后来改成直接使用 HPM SDK 自带 OpenOCD 脚本:

```text
D:/Zephyr_HPMicro/sdk_env/hpm_sdk/boards/openocd/probes/cmsis_dap.cfg
D:/Zephyr_HPMicro/sdk_env/hpm_sdk/boards/openocd/soc/hpm5300.cfg
D:/Zephyr_HPMicro/sdk_env/hpm_sdk/boards/openocd/boards/hpm5300evk.cfg
```

并且 adapter speed 使用 500kHz。

### support 目录问题

Zephyr OpenOCD runner 曾因 `support` 目录不存在报错:

```text
FileNotFoundError: ... hpm5361icb\support
```

处理:

- 保留空 `support` 目录。
- 但实际 OpenOCD config 使用 SDK 自带脚本。

### 时钟和低功耗

HPM5361 SoC 初始化对齐 HPM5300EVK 官方思路:

- group0 加入 `clock_cpu0`
- `clock_ahb`
- `clock_lmm0`
- `clock_mchtmr0`
- `clock_rom`
- `clock_mot0`
- `clock_gpio`
- `clock_hdma`
- `clock_xpi0`
- `clock_ptpc`
- CPU0 domain 配成 PLL0。
- `clock_mchtmr0` 使用 24MHz。
- 设置:

```c
sysctl_set_cpu_lp_mode(HPM_SYSCTL, HPM_CORE0, cpu_lp_mode_ungate_cpu_clock);
```

这个设置用于避免 CPU idle/WFI 后关键时钟被 gate，影响 tick 唤醒判断。

### PMA/PMP 问题

曾出现:

```text
undefined reference to `pma_config_attributes'
```

HPM5361 官方 feature 中 `PMP_SUPPORT_PMA` 为 0，因此不能按 PMA 路径配置。当前使用 PMP 处理 nocache 场景，并且默认:

```text
CONFIG_SOC_ANDES_V5_PMA=n
```

### DCDC API 链接问题

曾出现:

```text
undefined reference to `pcfg_dcdc_set_voltage'
```

原因:

- `soc.c` 使用 `pcfg_dcdc_set_voltage()`。
- Kconfig/CMake 没有把对应 HPM SDK PCFG 驱动拉进来。

处理:

- 在 HPM5300 SoC Kconfig 中选择 PCFG 支持。

## GPIO ISR 修改

早期 halt only 经常停在 GPIO ISR，且担心 PA10 WS2812B 输出触发 GPIO 中断。

GPIO ISR 后来改为:

```c
uint32_t raw_status = gpio_base->IF[port_base].VALUE;
uint32_t int_status = raw_status & gpio_base->IE[port_base].VALUE;

gpio_base->IF[port_base].VALUE = raw_status;

if (int_status != 0U) {
    gpio_fire_callbacks(&data->callbacks, dev, int_status);
}
```

含义:

- 清所有 raw pending flag。
- 只对已经 enable 的中断触发 callback。
- 避免未使能的 GPIO flag 造成 callback 误触发。

同时 GPIO init 阶段禁用 IE/AS/PD，并清 IF。

这对“GPIO ISR 误入”有帮助，但最终 WS2812B 不变色的直接根因仍然是时序。

## Kconfig 发现的问题

`projects/thread/Kconfig` 中看到过中文编码/换行污染，例如:

```text
# can... config TRD_CAN_TX
...
# tflm... config TRD_TFLM
```

当前 `.config` 中结果是:

```text
CONFIG_TRD_GPIO=y
# CONFIG_TRD_CHASSIS is not set
# CONFIG_TRD_GIMBAL is not set
# CONFIG_TRD_CAN_TX is not set
# CONFIG_TRD_REMOTE is not set
# CONFIG_TRD_TFLM is not set
```

也就是说当前 GPIO/WS2812B 调试阶段没有受 CAN/remote/TFLM 影响。

但后续建议修复 Kconfig 文件编码和换行，避免某些 symbol 实际没被正确定义。

## 构建和烧录命令

用户指定本工程使用:

```powershell
.\cmd\build\build.bat hpm5361icb -p
```

全量 pristine 编译。

普通增量编译:

```powershell
.\cmd\build\build.bat hpm5361icb
```

烧录:

```powershell
west flash
```

注意:

- 本次文档编写没有执行编译。
- 后续不要直接用 `west build` 替代项目脚本。

## 调试建议

### 查看是否跑到 WS2812B set

使用硬件断点:

```gdb
delete breakpoints
hbreak D:/Zephyr/projects/tflm/projects/thread/gpio/trd_gpio.cpp:53
continue
```

如果反复命中，说明线程调度和 `k_msleep(500)` 没问题。

### 不要单步 WS2812B

不要对下面函数使用 Step Over/Step Into:

```cpp
led_r.set(...)
```

原因:

- WS2812B 的 bit 时序是纳秒级。
- 调试器暂停会让灯接收到错误波形。
- 可能表现为不变色、白光、绿色、闪一下后不动。

### 判断 idle 是否正常

如果停在:

```text
arch_cpu_idle()
```

先看:

```gdb
p/x $mcause
p/x $mepc
p/x $mtval
```

如果:

```text
mcause = 0x80000007
```

这是 machine timer interrupt，不是 fault。

### 判断线程 timeout

可看:

```gdb
p/x _kernel.cpus[0].current
p/x _kernel.ready_q.cache
x/8wx &timeout_list
x/8wx &slice_timeouts
```

之前读到:

```text
0x80db0 = z_idle_threads
0x80510 = thread::output::thread_
0x80528 = output thread timeout node
```

这说明 output 线程执行过 `k_msleep()`，系统处于 idle 等 tick 唤醒。

## 后续风险和待办

### 1. remote/UART/DMA 尚未最终复测

最初怀疑 remote 是因为:

- 不开 remote 时，GPIO 线程可反复烧录。
- 打开 remote 后，只能烧录一次或调试异常。

当前只确认 WS2812B 已经成功，不等于 remote/UART/DMA 已经正确。

后续打开 remote 时重点检查:

- UART4 RX pinmux 是否正确。
- UART idle TRGM/GPTMR 参数是否属于 HPM5361。
- DMA channel/request source 是否正确。
- UART/DMA 中断号是否正确。
- 是否有 DMA 写错内存导致 timeout/list 指针污染。

### 2. CAN 尚未最终复测

曾经打开 CAN 后停在 MCAN loopback/mode 相关位置，并影响 GPIO 线程。当前 CAN 关闭。

后续打开 CAN 时重点检查:

- MCAN base address。
- MCAN clock。
- IRQ number。
- Message RAM 是否放到正确 memory region。
- `AHB_SRAM` linker section 是否有完整定义。
- PA14/PA15 pinmux 是否和硬件一致。

### 3. GPIO driver 仍建议复查

当前 GPIO driver 已经比最初安全，但仍建议后续审查:

- `gpio_hpm_port_set_masked_raw()` 中对 `OE[port].CLEAR` 的读取是否合理。
- HPM GPIO 寄存器中部分字段是 write-only，读 write-only 寄存器可能不可靠。
- GPIO API 对普通 LED/输入可以继续测，但不要用于 WS2812B 时序输出。

### 4. BOOT 引脚必须固定

BOOT 全悬空能恢复烧录只是临时救援手段，不是稳定设计。

后续硬件上建议:

- BOOT strap 全部给确定上下拉。
- 调试接口的 nRESET、TCK/TMS/TDI/TDO 走线和上拉下拉复核。
- 如果可能，保留强制 ISP/flash boot 的拨码或跳帽。

## 当前状态

当前已经确认:

- HPM5361ICB 能烧录。
- VSCode/OpenOCD 能调试到 `main`。
- 线程能跑到 `led_r.set()`。
- `k_msleep(500)`/mtime/idle 不再是主问题。
- WS2812B 已经能持续变色。

当前最重要的保留点:

- `ws2812b.hpp` 中 `.fast + mcycle + SET/CLEAR` 的实现不要随便改回 GPIO API。
- 调试时不要单步 WS2812B 发送函数。
- 后续打开 remote/CAN 时，继续用这个 WS2812B 作为系统活性指示器。

