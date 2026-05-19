# HPMicro 新芯片移植指南

本文档说明如何在本项目中为新的 HPMicro 芯片添加 Zephyr 支持。以 HPM6E80 → HPM5361 移植为例。

## 目录

1. [DTS 层](#1-dts-层)
2. [SoC 层](#2-soc-层)
3. [Board 层](#3-board-层)
4. [项目适配](#4-项目适配)
5. [常见问题](#5-常见问题)

---

## 1. DTS 层

路径: `d:\Zephyr_HPMicro\sdk_glue\dts\riscv\hpmicro\`

### 1.1 创建 SoC 系列 DTSI

创建 `<family>.dtsi`（如 `hpm53xx.dtsi`），包含:

**CPU 核心**

```dts
cpus {
    cpu0: cpu@0 {
        compatible = "andestech,andescore-d25", "riscv";
        reg = <0>;
        riscv,isa = "rv32gc_xandes";
        cpu0_intc: interrupt-controller {
            compatible = "riscv,cpu-intc";
            #interrupt-cells = <1>;
            interrupt-controller;
        };
    };
};
```

**中断控制器 IRQ 号**

| 设备 | 中断类型 | 正确 IRQ |
|------|---------|---------|
| PLIC | Machine External Interrupt | 11 |
| PLIC_SW | Machine Software Interrupt | 3 |
| MTIMER | Machine Timer Interrupt | 7 |

```dts
plic: interrupt-controller@e4000000 {
    interrupts-extended = <&cpu0_intc 11>;
};
plic_sw: interrupt-controller@e6400000 {
    interrupts-extended = <&cpu0_intc 3>;
};
mtimer: machine-timer@e6000000 {
    interrupts-extended = <&cpu0_intc 7>;
};
```

`riscv,ndev` 设为 `1023` 即可。

**外设节点**

每个外设需包含:
- `reg` — 寄存器地址和大小
- `interrupts` — 中断号（相对 PLIC 的偏移，从 1 开始）
- `interrupt-parent = <&plic>`
- `status = "disabled"`（板级 DTS 中开启需要的设备）

**GPIO 端口**（在 `&gpio0` 外部，因为 `ranges` 提供子节点地址映射）

```dts
&gpio0 {
    gpioa: gpio@0 {
        reg = <0x0 0x4000>;
        gpio-controller;
        #gpio-cells = <2>;
        hpmicro-gpio-port = <0>;
        interrupts = <1 1>;
        interrupt-parent = <&plic>;
    };
    ...
};
```

不同芯片的 GPIO 端口数不同（HPM5361: A/B/X/Y，无 Z）。

### 1.2 创建具体芯片 DTSI

简单文件，包含 family dtsi:

```dts
#include <hpmicro/hpm53xx.dtsi>
/ { };
```

### 1.3 时钟绑定头文件

路径: `d:\Zephyr_HPMicro\sdk_glue\dts\bindings\clock\`

创建芯片时钟宏定义头文件，如 `hpm5361-clocks.h`，定义 `CLOCK_UART0`、`CLOCK_CAN0` 等。

---

## 2. SoC 层

路径: `d:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\<SERIES>\`

参考现有 HPM6200 目录结构，创建以下文件:

### 2.1 Kconfig.soc — SoC 系列和型号选择

```kconfig
config SOC_SERIES_HPM5300
    bool "HPM5300 series"
    select SOC_FAMILY_HPM
    select HAS_HPMSDK_PLLCTL
    select HAS_HPMSDK_PCFG

config SOC_HPM5361
    bool "HPM5361"
    select SOC_SERIES_HPM5300

config SOC_SERIES
    default "HPM5300" if SOC_SERIES_HPM5300

config SOC
    default "HPM5361" if SOC_HPM5361
```

### 2.2 Kconfig — SoC 功能配置

```kconfig
config SOC_SERIES_HPM5300
    select RISCV
    select RISCV_PRIVILEGED
    select RISCV_HAS_PLIC
    select DYNAMIC_INTERRUPTS

config SOC_HPM5361
    select RISCV_ISA_RV32I
    select RISCV_ISA_EXT_M
    select RISCV_ISA_EXT_A
    select RISCV_ISA_EXT_C
    select RISCV_ISA_EXT_ZICSR
    select RISCV_ISA_EXT_ZIFENCEI
    select ATOMIC_OPERATIONS_C
    select INCLUDE_RESET_VECTOR
    select CPU_HAS_DCACHE
    select CPU_HAS_ICACHE
    select RISCV_PMP if !MCUBOOT
```

检查新芯片是否有 FPU，有则添加对应选项。

### 2.3 Kconfig.defconfig.series — 系列默认值

```kconfig
if SOC_SERIES_HPM5300
config SYS_CLOCK_HW_CYCLES_PER_SEC
    default 24000000
config KERNEL_ENTRY
    default "_start"
config ICACHE_ENABLE
    default y
config DCACHE_ENABLE
    default y
config NUM_IRQS
    default 80
endif
```

`NUM_IRQS` 根据芯片外设中断数设定，留余量。

### 2.4 Kconfig.defconfig.<chip> — 芯片堆栈配置

```kconfig
if SOC_HPM5361
config SYS_CLOCK_TICKS_PER_SEC
    default 1000
config ISR_STACK_SIZE
    default 2048
config MAIN_STACK_SIZE
    default 8192
config IDLE_STACK_SIZE
    default 2048
endif
```

### 2.5 CMakeLists.txt

```cmake
zephyr_include_directories(.)
zephyr_sources(start.S soc.c)
set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld)
```

### 2.6 soc.c — SoC 初始化

需处理:

- **XIP 支持**: `nor_cfg_option()` / `rom_marker` 初始化
- **时钟初始化**: `sysctl_clock_set_preset()`、`clock_add_to_group()`、`sysctl_config_cpu0_domain_clock()`、`clock_set_source_divider()`
- **PMA/PMP 配置**: NOCACHE 内存区域的 PMA/PMP 配置
- 通过 `SYS_INIT(soc_init, PRE_KERNEL_1, ...)` 注册

**API 差异**:

- HPM5300 使用 PLLCTL v1，HPM6200+ 使用 PLLCTL v2 — API 不同，包含的头文件也不同
- 没有 PLLCTL v2 的 SoC 不能包含 `hpm_pllctlv2_drv.h`，相关时钟操作使用 PLLCTL v1 API
- `sysctl_config_cpu0_domain_clock()` 参数个数因芯片而异
- 没有 PMA 硬件的芯片使用 PMP 代替

### 2.7 soc.h

```c
#define RISCV_MTIME_BASE   ((uintptr_t)DT_REG_ADDR_BY_IDX(DT_NODELABEL(mtimer), 0))
#define RISCV_MTIMECMP_BASE ((uintptr_t)DT_REG_ADDR_BY_IDX(DT_NODELABEL(mtimer), 1))
```

### 2.8 start.S

```asm
csrw mstatus, zero
l1c_ic_enable()
l1c_dc_enable()
jal c_startup
```

### 2.9 linker.ld

```
#include <../common/linker.ld>
```

---

## 3. Board 层

路径: `d:\Zephyr_HPMicro\sdk_glue\boards\hpmicro\<board_name>\`

Board 目录名 = board 名称（如 `hpm5361evk`）。

### 3.1 board DTS

创建 `<board_name>.dts`:

```dts
#include <hpmicro/hpm5361.dtsi>
#include "<board_name>-pinctrl.dtsi"

/ {
    model = "...";
    compatible = "<vendor>,<board_name>";

    chosen {
        zephyr,console = &uart0;
        zephyr,shell-uart = &uart0;
        zephyr,sram = &dlm;
        zephyr,flash = &flash0;
        zephyr,itcm = &ilm;
        zephyr,dtcm = &dlm;
        zephyr,code-partition = &slot0_partition;
    };

    leds {
        compatible = "gpio-leds";
        led_r: led_r {
            gpios = <&gpioa 23 GPIO_ACTIVE_HIGH>;
        };
    };
};

&uart0 { status = "okay"; ... };
&xpi0 {
    status = "okay";
    flash0: flash@80000000 {
        /* flash 参数 */
        partitions { ... };
    };
};
```

flash 分区必须定义在主 DTS 中（`flash_hpmicro.c` 通过 `DT_NODELABEL` 引用）。

### 3.2 board.c

```c
#include <hpm_ppor_drv.h>

void sys_arch_reboot(int type)
{
    ppor_sw_reset(HPM_PPOR, 3);
}
```

不同芯片的 PPOR 寄存器布局不同 — 检查 `hpm_ppor_regs.h`，不要直接写寄存器。

### 3.3 pinctrl DTSI

创建 `<board_name>-pinctrl.dtsi`，定义引脚复用:

```
pinmux_uart0: pinmux_uart0 {
    group0 {
        pinmux = <...>, <...>;
    };
};
```

### 3.4 defconfig

创建 `<board_name>_defconfig`:

```
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_XIP=y
CONFIG_PINCTRL=y
CONFIG_GPIO=y
CONFIG_SERIAL=y
CONFIG_NOCACHE_MEMORY=y
CONFIG_FLASH=y
```

### 3.5 Kconfig.board

**重要**: 文件名必须是 `Kconfig.<board_name>`（如 `Kconfig.hpm5361evk`），不能是 `Kconfig.board_<board_name>`。Zephyr HWMv2 只识别 `Kconfig.<board_name>` 模式。

```kconfig
config BOARD_HPM5361EVK
    bool "<board name>"
    select SOC_HPM5361
```

### 3.6 Kconfig（主文件）

板级额外配置项:

```kconfig
if BOARD_HPM5361EVK

config INIT_EXT_RAM
    bool "Initialize external RAM"
    default n

endif
```

### 3.7 Kconfig.defconfig

板级默认配置（可选）:

```kconfig
if BOARD_HPM5361EVK
config FOO
    default bar
endif
```

### 3.8 board.yml

```yaml
board:
  name: hpm5361evk
  vendor: hpmicro
  socs:
    - name: hpm5361
```

board 名 = 目录名。

### 3.9 board.cmake

```cmake
board_runner_args(openocd --cmd-pre-init "set CONFIG hpm5300evk")
board_runner_args(openocd --cmd-pre-init "set CHIP hpm5300")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
```

### 3.10 <board_name>.yaml

板级元数据:

```yaml
identifier: hpm5361evk
name: HPMicro HPM5361 Custom Board
type: mcu
arch: riscv32
toolchain:
  - zephyr
  - cross-compile
ram: 128
flash: 1024
supported:
  - gpio
  - uart
  - mcan
vendor: hpmicro
```

---

## 4. 项目适配

路径: `e:\Zephyr_project\HPMicro\blinky\`

### 4.1 CMakeLists.txt

设置 `BOARD_ROOT` 指向项目目录和 `projects/` 子目录，使自定义 board 可被发现:

```cmake
set(BOARD_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/projects
  CACHE PATH ""
)

set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${PROJ_DIR}/boards/${BOARD}.overlay)
```

### 4.2 Overlay

项目级 overlay 放在 `projects/boards/<board_name>.overlay`，用于添加项目特定的设备或配置（如 CAN 引脚复用），不包含 flash 分区等 board 级定义。

### 4.3 清理旧板

切换芯片时删除旧板文件:
- `projects/boards/<old_board>.overlay`
- `projects/boards/<old_board>.yaml`
- `projects/boards/Kconfig.<old_board>`
- `projects/boards/<old_board>_defconfig`

---

## 5. 常见问题

### 5.1 Kconfig.board 没生效

症状: Kconfig 警告 "X with value n, was y-selected by Y"

原因: `Kconfig.<board_name>` 命名不正确，Zephyr HWMv2 没找到该文件，板级配置未加载

修复: 确认文件名 `Kconfig.<board_name>` 与 reference board 一致

### 5.2 RISC-V 中断号错误

症状: `gen_isr_tables.py: error: IRQ X not present in parent offsets`

原因: PLIC/PLIC_SW/MTIMER 的 `interrupts-extended` 用了 IRQ 0

修复: PLIC→11, PLIC_SW→3, MTIMER→7

### 5.3 PPOR 寄存器差异

症状: `'PPOR_Type' has no member named 'RESET_SOFT'`

原因: 不同芯片 PPOR 寄存器布局不同

修复: 使用 `ppor_sw_reset()` API 而不是直接写寄存器

### 5.4 外设中断号范围

`riscv,ndev` 不影响实际中断分配，设为 `1023` 即可。

### 5.5 Flash 分区缺失

症状: `flash_hpmicro.c` 编译错误，找不到分区节点

修复: 在 board DTS 中定义所有 flash 分区（不在 overlay 中）

### 5.6 时钟 API 差异

不同芯片系列的 HPM SDK 时钟 API 有差异:
- PLLCTL v1 (HPM5300): `hpm_pllctlv1_drv.h`
- PLLCTL v2 (HPM6200+): `hpm_pllctlv2_drv.h`
- `sysctl_config_cpu0_domain_clock` 参数个数不同

### 5.7 多级中断 IRQ 冲突（PLIC + UART DMA）

症状: `gen_isr_tables.py` 报错:
```
multiple registrations at table_index 13 for irq 13 (0xd)
Existing handler 0x310, new handler 0x360
```

根因: HPM 的 PLIC 驱动强制启用 `CONFIG_MULTI_LEVEL_INTERRUPTS`。GPIO 端口经 `DT_IRQN` 宏编码为多级 IRQ：

```c
#define IRQ_TO_L2(irq) ((irq + 1) << CONFIG_1ST_LEVEL_INTERRUPT_BITS)
```

例如 gpioa(IRQ 1) 编码为 `((1+1)<<8) | 11 = 0x020b`，gen_isr_tables 计算 `table_index = 2ND_LVL_ISR_TBL_OFFSET + 2 - 1`。当 `2ND_LVL_ISR_TBL_OFFSET=12` 时，gpioa 落到 **index 13**，和 UART0 的简单 IRQ 13 冲突。

HPM6E00 无此问题是因为外设 IRQ 号较大（UART0=31），不会与 2nd-level 表重叠。

修复: 在 board 级 `Kconfig.defconfig` 中调整偏移量和表大小：
```kconfig
config 2ND_LVL_ISR_TBL_OFFSET
    default 13          # 避开 IRQ 13

config NUM_IRQS
    default 160         # 扩大 ISR 表，容纳偏移后的 2nd-level 表
```

原理: 偏移 +1 后 gpioa(原 index 13) 跳到 index 14，UART0 IRQ 13 仍在 index 13，不再冲突。

注意: 不同芯片的外设 IRQ 号不同（HPM5300 UART0=13，HPM6E00 UART0=31），移植新芯片时需根据 SoC 的 IRQ 映射表调整 `2ND_LVL_ISR_TBL_OFFSET`。

---

## 6. 附录：创建自定义 STM32 板

当芯片变体没有自动生成的 pinctrl 时（如 STM32F407IGH6 176 脚 BGA），需手动创建板级定义。

### 6.1 Board 目录

```
zephyr/boards/st/stm32f407igh6/
├── stm32f407igh6.dts               # 主设备树
├── stm32f407igh6-pinctrl.dtsi      # 引脚复用
├── Kconfig.stm32f407igh6           # Kconfig.board
├── stm32f407igh6_defconfig         # 默认配置
├── stm32f407igh6.yaml              # 元数据
├── board.yml
├── board.cmake                     # 烧录器
└── README.md                       # 标记为自定义
```

### 6.2 主 DTS

```dts
/dts-v1/;
#include <st/f4/stm32f407Xg.dtsi>
#include "stm32f407igh6-pinctrl.dtsi"

/ {
    model = "Custom STM32F407IGH6 board";
    compatible = "st,stm32f407igh6";

    chosen {
        zephyr,sram = &sram0;
        zephyr,flash = &flash0;
        zephyr,ccm = &ccm0;
    };
    /* 板级设备... */
};

&dma1 { status = "okay"; };
&dma2 { status = "okay"; };
```

### 6.3 pinctrl 文件

include 路径用 `<dt-bindings/...>`，不带 `zephyr/` 前缀：

```dts
#include <dt-bindings/pinctrl/stm32-pinctrl.h>

/ {
    soc {
        pinctrl: pin-controller@40020000 {
            usart1_tx_pa9: usart1_tx_pa9 {
                pinmux = <STM32_PINMUX('A', 9, AF7)>;
            };
        };
    };
};
```

**注意：** `STM32_PINMUX` 的第三个参数只传 `AF7`，不是 `STM32_AF7`（宏内部已拼接 `STM32_` 前缀）。

### 6.4 构建

```bash
west build -b stm32f407igh6 -p
```

项目 overlay 命名需与 board 名一致（如 `stm32f407igh6.overlay`），`build.bat` 自动识别 board 名。
