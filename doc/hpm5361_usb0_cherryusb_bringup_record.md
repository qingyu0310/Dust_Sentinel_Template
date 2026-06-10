# HPM5361ICB USB0 CherryUSB 枚举调试记录

> 记录时间：2026-06-10  
> 目标：把“当前可以枚举成虚拟串口的 USB 版本”和“之前一直不能枚举的 USB 版本/中间失败状态”固定下来，避免后续调试重复走错路径。

## 结论先行

当前已经验证成功的是 **HPMicro 官方 Zephyr CherryUSB CDC ACM sample 路径移植版**：

- 应用入口：`projects/thread/test/trd_test.cpp`
- 开关：`CONFIG_TRD_TEST_CHERRYUSB=y`
- HPM CherryUSB 设备驱动节点：`projects/boards/hpm/hpm5361icb/hpm5361icb.overlay` 中的 `cherryusb_usb0: &usb0`
- CherryUSB 配置头：`D:/Zephyr_HPMicro/sdk_glue/samples/cherryusb/config/usb_config.h`
- 主机侧结果：Windows 已出现虚拟串口

当前能枚举，不是因为手动“硬怼 PHY 寄存器”，而是因为下面几件事终于同时满足了：

1. `usb0` 走的是 HPM 官方 CherryUSB controller driver，而不是普通 Zephyr CDC ACM UART 包装层。
2. CherryUSB 需要的 `.nocache` 段可以正常使用，`g_usbd_core` / DCD buffer 不再落到错误或未配置的内存属性里。
3. `usb_device_init()` 最后真正调用了 `usb_dcd_connect()`，把 `USBCMD.RS` 从 0 置 1，让 device attach 到总线。
4. `usbd_initialize()` 之后出现了完整枚举事件链：`INIT -> RESET -> CONNECTED -> CONFIGURED`。

之前“不枚举”的版本主要有两类：

- **旧 Zephyr CDC ACM UART 版本**：`CONFIG_COM_USB` 路径，走 `UsbUart` / Zephyr new USB device stack，未验证出 HPM5361ICB USB0 可枚举。
- **CherryUSB 移植过程中的失败状态**：已经进入 CherryUSB 代码，但先后卡在 `.nocache` 和 `connect` 两个关键点。

这份记录重点保存已经查清楚的第二类失败状态，因为它们有明确日志和寄存器证据。

## 两个 USB 版本的代码路径

### 1. 当前能枚举版本：官方 Zephyr CherryUSB CDC ACM 移植

官方例程位置：

```text
D:/Zephyr_HPMicro/sdk_glue/samples/cherryusb/device/cdc_acm/cdc_acm_vcom
```

官方例程关键点：

```cmake
set(CHERRYUSB_CONFIG_DIR ${HPM_ZEPHYR_DIR}/samples/cherryusb/config)
zephyr_include_directories(${CHERRYUSB_CONFIG_DIR})
```

```conf
CONFIG_CHERRYUSB=y
CONFIG_CHERRYUSB_DEVICE=y
CONFIG_CHERRYUSB_DEVICE_SPEED_AUTO=y
CONFIG_CHERRYUSB_DEVICE_HPM=y
CONFIG_CHERRYUSB_DEVICE_CDC_ACM=y
```

本工程移植后的关键配置：

```conf
CONFIG_COM_USB=n
CONFIG_TRD_TEST_CHERRYUSB=y
CONFIG_CHERRYUSB=y
CONFIG_CHERRYUSB_DEVICE=y
CONFIG_CHERRYUSB_DEVICE_SPEED_AUTO=y
CONFIG_CHERRYUSB_DEVICE_HPM=y
CONFIG_CHERRYUSB_DEVICE_CDC_ACM=y
CONFIG_SOC_ANDES_V5_PMA=y
CONFIG_NOCACHE_MEMORY=y
```

本工程移植后的 `usb0` overlay：

```dts
cherryusb_usb0: &usb0 {
    compatible = "hpmicro,hpm-cherryusb";
    clk-name = <CLOCK_USB0>;
    status = "okay";
};
```

应用侧初始化顺序：

```text
DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0))
-> usbd_desc_register()
-> usbd_add_interface()
-> usbd_add_endpoint()
-> usbd_initialize()
```

这和官方 Zephyr CherryUSB sample 一致：sample 本身没有裸 SDK 的 `board_init_usb()`，也没有手动写一大段 PHY prime 逻辑。

### 2. 之前不枚举版本：Zephyr CDC ACM UART 路径

旧路径在当前文件中还保留在 `#elif defined(CONFIG_COM_USB)` 分支：

```text
CONFIG_COM_USB
-> drivers/communication/usb/usb.cpp
-> UsbUart
-> DEVICE_DT_GET(DT_ALIAS(pc_usb))
-> uart_poll_out / uart_irq_rx_enable
```

该路径的 Kconfig：

```conf
config COM_USB
    depends on DT_HAS_ZEPHYR_CDC_ACM_UART_ENABLED
    select USB_DEVICE_STACK_NEXT
    select USBD_CDC_ACM_CLASS
    select CDC_ACM_SERIAL_INITIALIZE_AT_BOOT
    select SERIAL
    select UART_INTERRUPT_DRIVEN
    select UART_LINE_CTRL
```

这个版本的问题不是应用层 `UsbUart::Send()` 写法本身，而是底层 USB device attach/enumeration 没有闭环。它依赖 Zephyr new USB device stack 和 `zephyr,cdc-acm-uart` 设备节点，而本轮最终跑通的是 HPM 官方 CherryUSB HPM port。

当前结论：在 HPM5361ICB USB0 上，先以 CherryUSB 官方 sample 路径作为可信基线。旧 `CONFIG_COM_USB` 路径不要再当作 USB0 bring-up 的第一验证路径。

### 2.1 旧 `CONFIG_COM_USB` 版实际踩过的问题补记

旧版不是“只差一个串口发送函数”。它的问题发生在应用层 `UsbUart` 之前，核心是 **Zephyr new USB device stack + HPM UDC glue** 这一整条链没有在 HPM5361ICB USB0 上闭环。

旧版链路应该是：

```text
CONFIG_COM_USB=y
-> drivers/communication/usb/usb.cpp
-> UsbUart::Init()
-> DEVICE_DT_GET(DT_ALIAS(pc_usb))
-> zephyr,cdc-acm-uart
-> Zephyr USB device stack next
-> UDC_HPMICRO
-> D:/Zephyr_HPMicro/sdk_glue/drivers/usb/udc/udc_hpmicro.c
-> D:/Zephyr_HPMicro/sdk_env/hpm_sdk/components/usb/device/hpm_usb_device.c
-> D:/Zephyr_HPMicro/sdk_env/hpm_sdk/drivers/src/hpm_usb_drv.c
```

旧版需要的 DTS/Kconfig 条件和当前 CherryUSB 版不同：

```conf
config COM_USB
    depends on DT_HAS_ZEPHYR_CDC_ACM_UART_ENABLED
    select USB_DEVICE_STACK_NEXT
    select USBD_CDC_ACM_CLASS
    select CDC_ACM_SERIAL_INITIALIZE_AT_BOOT
```

```conf
config UDC_HPMICRO
    depends on DT_HAS_HPMICRO_HPM_UDC_ENABLED
    select UDC_DRIVER
    select NOCACHE_MEMORY if ARCH_HAS_NOCACHE_MEMORY_SUPPORT
```

这意味着旧版至少需要两个 DTS 条件：

```text
usb0 compatible = "hpmicro,hpm-udc"
存在 compatible = "zephyr,cdc-acm-uart" 的 CDC ACM UART 节点
DT_ALIAS(pc_usb) 指向这个 CDC ACM UART 节点
```

而当前能枚举版使用的是：

```dts
cherryusb_usb0: &usb0 {
    compatible = "hpmicro,hpm-cherryusb";
    clk-name = <CLOCK_USB0>;
    status = "okay";
};
```

也就是说，同一个 `usb0` 不能同时按旧版 UDC 和新版 CherryUSB 两种 compatible 路径跑。当前 overlay 已经切到 `hpmicro,hpm-cherryusb`，所以 `CONFIG_COM_USB` 分支即使代码还在，也不是当前实际枚举路径。

旧版调试中遇到的具体问题如下。

#### 旧版问题 A：应用层串口代码容易误导方向

旧版应用层代码：

```cpp
UsbUart::Config cfg {};
cfg.buf_size = 256;
cfg.wait_dtr = false;
cfg.assert_line_state = true;

const struct device* dev = DEVICE_DT_GET(PC_USB_NODE);
usb_.Init(dev, cfg);
```

这段只在 USB device 已经被主机枚举、CDC ACM UART 设备已经 ready 后才有意义。旧版问题是 Windows 侧长期没有虚拟串口，因此还没到 `uart_poll_out()`、DTR、波特率、RX FIFO 这些应用层问题。

当时如果继续调 `UsbUart::Send()`、`WaitHostReady()`、`UART_LINE_CTRL_DTR`，方向会偏，因为主机侧连 COM 口都没有出现。

#### 旧版问题 B：UDC path 的 connect 时机和 CherryUSB path 不同

旧 UDC driver 的代码结构是：

```text
udc_hpm_init()
-> usb_device_init(handle, int_mask)
-> enable IRQ

udc_hpm_enable()
-> handle->regs->OTGSC &= ~USB_OTGSC_VD_MASK
-> usb_device_connect(handle)
```

也就是说，Zephyr UDC path 设计上把 `connect` 放在 `.enable` 阶段。

而 CherryUSB HPM port 的代码结构是：

```text
usb_dc_init()
-> usb_device_init(handle, int_mask)
-> usb_dc_isr_connect()
```

CherryUSB HPM port 没有 Zephyr UDC 那个 `.enable` 回调层。因此同一个底层 `usb_device_init()` 如果被改成“不 connect，等上层 enable”，旧 UDC path 可能逻辑上说得通，但 CherryUSB path 会直接坏掉。

这就是后来出现过的中间失败状态：

```c
/* 错误中间状态 */
handle->regs->USBCMD &= ~USB_USBCMD_RS_MASK;
```

它的后果是：

```text
usbd_initialize() 返回
USBCMD.RS 仍为 0
主机看不到设备
```

最终能枚举版恢复为：

```c
/* Connect */
usb_dcd_connect(handle->regs);
```

这条经验很重要：**不能为了旧 UDC path 的 enable 模型，改坏 CherryUSB path 共享的 `hpm_usb_device.c`。**

#### 旧版问题 C：UDC path 的 nocache 也没有闭环

旧 UDC driver 里有两套和内存有关的东西：

```c
#define USB_HPM_DCD_DATA_SECTION __attribute__((__section__("AHB_SRAM.usb_dcd")))
```

以及：

```c
#if defined(CONFIG_NOCACHE_MEMORY)
K_HEAP_DEFINE_NOCACHE(hpm_packet_alloc_pool, 16u * 2u * 1024u);
#endif
```

但 `UDC_HPMICRO` 只是：

```conf
select NOCACHE_MEMORY if ARCH_HAS_NOCACHE_MEMORY_SUPPORT
```

而 HPM5361ICB board defconfig 曾经默认关掉：

```conf
CONFIG_SOC_ANDES_V5_PMA=n
```

这会导致 `ARCH_HAS_NOCACHE_MEMORY_SUPPORT` 不成立，`NOCACHE_MEMORY` 不一定真正打开。旧 UDC path 在这种状态下，EP0/DCD descriptor 也许放到了 `AHB_SRAM.usb_dcd`，但 packet buffer 的 nocache heap 不一定启用，控制传输和数据传输仍可能踩 cache/地址转换问题。

CherryUSB 后来卡在 `.nocache` 的问题，反过来也证明了这条线不是小问题：USB controller 访问的 queue head、qTD、EP0 buffer、bulk buffer 都不能随便放在普通 cached RAM 里。

#### 旧版问题 D：曾经看到 reset/URI 但不能证明枚举

旧 UDC path 里加过类似调试：

```c
udc_hpm_dump_regs(handle, " reset_irq");
udc_hpm_dump_regs(handle, " setup_irq");
```

它会打印：

```text
USBCMD
USBSTS
USBINTR
USBMODE
PORTSC
OTGSC
DEVICEADDR
ENDPTLISTADDR
ENDPTSETUPSTAT
ENDPTPRIME
ENDPTSTAT
ENDPTCOMPLETE
OTG_CTRL0
PHY_STATUS
```

当时最容易误判的是：看到 `reset_irq` 或 `USBSTS.URI`，就以为 Windows 已经枚举到了设备。

后来已经明确：这只能说明 controller 收到了 reset interrupt，不代表 EP0 SETUP 正常、不代表 descriptor 被主机拿到、不代表 `SET_CONFIGURATION` 完成。旧版缺少最终证据：

```text
SETUP 包连续正常
DEVICEADDR 被设置
PORTSC1.PSPD 有有效速度
USBD/UDC 上报 configured
Windows 出现虚拟串口
```

所以旧版不是“其实快好了”，而是始终缺少完整枚举闭环。

#### 旧版问题 E：DP/DM 现象说明不是单纯 PC 终端设置

之前已经在 MCU package pins 量过：

```text
MCU pin 72: DP
MCU pin 73: DM
测得都为 0 V
```

当时的关键矛盾是：

```text
软件侧以为已经设置 USBCMD.RS=1
但 USB0 PHY 没有在 MCU 引脚上表现出预期 D+ pull-up
```

这说明旧版问题不应该再回到“串口助手波特率”“DTR 没开”“PC 端没打开串口”这些应用层设置。没有 D+ attach / 没有虚拟串口时，PC 端串口参数根本还没进入问题域。

#### 旧版问题 F：`USB_DEVICE_FORCE_FULL_SPEED` 只是补丁，不是根因修复

旧版 Kconfig 里加过：

```conf
config USB_DEVICE_FORCE_FULL_SPEED
    default y if COM_USB
    depends on UDC_HPMICRO && USBD_MAX_SPEED_FULL
```

原因是 HPM SDK DCD 代码检查的是：

```c
#if defined(CONFIG_USB_DEVICE_FS) || defined(CONFIG_USB_DEVICE_FORCE_FULL_SPEED)
    ptr->PORTSC1 |= USB_PORTSC1_PFSC_MASK;
#endif
```

而 Zephyr 的 `USBD_MAX_SPEED_FULL` 不一定自动传播成 HPM SDK 代码认识的宏。

这个补丁最多解决 full-speed 强制位传递问题，不会自动解决：

```text
DTS compatible 是否正确
CDC ACM UART 节点是否存在
UDC enable 是否真正调用
USBCMD.RS 是否置位
nocache buffer 是否可被 USB DMA 访问
EP0 SETUP 是否正常进入 Zephyr USBD core
```

所以旧版调了很久仍不枚举，不是因为少一个波特率或少一个 tick，而是底层 device attach 和 EP0 枚举链没有完整跑通。

## 调试过程中遇到的问题

### 问题 1：误把裸 SDK USB 初始化思路套到 Zephyr CherryUSB sample

一开始绕到了裸 HPM SDK sample 的 `board_init_usb()`、手动 PHY prime、VBUS/ID override 等方向。后来重新看官方 Zephyr sample 后确认：

- `sdk_glue/samples/cherryusb/device/cdc_acm/cdc_acm_vcom/src/main.c` 只做：

```c
uint32_t usb_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));
cdc_acm_init(0, usb_base);
```

- `cdc_acm.c` 做描述符、接口、端点注册，最后 `usbd_initialize()`。
- Zephyr sample 的关键不是 `board_init_usb()`，而是 CMake 加入 `samples/cherryusb/config`，以及 HPM CherryUSB driver 的 `preinit`。

所以后续判断都按 Zephyr CherryUSB sample 路径，而不是裸 SDK sample 路径。

### 问题 2：`.nocache` 没有被正确接住，卡在 `usbd_desc_register()`

失败日志位置：

```text
cherryusb cdc_acm test begin base=0xf300c000
cherryusb cdc desc register
```

然后卡住，不进入：

```text
cherryusb cdc add intf0
```

源码证据：

```c
USB_NOCACHE_RAM_SECTION struct usbd_core_priv {
    ...
} g_usbd_core[CONFIG_USBDEV_MAX_BUS];
```

`usbd_desc_register()` 开头会：

```c
memset(&g_usbd_core[busid], 0, sizeof(struct usbd_core_priv));
```

也就是说，卡在 `desc register` 后，本质上是在访问 `.nocache` 中的 `g_usbd_core` 时出问题。

当时还出现过链接/Kconfig 证据：

```text
orphan section `.nocache`
CONFIG_NOCACHE_MEMORY=y was assigned but became n
unmet dependency: (SOC_FAMILY_MTK || ARCH_HAS_NOCACHE_MEMORY_SUPPORT) (=n)
```

原因：

- CherryUSB 的 `usb_config.h` 定义：

```c
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".nocache")))
```

- HPM5361ICB board defconfig 默认：

```conf
CONFIG_SOC_ANDES_V5_PMA=n
```

- 而 HPM5300 SoC 的 `SOC_ANDES_V5_PMA` 会 `select ARCH_HAS_NOCACHE_MEMORY_SUPPORT`。
- 没有打开 `SOC_ANDES_V5_PMA` 时，应用里强行写 `CONFIG_NOCACHE_MEMORY=y` 会被 Kconfig 打回 `n`。

修正：

```conf
CONFIG_SOC_ANDES_V5_PMA=y
CONFIG_NOCACHE_MEMORY=y
```

注意：这里的 `SOC_ANDES_V5_PMA` 是 HPM5300 Zephyr glue 里的 Kconfig 符号名；在 HPM5300 `soc.c` 中实际走的是 PMP 相关初始化来支撑 nocache 区域，不要把它理解成必须存在独立 PMA 外设。

修正后，`usbd_desc_register()` 可以通过，后续能看到：

```text
cherryusb cdc add intf0
cherryusb cdc add intf1
cherryusb cdc add out ep
cherryusb cdc add in ep
```

### 问题 3：`usbd_initialize()` 能返回，但主机仍看不到设备

中间失败状态：

```text
cherryusb cdc usbd_initialize
...
hpm_usb dcd_init done USBCMD=0x0 USBMODE=0x5002 PORTSC=0x1c000004 OTGSC=0x2e21
cherryusb cdc usbd_initialize done
cherryusb cdc_acm test ready
tick
tick
```

这个状态说明 controller init 走完了，但是 `USBCMD.RS` 没有置 1，device 没有真正 attach。

查代码链：

```text
usbd_initialize()
-> usb_dc_init()
-> usb_device_init()
-> usb_dcd_init()
-> usb_dcd_connect()
```

HPM device init 的关键源码应为：

```c
/* Enable interrupt mask */
usb_enable_interrupts(handle->regs, int_mask);

/* Connect */
usb_dcd_connect(handle->regs);
```

而调试过程中这个位置曾被改成：

```c
handle->regs->USBCMD &= ~USB_USBCMD_RS_MASK;
```

这会导致 `usb_device_init()` 结束时明确保持 disconnect 状态。

修正后 `usb_dcd_connect()` 的寄存器证据：

```text
hpm_usb dcd_connect before USBCMD=0x0 PORTSC=0x1c000004 OTGSC=0x200e21
hpm_usb dcd_connect after  USBCMD=0x1 PORTSC=0x1c000004 OTGSC=0x200e21
```

这里最关键的是：

```text
USBCMD: 0x0 -> 0x1
```

`USBCMD.RS=1` 后，device 才真正 run/connect。

### 问题 4：Windows 请求 0xEE 字符串描述符

成功枚举过程中仍会出现：

```text
[E/usbd_core] descriptor <type:3,index:ee> not found!
[E/usbd_core] standard request error
[E/usbd_core] Setup: bmRequestType 0x80, bRequest 0x06,
              wValue 0x03ee, wIndex 0x0000, wLength 0x0012
```

解释：

- `type:3` 是 String Descriptor。
- `index:0xee` 是 Windows 常见的 Microsoft OS String Descriptor 请求。
- 当前官方 CDC ACM sample 没有提供这个描述符，所以 CherryUSB 打 error。
- 但这个错误不是枚举阻断点，因为后面已经出现 `CONFIGURED`，Windows 也出现虚拟串口。

所以这个错误可以后续再清理，不影响“USB0 是否可枚举”的主结论。

### 问题 5：虚拟串口已经出现，但应用层 tick 接收仍未完全闭环

后续为了测试 CDC IN，应用里加了：

```c
tick_buffer = "tick\r\n";
usbd_ep_start_write(0, CDC_IN_EP, tick_buffer, sizeof(tick_buffer) - 1U);
```

当前已经观察到的状态：

```text
cdc_configured = true
g_cdc_dtr_ready = false
cdc_tx_busy = false
```

这说明 USB 枚举层已经完成，发送端点也没有忙。后续“虚拟串口收不到 tick”要单独继续查 CDC IN 传输、主机是否轮询 IN endpoint、DTR/RTS、终端打开方式或 IN callback 是否返回。它不是当前“能不能枚举”的根因。

## 当前成功枚举日志

完整关键日志：

```text
cherryusb_hpm preinit base=0xf300c000 clock=0x134050d
cherryusb_hpm clock added
cherryusb_hpm pinctrl applied
cherryusb_hpm power polarity set
cherryusb_hpm preinit done
*** Booting Zephyr OS build v4.3.0 ***
cherryusb cdc_acm test begin base=0xf300c000
cherryusb cdc desc register
cherryusb cdc add intf0
cherryusb cdc add intf1
cherryusb cdc add out ep
cherryusb cdc add in ep
cherryusb cdc usbd_initialize
hpm_usb dcd_init enter USBCMD=0x80000 USBMODE=0x5000 PORTSC=0x30000000
hpm_usb phy_init enter host=0 OTG_CTRL0=0xff080a00 PHY_CTRL0=0x0 PHY_CTRL1=0x1 PHY_STATUS=0x820017
hpm_usb phy_deinit enter OTG_CTRL0=0xff080a00 PHY_CTRL1=0x1 PHY_STATUS=0x820017
hpm_usb phy_deinit wait reset_sw OTG_CTRL0=0xff080a00 PHY_CTRL1=0x1 PHY_STATUS=0x820017
hpm_usb phy_deinit reset asserted OTG_CTRL0=0xff080a00 PHY_CTRL1=0x1 PHY_STATUS=0x800017
hpm_usb phy_deinit done OTG_CTRL0=0xff080a00 PHY_CTRL1=0x1 PHY_STATUS=0x800017
hpm_usb phy_init after deinit OTG_CTRL0=0xff080a00 PHY_CTRL0=0x0 PHY_CTRL1=0x1 PHY_STATUS=0x800017
hpm_usb phy_init pulldown enabled PHY_CTRL0=0x0
hpm_usb phy_init wait clk OTG_CTRL0=0xfd081200 PHY_CTRL0=0x0 PHY_CTRL1=0x1 PHY_STATUS=0x800017
hpm_usb phy_init clk valid OTG_CTRL0=0xfd081200 PHY_CTRL0=0x0 PHY_CTRL1=0x1 PHY_STATUS=0x81d20017
hpm_usb phy_init done OTG_CTRL0=0xfd081200 PHY_CTRL0=0x800 PHY_CTRL1=0x100001 PHY_STATUS=0x81d20017
hpm_usb dcd_init after phy USBCMD=0x80000 USBMODE=0x5000 PORTSC=0x3c000004
hpm_usb dcd_init wait controller reset USBCMD=0x80000
hpm_usb dcd_init controller reset done USBCMD=0x80000
hpm_usb dcd_init done USBCMD=0x0 USBMODE=0x5002 PORTSC=0x1c000004 OTGSC=0x2e21
hpm_usb dcd_connect before USBCMD=0x0 PORTSC=0x1c000004 OTGSC=0x200e21
hpm_usb dcd_connect after USBCMD=0x1 PORTSC=0x1c000004 OTGSC=0x200e21
cherryusb event=11
cherryusb cdc usbd_initialize done
cherryusb cdc_acm test ready
cherryusb event=1
cherryusb event=3
[E/usbd_core] descriptor <type:3,index:ee> not found!
[E/usbd_core] standard request error
[E/usbd_core] Setup: bmRequestType 0x80, bRequest 0x06, wValue 0x03ee, wIndex 0x0000, wLength 0x0012
cherryusb event=7
```

事件编号来自 CherryUSB `enum usbd_event_type`：

```text
event=11  USBD_EVENT_INIT
event=1   USBD_EVENT_RESET
event=3   USBD_EVENT_CONNECTED
event=7   USBD_EVENT_CONFIGURED
```

真正证明枚举成功的不是某一个寄存器，而是：

```text
USBCMD.RS=1
-> 主机 reset
-> CONNECTED
-> CONFIGURED
-> Windows 出现虚拟串口
```

## 查清楚的寄存器状态

### USB base 和 clock

```text
base  = 0xf300c000
clock = 0x134050d
```

来源：

```text
cherryusb_hpm preinit base=0xf300c000 clock=0x134050d
```

说明：

- `base=0xf300c000` 是 `DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0))` 得到的 USB0 寄存器基地址。
- `clock=0x134050d` 来自 overlay 的 `clk-name = <CLOCK_USB0>`。
- `cherryusb_hpm clock added` 表示 `clock_add_to_group(CLOCK_USB0, 0)` 已执行。

### OTG_CTRL0

日志：

```text
enter:      OTG_CTRL0=0xff080a00
wait clk:   OTG_CTRL0=0xfd081200
clk valid:  OTG_CTRL0=0xfd081200
```

已经确认的动作：

- `usb_phy_deinit()` 操作 `OTG_UTMI_RESET_SW` 和 `OTG_UTMI_SUSPENDM_SW`。
- `usb_phy_init()` 重新释放 UTMI reset，并拉起 UTMI suspendm。
- 成功进入 `wait clk` 后，继续等待 `PHY_STATUS.UTMI_CLK_VALID`。

这里不要单独用某一个 `OTG_CTRL0` 值判断枚举成败；它只说明 PHY init 序列走到了对应阶段。

### PHY_CTRL0

日志：

```text
enter:      PHY_CTRL0=0x0
after phy:  PHY_CTRL0=0x800
```

源码动作：

```c
ptr->PHY_CTRL0 |= USB_PHY_CTRL0_OP_MODE_SUSPENDM_ENJ_MASK;
```

说明：

- `0x800` 对应 PHY suspend/op-mode 相关控制位被置位。
- 调试过程中曾尝试在 device 分支里强行 VBUS/ID override，后来已经去掉，因为这不是官方 Zephyr CherryUSB sample 的必要路径。

### PHY_CTRL1

日志：

```text
enter:      PHY_CTRL1=0x1
after phy:  PHY_CTRL1=0x100001
```

源码动作：

```c
ptr->PHY_CTRL1 |= USB_PHY_CTRL1_UTMI_CFG_RST_N_MASK;
```

说明：

- `usb_phy_deinit()` 会先清相关 reset/suspend 控制。
- `usb_phy_init()` 再置 `UTMI_CFG_RST_N`。
- 这是 PHY 配置复位释放链条的一部分。

### PHY_STATUS

日志：

```text
enter:      PHY_STATUS=0x820017
deinit:     PHY_STATUS=0x800017
wait clk:   PHY_STATUS=0x800017
clk valid:  PHY_STATUS=0x81d20017
done:       PHY_STATUS=0x81d20017
```

关键判断：

```c
do {
    status = USB_PHY_STATUS_UTMI_CLK_VALID_GET(ptr->PHY_STATUS);
} while (status == 0);
```

说明：

- 能走出 `wait clk` 并打印 `clk valid`，说明 UTMI clock valid 条件满足。
- 之前 USB0 卡死如果发生在 `wait clk` 前后，要优先看这个状态。
- 本轮成功日志里，PHY 时钟不是阻断点。

### USBCMD

日志：

```text
dcd_init enter:          USBCMD=0x80000
dcd_init done:           USBCMD=0x0
dcd_connect before:      USBCMD=0x0
dcd_connect after:       USBCMD=0x1
```

源码动作：

```c
ptr->USBCMD &= ~USB_USBCMD_RS_MASK;
ptr->USBCMD |= USB_USBCMD_RST_MASK;
...
ptr->USBCMD |= USB_USBCMD_RS_MASK;
```

关键结论：

- `dcd_init done` 时 `USBCMD=0x0` 是合理的，因为 init 阶段会 stop controller。
- 真正 attach 的动作在 `usb_dcd_connect()`。
- 能枚举版必须看到：

```text
USBCMD.RS: 0 -> 1
```

之前不枚举的中间状态正是缺了这一步，或者这一步被错误清掉。

### USBMODE

日志：

```text
dcd_init enter:  USBMODE=0x5000
dcd_init done:   USBMODE=0x5002
```

源码动作：

```c
ptr->USBMODE &= ~USB_USBMODE_CM_MASK;
ptr->USBMODE |= USB_USBMODE_CM_SET(2);
```

结论：

- `USBMODE.CM=2` 表示 controller 被设置为 device mode。
- 这是 device 枚举的必要条件。

### PORTSC1

日志：

```text
dcd_init enter:      PORTSC=0x30000000
after phy:           PORTSC=0x3c000004
dcd_init done:       PORTSC=0x1c000004
dcd_connect before:  PORTSC=0x1c000004
dcd_connect after:   PORTSC=0x1c000004
```

源码动作：

```c
ptr->PORTSC1 &= ~USB_PORTSC1_STS_MASK;
ptr->PORTSC1 &= ~USB_PORTSC1_PTW_MASK;
```

说明：

- `PORTSC1.STS` 被清掉，选择 device 侧需要的 parallel interface signal。
- `PORTSC1.PTW` 被清掉，选择 transceiver width。
- `PORTSC1` 在 `connect` 前后值不变，不代表没 connect；connect 的直接证据是 `USBCMD.RS=1`，后续主机 reset/configured 是总线证据。

### OTGSC

日志：

```text
dcd_init done:       OTGSC=0x2e21
dcd_connect before:  OTGSC=0x200e21
dcd_connect after:   OTGSC=0x200e21
```

相关位定义来自 `usb_chipidea_reg.h`：

```text
OTGSC.ASV   = 0x400
OTGSC.AVV   = 0x200
OTGSC.ID    = 0x100
OTGSC.IDPU  = 0x20
OTGSC.VD    = 0x1
```

解释：

- `0x2e21` / `0x200e21` 中可见 `AVV`、`ASV`、`IDPU`、`VD` 等状态/控制位。
- `OTGSC` 能帮助判断 VBUS/session/id 相关状态，但不能单独证明枚举成功。
- 本轮最终还是以 `RESET -> CONNECTED -> CONFIGURED` 和 Windows 虚拟串口作为闭环证据。

### USBSTS

本轮最新成功日志没有直接打印 `USBSTS` 的最终值。

需要保留的判断原则：

- `USBSTS.URI` 或某个 `reset_irq` 只能说明看到了 USB reset interrupt。
- 它不能单独证明枚举完成。
- 后续如果继续查更深层问题，应同时看：

```text
USBSTS.URI
USBSTS.PCI
PORTSC1.PSPD
EP0 SETUP
USBD_EVENT_CONFIGURED
```

不要再只拿 `USBSTS.URI` 当作“已经枚举”的证据。

### ENDPTLISTADDR

当前没有打印 `ENDPTLISTADDR` 的值，但源码确认已经设置：

```c
usb_dcd_set_edpt_list_addr(
    handle->regs,
    core_local_mem_to_sys_address(0, (uint32_t)handle->dcd_data->qhd)
);
```

相关 buffer：

```c
static USB_NOCACHE_RAM_SECTION ATTR_ALIGN(...)
    uint8_t _dcd_data[...];

static USB_NOCACHE_RAM_SECTION usb_device_handle_t usb_device_handle[...];
```

因此 `.nocache` 和 `ENDPTLISTADDR` 是绑定问题：

- `.nocache` 不对，DCD queue head / transfer descriptor 也不可信。
- `.nocache` 正确后，EP0 descriptor 才有机会被 controller 正常访问。

## 为什么这个版本一下子就可以了

不是“突然好了”，而是阻断点被拆开后逐个清掉了。

### 第一步：先让 CherryUSB core 能访问自己的全局状态

失败时：

```text
cherryusb cdc desc register
```

之后卡死。

原因是 `g_usbd_core` 在 `.nocache`，而 `.nocache` 没有被正确支持。

修正：

```conf
CONFIG_SOC_ANDES_V5_PMA=y
CONFIG_NOCACHE_MEMORY=y
```

结果：

```text
cherryusb cdc add intf0
cherryusb cdc add intf1
cherryusb cdc add out ep
cherryusb cdc add in ep
```

### 第二步：让 controller 真正进入 run/connect

失败时：

```text
hpm_usb dcd_init done USBCMD=0x0 ...
cherryusb cdc usbd_initialize done
```

但没有虚拟串口。

原因是 `USBCMD.RS` 没有被置 1。`usbd_initialize()` 返回只能说明软件初始化函数执行完，不等价于 USB device attach。

修正：

```c
usb_dcd_connect(handle->regs);
```

结果：

```text
hpm_usb dcd_connect before USBCMD=0x0 ...
hpm_usb dcd_connect after  USBCMD=0x1 ...
```

### 第三步：主机侧完整枚举事件出现

成功时：

```text
cherryusb event=11  INIT
cherryusb event=1   RESET
cherryusb event=3   CONNECTED
cherryusb event=7   CONFIGURED
```

再加上 Windows 端出现虚拟串口，这才是完整闭环。

## 为什么之前那版不行

### 如果指旧 `CONFIG_COM_USB` 版

旧版不行，不是因为应用层没写好 `tick`，也不是因为波特率。它的失败点在 USB device 枚举链路本身。

旧版链路是：

```text
CONFIG_COM_USB
-> UsbUart
-> DT_ALIAS(pc_usb)
-> zephyr,cdc-acm-uart
-> Zephyr USB device stack next
-> UDC_HPMICRO / hpmicro,hpm-udc
```

这条链至少有六个风险点，任何一个没闭环都会导致 Windows 不出现虚拟串口：

```text
1. DTS 要有 zephyr,cdc-acm-uart 节点
2. DT_ALIAS(pc_usb) 要指向这个 CDC ACM UART 节点
3. usb0 compatible 要走 hpmicro,hpm-udc
4. UDC_HPMICRO 要真正启用
5. UDC enable 阶段要执行 usb_device_connect()
6. EP0 的 queue head、setup buffer、packet buffer 要在 USB DMA 可访问的 nocache/正确地址区
```

而当前跑通的链路是另一条：

```text
hpmicro,hpm-cherryusb
-> cherryusb_hpm_driver_preinit
-> usbd_initialize
-> usb_dc_hpm.c
-> hpm_usb_device.c
-> hpm_usb_drv.c
```

两条链的差别不是名字不同，而是生命周期不同：

```text
Zephyr UDC path:
    udc_hpm_init()
    -> usb_device_init()
    -> 等 Zephyr USBD core 调 udc_hpm_enable()
    -> usb_device_connect()

CherryUSB path:
    usbd_initialize()
    -> usb_dc_init()
    -> usb_device_init()
    -> usb_dcd_connect()
```

这就是为什么当时为了旧 UDC path 试图把 `hpm_usb_device.c` 改成“init 时不 connect，等 enable 再 connect”时，CherryUSB path 会立刻不枚举。因为 CherryUSB 没有 Zephyr UDC 那个 enable 层。

旧版还踩了 nocache 问题。`UDC_HPMICRO` 只会在 `ARCH_HAS_NOCACHE_MEMORY_SUPPORT` 成立时选择 `NOCACHE_MEMORY`，而 HPM5361ICB 之前默认关了 `CONFIG_SOC_ANDES_V5_PMA`。这意味着 UDC path 里的 `K_HEAP_DEFINE_NOCACHE()` 不一定启用，EP0/packet buffer 可能仍然不是 USB controller 可靠可访问的区域。

旧版调试时还容易被 `reset_irq` 或 `USBSTS.URI` 误导。它们只能说明主机 reset 到了 controller，不代表 SETUP 包、地址设置、配置描述符读取和 `SET_CONFIGURATION` 已经完成。旧版长期缺少的是最终闭环：

```text
EP0 SETUP 连续正常
DEVICEADDR 被设置
PORTSC1.PSPD 有有效速度
上层 USBD/UDC configured
Windows 出现虚拟串口
```

所以旧版不行的准确说法是：**不是 CDC 串口应用层没调好，而是 Zephyr UDC/HPM glue/device-tree/nocache/connect 这条底层枚举链没有跑完。**

### 如果指 CherryUSB 中间失败版

它“不行”的原因已经查清楚：

1. `.nocache` 没配好时，卡在 `usbd_desc_register()`。
2. `USBCMD.RS` 没置 1 时，`usbd_initialize()` 能返回但主机看不到设备。
3. 把裸 SDK 的手动 PHY override 套进 Zephyr sample 不是必要条件，反而会干扰判断。

## 后续调试原则

1. USB0 枚举问题先走当前 CherryUSB sample 基线，不要先回到 `CONFIG_COM_USB`。
2. 不要再让用户回头查连接器侧 DP/DM；之前已经量过 MCU package pin 72/73。
3. 判断 attach/enumeration 必须看完整链条：

```text
UTMI_CLK_VALID
USBMODE.CM=2
ENDPTLISTADDR 已设置
USBCMD.RS=1
RESET event
CONNECTED event
CONFIGURED event
主机出现虚拟串口
```

4. 单独的 `USBSTS.URI`、单独的 `reset_irq`、单独的 `usbd_initialize done` 都不是最终成功证据。
5. 现在如果继续查“虚拟串口收不到 tick”，应从 CDC IN endpoint 传输链路开始，而不是回退到 PHY 枚举问题。
