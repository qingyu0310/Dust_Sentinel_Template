# 底盘控制与 NN 学习路线详解

> 面向 `Zephyr + HPMicro + 底盘控制 + NN` 的长期学习规划
>
> 目标不是“把所有课都补完”，而是把当前工程背后的关键能力一层层补透。

---

## 目录

- [1. 这份文档解决什么问题](#1-这份文档解决什么问题)
- [2. 你现在最缺的到底是什么](#2-你现在最缺的到底是什么)
- [3. 先排优先级：哪些必须先学](#3-先排优先级哪些必须先学)
- [4. 芯片架构：现在该学什么](#4-芯片架构现在该学什么)
- [5. 启动流程：从上电到线程跑起来](#5-启动流程从上电到线程跑起来)
- [6. Zephyr 与实时系统：怎么读、先看什么](#6-zephyr-与实时系统怎么读先看什么)
- [7. 数据结构：只学对当前项目有用的部分](#7-数据结构只学对当前项目有用的部分)
- [8. 编译、链接、内存布局：为什么现在就要学](#8-编译链接内存布局为什么现在就要学)
- [9. 控制理论与离散系统](#9-控制理论与离散系统)
- [10. 状态估计与系统辨识](#10-状态估计与系统辨识)
- [11. 通信协议：CAN / UART / USB](#11-通信协议can--uart--usb)
- [12. 机器学习与 TinyML](#12-机器学习与-tinyml)
- [13. Linux：为什么你也要学](#13-linux为什么你也要学)
- [14. 汇编：为什么你也要学](#14-汇编为什么你也要学)
- [15. 中文资料优先的资源单](#15-中文资料优先的资源单)
- [16. 第一阶段建议学习顺序](#16-第一阶段建议学习顺序)
- [17. 学习周期预估](#17-学习周期预估)
- [18. 现在不用急着学什么](#18-现在不用急着学什么)
- [19. 最后的结论](#19-最后的结论)

---

## 1. 这份文档解决什么问题

这份文档不是给你解释你自己写的框架。

你当然知道：

- 线程怎么分
- 底盘链路怎么走
- 遥控、CAN、IMU、USB 在哪里

你现在真正需要的不是再看一遍工程框图，而是回答：

1. 你现在到底缺哪些底层能力
2. 这些能力为什么会在这个项目里同时冒出来
3. 哪些要现在学，哪些可以后放
4. 每一类到底该怎么学，看到什么程度算够

一句话说：

> 这份文档讲的不是“你的工程长什么样”，而是“你要靠哪些知识，才能把这套工程真正吃透”。

---

## 2. 你现在最缺的到底是什么

你现在不是“不会写工程”。

恰恰相反，你已经能把一个相当完整的工程拼起来了：

- 底盘线程
- 遥控输入
- CAN 发送
- IMU 姿态
- USB 通信
- TFLM 占位入口

所以你现在最缺的，不是“再学怎么分模块”，而是下面这 7 条穿透能力：

### 2.1 公式 -> 代码

你已经在用：

- PID
- RLS
- 四元数 EKF
- 功率约束

但还需要补到：

- 这段代码为什么这么写
- 它对应什么控制/估计公式
- 参数改了以后为什么行为会变

### 2.2 时间 -> 行为

你写的已经不是普通应用代码，而是实时系统。

所以你必须补：

- 1ms 周期意味着什么
- 抖动从哪来
- ISR 和线程边界怎么分
- 缓冲结构为什么会改变系统行为

### 2.3 配置 -> 二进制

你已经在碰：

- `Kconfig`
- `#ifdef`
- 模板
- section
- DMA buffer
- linker script

所以你已经进入“编译/链接会直接影响运行行为”的阶段了。

### 2.4 芯片 -> 系统

你现在不是写一个纯 PC 程序，而是在一个真实 MCU 上跑：

- SRAM/ITCM/Flash 怎么分
- cache 开没开
- 中断怎么进
- 启动文件怎么跳到 C
- DTS 怎么描述硬件

这些都已经会影响你。

### 2.5 数据 -> 模型

你后面要把 NN 真正接进控制链，就必须补透：

- 采什么数据
- 标签是什么
- 归一化怎么统一
- 板端推理为什么和 PC 不一致

### 2.6 源码 -> 抽象

你现在已经不适合只靠“教程式理解”了。

你需要逐渐具备：

- 看官方框架源码
- 识别抽象边界
- 追到真正发生事情的实现

### 2.7 出错 -> 定位

你后面会越来越多地碰到下面这种问题：

- 不是业务错
- 不是一个 API 错
- 而是启动、调度、消息、驱动、内存布局一起作用后的问题

所以要开始建立“多层定位能力”。

---

## 3. 先排优先级：哪些必须先学

你确实还有很多东西没学，但不能平铺。

### 3.1 第一梯队：现在必须学

1. 控制理论与离散系统
2. Zephyr / 实时系统
3. 数据结构工程子集
4. 编译 / 链接 / 内存布局
5. 芯片架构与启动流程

### 3.2 第二梯队：很重要，但可以穿插推进

1. 状态估计与辨识
2. CAN / UART / USB 协议
3. Linux 工具链能力
4. 汇编阅读能力

### 3.3 第三梯队：目标线，不是基础线

1. 机器学习
2. TinyML / TFLM 上板
3. NN 接控制链

### 3.4 第四梯队：以后一定要补，但不是现在最短板

1. 完整版编译原理体系
2. 完整版数据结构体系
3. 更深入的优化/MPC/端到端控制

---

## 4. 芯片架构：现在该学什么

### 4.1 为什么要学

因为你后面所有这些问题最后都会落回芯片：

- 代码放哪
- 栈放哪
- 中断怎么进
- 外设挂在哪
- SRAM 为什么不够
- DMA buffer 为什么要特殊对齐

你现在不用学成“芯片设计工程师”，但必须具备最基本的 MCU 架构视角。

### 4.2 你现在该学哪些点

只盯住下面这些：

- CPU 核心做什么
- 寄存器和栈是什么
- Flash / SRAM / ITCM / DTCM 分工
- 总线和外设地址空间
- 中断向量表
- cache 的基本作用
- 时钟树的基本概念
- 外设寄存器映射

### 4.3 对应当前项目看哪里

先看这些文件：

1. `D:\Zephyr_HPMicro\sdk_glue\boards\hpmicro\hpm5361icb\hpm5361icb.dts`
2. `D:\Zephyr_HPMicro\sdk_glue\boards\hpmicro\hpm5361icb\hpm5361icb-pinctrl.dtsi`
3. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\HPM5300\soc.h`
4. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\HPM5300\soc.c`
5. `D:\Zephyr_HPMicro\sdk_glue\dts\riscv\hpmicro\hpm5361.dtsi`

### 4.4 你现在学到什么程度算够

至少能回答：

1. 代码主要放在哪块存储里
2. 栈和全局数据大致放在哪
3. UART / CAN / PWM 这些外设是怎么通过 DTS 映射进系统的
4. 为什么同一个 SoC，换个 board 文件，软件看到的硬件就不一样

---

## 5. 启动流程：从上电到线程跑起来

这一节非常重要，因为它把芯片、汇编、链接、Zephyr、你的应用串在一起。

### 5.1 为什么要学

因为你后面会越来越频繁地碰到这些问题：

- 程序从哪开始执行
- 为什么一上电先不是 `main()`
- cache 什么时候开
- 栈什么时候准备好
- C 运行时什么时候起来
- Zephyr 什么时候接管
- 你的线程什么时候真的开始跑

### 5.2 当前项目对应的真实文件

按顺序先看这些：

1. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\HPM5300\start.S`
2. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\HPM5300\linker.ld`
3. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\common\entry.ld`
4. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\common\ISR.ld`
5. `D:\Zephyr\zephyr\kernel\init.c`
6. `D:\Zephyr\projects\tflm\src\main.c`
7. `D:\Zephyr\projects\tflm\projects\apps\System_startup.cpp`

### 5.3 你现在要建立的启动链路

你至少要把下面这条链在脑子里串起来：

```text
上电 / reset
  -> start.S
  -> cache / 早期初始化
  -> c_startup / __start
  -> Zephyr kernel init
  -> main()
  -> System_Bsp_Init()
  -> System_Modules_Init()
  -> System_Thread_Start()
  -> 各线程开始运行
```

### 5.4 这条线现在怎么学

不要一上来把汇编逐行背掉。

你现在只需要先搞懂：

1. `start.S` 在干什么
2. linker script 在决定什么
3. `kernel/init.c` 为什么重要
4. 为什么真正的“你的系统开始跑起来”不等于“进入 main”

### 5.5 学到什么程度算够

至少能回答：

1. 为什么 MCU 上电后不会直接进 `main`
2. 栈是什么时候可用的
3. Zephyr 是怎么把初始化和线程系统拉起来的
4. 你的 `main -> System_startup -> thread_start` 在整个启动链里处于什么位置

---

## 6. Zephyr 与实时系统：怎么读、先看什么

### 6.1 为什么 Zephyr 要作为主线

你现在用的是 Zephyr，不是 FreeRTOS。

所以：

- 如果你想搞懂“你的项目为什么这样跑”，主线必须是 Zephyr
- FreeRTOS 最多只能当 RTOS 共性补充

### 6.2 你现在最该搞懂的不是 API，而是这些事

- 线程怎么创建
- 调度怎么发生
- `msgq` 和 `zbus` 到底干了什么
- 阻塞和唤醒怎么发生
- 为什么消息发布有实时成本

### 6.3 Zephyr 源码阅读顺序

#### 第 0 层：从你自己的入口切进去

1. `D:\Zephyr\projects\tflm\src\main.c`
2. `D:\Zephyr\projects\tflm\projects\apps\System_startup.cpp`
3. `D:\Zephyr\projects\tflm\projects\thread\Kconfig`
4. `D:\Zephyr\projects\tflm\projects\thread\thread.hpp`

#### 第 1 层：先看 Zephyr 的接口层

1. `D:\Zephyr\zephyr\include\zephyr\kernel.h`
2. `D:\Zephyr\zephyr\include\zephyr\kernel\thread.h`
3. `D:\Zephyr\zephyr\include\zephyr\kernel\thread_stack.h`
4. `D:\Zephyr\zephyr\include\zephyr\kernel\msg_q.h`
5. `D:\Zephyr\zephyr\include\zephyr\kernel\sem.h`

#### 第 2 层：再看 kernel 实现

1. `D:\Zephyr\zephyr\kernel\init.c`
2. `D:\Zephyr\zephyr\kernel\thread.c`
3. `D:\Zephyr\zephyr\kernel\sched.c`
4. `D:\Zephyr\zephyr\kernel\msg_q.c`
5. `D:\Zephyr\zephyr\kernel\sem.c`

#### 第 3 层：读你项目真正在跑的消息机制

1. `D:\Zephyr\zephyr\include\zephyr\zbus\zbus.h`
2. `D:\Zephyr\zephyr\subsys\zbus\zbus.c`
3. `D:\Zephyr\projects\tflm\topic\remote_to\remote_to.hpp`
4. `D:\Zephyr\projects\tflm\topic\to_can_tx\to_can_tx.hpp`
5. `D:\Zephyr\projects\tflm\topic\imu_to\imu_to.cpp`
6. `D:\Zephyr\projects\tflm\modules\imu\imu.cpp`

#### 第 4 层：再看驱动接口层

1. `D:\Zephyr\zephyr\include\zephyr\drivers\uart.h`
2. `D:\Zephyr\zephyr\include\zephyr\drivers\can.h`
3. `D:\Zephyr\zephyr\include\zephyr\drivers\gpio.h`
4. `D:\Zephyr\zephyr\include\zephyr\drivers\pwm.h`
5. `D:\Zephyr\zephyr\include\zephyr\devicetree.h`

#### 第 5 层：最后下钻到你板子实际用的实现

1. `D:\Zephyr_HPMicro\sdk_glue\drivers\serial\uart_hpmicro.c`
2. `D:\Zephyr_HPMicro\sdk_glue\drivers\can\can_hpmicro.c`
3. `D:\Zephyr_HPMicro\sdk_glue\drivers\usb\udc\udc_hpmicro.c`
4. `D:\Zephyr_HPMicro\sdk_glue\drivers\usb\cherryusb\cherryusb_hpmicro.c`

### 6.4 你现在学到什么程度算够

至少能回答：

1. `Thread<2048>` 最后怎么变成 Zephyr 线程
2. `k_msgq_get()` 和 `zbus_chan_pub()` 到底做了什么
3. 为什么 `publish` / `put` 不是“零成本动作”
4. Zephyr 抽象层和 HPMicro 具体驱动层是怎么接起来的

---

## 7. 数据结构：只学对当前项目有用的部分

### 7.1 为什么要学

因为你当前很多行为，其实本质上都是数据结构选择的结果。

你最该学的不是刷题课，而是：

- 定长数组
- 结构体边界
- 环形缓冲
- 固定容量消息
- 平铺矩阵

### 7.2 对应当前项目看哪里

1. `D:\Zephyr\projects\tflm\drivers\communication\uart\uart.hpp`
2. `D:\Zephyr\projects\tflm\drivers\communication\uart\uart.cpp`
3. `D:\Zephyr\projects\tflm\drivers\communication\usb\usb.hpp`
4. `D:\Zephyr\projects\tflm\drivers\communication\usb\usb.cpp`
5. `D:\Zephyr\projects\tflm\projects\thread\chassis\trd_chassis.hpp`
6. `D:\Zephyr\projects\tflm\topic\remote_to\remote_to.hpp`
7. `D:\Zephyr\projects\tflm\algorithm\identify\rls\rls.hpp`

### 7.3 你现在只要盯住这些问题

1. 为什么这里大量用定长数组
2. 为什么这里的消息结构都尽量扁平
3. 为什么 UART/USB 接收更适合环形缓冲
4. 为什么矩阵在板端常被平铺成 `float[]`

### 7.4 学到什么程度算够

至少能解释：

1. 为什么这里不用链表和动态容器
2. 为什么固定容量结构更适合实时系统
3. 为什么“结构体设计”本身就是接口设计

---

## 8. 编译、链接、内存布局：为什么现在就要学

### 8.1 为什么要学

因为你现在的工程已经不允许你把“编译”当黑盒了。

你已经在碰：

- `Kconfig`
- `#ifdef`
- 模板
- 静态对象
- DMA buffer
- linker script
- 模型数组

### 8.2 当前项目看哪里

1. `D:\Zephyr\projects\tflm\projects\thread\Kconfig`
2. `D:\Zephyr\projects\tflm\algorithm\Kconfig`
3. `D:\Zephyr\projects\tflm\projects\CMakeLists.txt`
4. `D:\Zephyr\projects\tflm\drivers\communication\usb\usb.cpp`
5. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\HPM5300\linker.ld`
6. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\common\linker.ld`

### 8.3 你现在最该搞懂的

1. `CONFIG_TRD_*` 怎么决定系统最终长什么样
2. 为什么模板一般放头文件
3. 为什么 DMA buffer 需要特殊 section 和对齐
4. 为什么模型上板一定会变成 flash / RAM 问题

### 8.4 学到什么程度算够

至少能回答：

1. 配置怎么控制代码进不进镜像
2. 静态对象大概何时初始化
3. linker script 大概在控制什么
4. 为什么“内存不够”常常不是一句增大数组那么简单

---

## 9. 控制理论与离散系统

### 9.1 为什么要学

因为你的底盘控制核心已经不是“会不会 PID”，而是：

- 你的 PID 现在到底变形成什么样了
- 1ms 周期下它到底是什么离散系统
- 功率限制接进来以后，闭环为什么变了

### 9.2 当前项目看哪里

1. `D:\Zephyr\projects\tflm\algorithm\controller\pid\pid.cpp`
2. `D:\Zephyr\projects\tflm\algorithm\controller\pid\pid.hpp`
3. `D:\Zephyr\projects\tflm\projects\thread\chassis\trd_chassis.cpp`
4. `D:\Zephyr\projects\tflm\algorithm\controller\power_ctrl\power_ctrl.cpp`
5. `D:\Zephyr\projects\tflm\scripts\root_locus.py`

### 9.3 你现在只要先盯住这些点

- PID 各项在代码里怎么落地
- `dt = 0.001` 的意义
- 限幅、死区、低通对行为的影响
- 功率限制为什么改变控制输出

### 9.4 学到什么程度算够

至少能回答：

1. `drive_velocity` 和 `drive_torque` 为什么串级
2. 采样周期一变，为什么同样参数行为会变
3. 功率限制为什么会让“裸 PID 直觉”失效

---

## 10. 状态估计与系统辨识

### 10.1 为什么要学

因为你已经在真实项目里用上了：

- RLS
- EKF
- 真实 `dt`

这不是以后再学的理论，是当前工程的一部分。

### 10.2 当前项目看哪里

1. `D:\Zephyr\projects\tflm\algorithm\identify\rls\rls.hpp`
2. `D:\Zephyr\projects\tflm\algorithm\controller\power_ctrl\power_ctrl.cpp`
3. `D:\Zephyr\projects\tflm\algorithm\filter\quaternion_ekf\quaternion_ekf.cpp`
4. `D:\Zephyr\projects\tflm\modules\imu\imu.cpp`

### 10.3 你现在只要先盯住这些问题

1. RLS 在这里到底在估什么
2. 遗忘因子为什么会影响跟踪速度
3. EKF 的状态、观测、门限分别是什么
4. 为什么真实 `dt` 会影响姿态估计

### 10.4 学到什么程度算够

至少能做到：

1. 说清当前 RLS 的估计对象
2. 说清当前 EKF 的状态/观测结构
3. 不再把滤波器当黑盒

---

## 11. 通信协议：CAN / UART / USB

### 11.1 为什么要学

因为你现在已经在写：

- CAN 发送线程
- UART 接收
- USB CDC ACM

如果协议层不补透，很多问题都会被你误判成“代码 bug”。

### 11.2 当前项目看哪里

1. `D:\Zephyr\projects\tflm\projects\thread\can\trd_can_tx.cpp`
2. `D:\Zephyr\projects\tflm\drivers\communication\can\can.cpp`
3. `D:\Zephyr\projects\tflm\drivers\communication\uart\uart.cpp`
4. `D:\Zephyr\projects\tflm\drivers\communication\usb\usb.cpp`
5. `D:\Zephyr\projects\tflm\topic\remote_to\remote_to.hpp`

### 11.3 你现在只要先盯住这些点

- CAN 的帧/过滤器/仲裁
- UART 中断与 DMA 接收模型
- USB CDC ACM 的枚举与端点

### 11.4 学到什么程度算够

至少能回答：

1. 为什么 CAN/UART/USB 的“收发逻辑”完全不同
2. 为什么通信故障不一定是业务代码错
3. 为什么缓冲和消费速度会决定系统行为

---

## 12. 机器学习与 TinyML

### 12.1 为什么要学

因为你目标不是“学一个神经网络例子”，而是把 NN 真接到底盘控制链。

### 12.2 当前项目看哪里

1. `D:\Zephyr\projects\tflm\scripts\train_pid.py`
2. `D:\Zephyr\projects\tflm\projects\thread\pc\trd_pc.cpp`
3. `D:\Zephyr\projects\tflm\algorithm\tflm\tflm.cpp`
4. `D:\Zephyr\projects\tflm\projects\thread\tflm\trd_tflm.cpp`
5. `D:\Zephyr\projects\tflm\projects\thread\chassis\trd_chassis.cpp`

### 12.3 你现在只要先盯住这些问题

1. 采什么数据
2. 标签是什么
3. 归一化怎么统一
4. 板端推理和 PC 推理为什么会不同

### 12.4 学到什么程度算够

至少能自己打通：

```text
采数
  -> 训练
  -> 导出
  -> 板端推理
  -> 接控制分支
```

---

## 13. Linux：为什么你也要学

### 13.1 为什么要学

Linux 不一定直接改你的控制算法，但会直接改你的开发效率。

你后面会越来越多地依赖：

- 搜源码
- 跑脚本
- 看构建
- 处理日志
- 看 ELF / 符号 / map
- 用 `gdb`

### 13.2 现在学什么最值钱

- 文件系统
- 进程
- 管道和重定向
- `grep/find/sed/awk`
- `make/cmake/ninja`
- `gdb`
- shell 脚本

### 13.3 学到什么程度算够

至少能熟练完成：

1. 快速定位一个符号/配置来源
2. 读懂基本构建错误
3. 写简单脚本处理日志和数据
4. 用符号工具辅助排查问题

---

## 14. 汇编：为什么你也要学

### 14.1 为什么要学

因为后面你一定会碰到：

- 启动文件
- 中断入口
- 栈帧
- 底层崩溃
- 优化后的反汇编

### 14.2 现在学什么最值钱

- 寄存器
- 栈
- 调用约定
- 参数传递
- 返回值
- 函数 prologue / epilogue
- 中断现场保存
- C 代码和反汇编对应

### 14.3 回到当前项目里看哪里

1. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\HPM5300\start.S`
2. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\common\ISR.ld`
3. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\common\entry.ld`
4. `D:\Zephyr_HPMicro\sdk_glue\soc\hpmicro\HPM5300\linker.ld`

### 14.4 学到什么程度算够

至少能做到：

1. 看懂一个普通函数的基本栈帧
2. 知道启动文件和链接脚本各自干什么
3. 看到反汇编时不至于完全失去方向

---

## 15. 中文资料优先的资源单

### 15.1 控制理论

- B站：`卢京潮 自动控制原理`
- 胡寿松《自动控制原理》
- 英文补充：MIT OCW `6.302`、`16.30`

### 15.2 Zephyr / 实时系统

- Zephyr 官方文档
- 你当前这条 Zephyr 源码阅读顺序
- FreeRTOS 只作 RTOS 共性补充

### 15.3 数据结构

- B站 / MOOC：`陈越 数据结构`
- 严蔚敏《数据结构（C语言版）》

### 15.4 编译 / 链接 / 装载

- 《程序员的自我修养：链接、装载与库》
- 《深入理解计算机系统》中文版
- B站：`哈工大 编译原理`

### 15.5 状态估计 / 辨识

- 《卡尔曼滤波与组合导航原理》
- 英文补充：Solà 的 `A micro Lie theory for state estimation in robotics`

### 15.6 机器学习 / TinyML

- B站：`李宏毅 机器学习`
- TensorFlow 中文回归教程
- Google 机器学习速成课程（中文）

### 15.7 Linux

- 《鸟哥的 Linux 私房菜》
- B站：`Linux 基础教程`
- B站：`gdb 教程`

### 15.8 汇编

- 王爽《汇编语言》
- B站：`汇编语言 王爽`

---

## 16. 第一阶段建议学习顺序

如果你现在就开始，我建议这样排：

1. 启动流程 + Zephyr 线程 / 调度
2. 控制理论 + 离散系统
3. 数据结构工程子集
4. 编译 / 链接 / 内存布局
5. 芯片架构
6. 状态估计 / 辨识
7. 通信协议
8. Linux
9. 汇编
10. ML / TinyML

为什么这样排：

```text
先搞清系统怎么活起来、怎么调度、怎么跑
再搞清控制和数据怎么在里面流动
最后再去接 NN
```

---

## 17. 学习周期预估

这一节不是给你压力，而是帮你建立正常预期。

你这条路线不是“看完几门网课就结束”，而是一个会持续 6 到 12 个月都还在继续增厚的学习过程。

### 17.1 8 周：起主干

如果你能连续投入 8 周，而且不东一榔头西一棒子，最现实的结果是：

- 启动流程有整体概念
- 知道 Zephyr 线程、`msgq`、`zbus` 大概怎么工作
- 能把 `pid.cpp` 和 `trd_chassis.cpp` 对起来看
- 知道当前项目里数据结构、编译、芯片、驱动分别对应哪一层

这个阶段的关键词不是“掌握”，而是：

```text
不再完全黑盒
```

你会开始有一种感觉：

- 看源码不再全是碎片
- 遇到问题时知道先去哪一层找

### 17.2 3 个月：主线立住

如果你能稳定学到 3 个月，通常应该达到：

- 能把启动流程从 `start.S` 串到 `main()` 再串到线程启动
- 能解释 Zephyr 线程、调度、消息机制和你项目之间的关系
- 能说清当前 PID、RLS、EKF 分别在干什么
- 能看懂当前 UART/USB/CAN 链路的基本行为
- 对 `Kconfig`、模板、linker script、section 不再陌生

这个阶段的关键词是：

```text
主干建立
```

你会开始从“我知道要学什么”，进入“我已经能把几条主线串起来”。

### 17.3 6 个月：开始质变

如果你能持续 6 个月，这时候通常会发生第一次明显质变：

- 你不再只是会改代码，而是能判断设计边界
- 你能从控制、实时系统、通信、内存布局多个角度看同一个问题
- RLS / EKF 这类东西不再只是“能调参”
- Linux / 汇编会开始真的变成调试工具，而不是额外负担
- 你能比较稳地推进 `采数 -> 训练 -> 导出 -> 板端推理`

这个阶段的关键词是：

```text
从会写项目，走向会看系统
```

### 17.4 1 年：体系开始闭环

如果你按这条路线认真推进 1 年，最理想的结果不是“所有东西学完了”，而是：

- 你对芯片、启动、RTOS、驱动、控制、估计、ML 之间的关系有了成体系理解
- 你不再依赖别人帮你分层定位问题
- 你会逐渐形成自己的阅读顺序、调试顺序和实现习惯
- 这份路线图里的很多条线会从“学习任务”变成“日常工具”

这个阶段的关键词是：

```text
体系化
```

### 17.5 你现在最该用的周期视角

对你来说，最重要的不是问“总共几年学完”，而是分段看：

#### 第一阶段：8 到 12 周

只盯住这几条：

1. 启动流程
2. Zephyr / 实时系统
3. 控制理论与离散系统
4. 数据结构工程子集
5. 编译 / 链接 / 内存布局

目标不是全懂，而是：

```text
把主干立住
```

#### 第二阶段：3 到 6 个月

再往里吃：

1. 状态估计 / 辨识
2. 通信协议
3. Linux / 汇编
4. 数据采集与板端推理链路

目标是：

```text
开始形成系统视角
```

#### 第三阶段：6 到 12 个月

这时候再追求：

1. 把各条线互相串起来
2. 把 NN 真正更稳地接进控制链
3. 形成自己的实现与调试方法论

目标是：

```text
从学知识，走向会做系统
```

### 17.6 一句话判断现在该怎么看周期

如果你现在问“这条路到底多长”，我给你的现实答案是：

- `1 个月`：只能起头
- `3 个月`：能把主干立住
- `6 个月`：开始有明显质变
- `1 年`：这条路线才真正进入稳定收获期

所以现在最好的心态不是：

```text
我还有这么多没学，完了
```

而是：

```text
先拿 8 到 12 周，把第一阶段打穿
```

---

## 18. 现在不用急着学什么

### 17.1 不用先把所有学科都铺开

你现在的危险不是“学得不够多”，而是“方向太多，注意力被摊平”。

### 17.2 不用先刷完整算法题体系

当前最值钱的不是红黑树和图论模板，而是环形缓冲、固定消息、平铺矩阵这些工程数据结构。

### 17.3 不用先完整学完编译原理前端

你现在先搞懂：

- 预处理
- 配置
- 模板
- 链接
- 内存布局

比学 LR 表更值钱。

### 17.4 不用先把 NN 做复杂

先让数据跑起来、模型能导出、板端能推理，再谈效果。

---

## 19. 最后的结论

你现在的状态不是“东缺一点，西缺一点，所以很乱”。

更准确地说，是：

> 你已经走到了一个阶段：  
> 工程已经能搭起来，  
> 所以芯片、启动、RTOS、数据结构、编译、控制、估计、通信、ML 这些基础开始同时冒头。

这不是坏事，反而说明你做的已经不是初级项目了。

所以现在最合理的做法，不是焦虑“还有好多没学”，而是：

```text
先按优先级一层层补：
启动
  -> Zephyr / 实时系统
  -> 控制
  -> 数据结构
  -> 编译链接内存
  -> 芯片架构
  -> 估计与辨识
  -> 通信
  -> Linux / 汇编
  -> ML / TinyML
```

一句话总结：

> **你现在不是缺框架，而是缺把框架背后的整套系统本质逐步看透。**
