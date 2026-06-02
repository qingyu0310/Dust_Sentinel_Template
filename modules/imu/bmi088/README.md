# BMI088 使用流程

本文说明 `modules/imu/bmi088` 里的 BMI088 Source 如何接入当前 IMU 模块。

## 1. 硬件与设备树

`board_rm_c` 的 SPI/BMI088 设备树在这里：

```text
projects/boards/st/board_rm_c/stm32f407igh6.overlay
```

当前 overlay 使用 STM32F407 SPI1：

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| SPI1_SCK | PB3 | SPI 时钟 |
| SPI1_MISO | PB4 | SPI MISO |
| SPI1_MOSI | PA7 | SPI MOSI |
| ACC_CS | PA4 | BMI088 加速度计片选，对应 `reg = <0>` |
| GYRO_CS | PB0 | BMI088 陀螺仪片选，对应 `reg = <1>` |
| ACC_INT | PC4 | 加速度计中断脚，当前自定义 Source 暂未使用 |
| GYRO_INT | PC5 | 陀螺仪中断脚，当前自定义 Source 暂未使用 |

引脚映射参考 DJI RoboMaster Development Board Type C User Manual 的 Six-Axis IMU (BMI088) 表；SPI1 的 SCK/MISO/MOSI 也和 `temp/spi.c` 中的旧 STM32 HAL 配置一致。

overlay 提供了 3 个 alias：

```dts
imu-spi = &spi1;
bmi088-accel = &bmi088_accel;
bmi088-gyro = &bmi088_gyro;
```

C/C++ 里使用 alias 时要把 `-` 写成 `_`：

```cpp
DT_ALIAS(bmi088_accel)
DT_ALIAS(bmi088_gyro)
```

## 2. Kconfig

使用 BMI088 Source 时打开：

```conf
CONFIG_TRD_IMU=y
CONFIG_MOD_DEV_IMU_BMI088=y
```

`MOD_DEV_IMU_BMI088` 会选择 `COM_SPI`，`COM_SPI` 会选择 Zephyr 的 `SPI` 和 `GPIO`。这里需要 `GPIO` 是因为 BMI088 两个片选都通过 `cs-gpios` 控制。

如果不走 `TRD_IMU`，至少需要：

```conf
CONFIG_MOD_DEV_IMU=y
CONFIG_MOD_DEV_IMU_BMI088=y
```

## 3. 初始化流程

`System_startup` 不需要、也不应该包含 BMI088 专用注册代码。只要打开 `CONFIG_TRD_IMU` 和 `CONFIG_MOD_DEV_IMU_BMI088`，应用侧保持普通 IMU 模块初始化即可：

```cpp
void System_Modules_Init()
{
    imu::thread_init();
}
```

默认流程集中在 `thread::imu::thread_init()` 内部：

- `bmi088::RegisterFromDevicetree()` 从 `DT_ALIAS(bmi088_accel)` 和 `DT_ALIAS(bmi088_gyro)` 构造两份 `spi_dt_spec`。
- `thread::imu::RegisterSource()` 将 BMI088 注册为当前 IMU Source。
- `imu_.Init()` 调用 `Bmi088::Init()`，完成 SPI ready 检查、chip id 校验、软复位和寄存器配置。

`bmi088::Register(config)` 仍然保留给测试或特殊板级配置使用。如果在 `thread_init()` 之前已经手动注册了 Source，默认 DT 注册不会覆盖已有 Source。

## 4. 数据流

运行时的数据流是：

```text
BMI088 SPI -> bmi088::Bmi088::Read()
           -> thread::imu::ImuWorker
           -> QuaternionEkf
           -> zbus pub_imu_to
```

上层模块不要直接依赖 BMI088。姿态、角速度、加速度和温度统一从 `topic/imu_to` 订阅。

## 5. 常见问题

- `AccelChipId`：优先检查 PA4 片选、SPI mode 3、`bmi088-accel` alias 和 `reg = <0>`。
- `GyroChipId`：优先检查 PB0 片选、`bmi088-gyro` alias 和 `reg = <1>`。
- `AccelNotReady/GyroNotReady`：检查 `CONFIG_MOD_DEV_IMU_BMI088`、`CONFIG_COM_SPI`、SPI1 pinctrl 和 `cs-gpios`。
- IMU 线程没有输出：确认 `CONFIG_TRD_IMU` 和 `CONFIG_MOD_DEV_IMU_BMI088` 已打开，并检查两个 BMI088 alias 是否存在。
- 数据方向不符合车体坐标系：先不要改 EKF，在 `Bmi088::Read()` 转换后做轴向映射或符号修正。
