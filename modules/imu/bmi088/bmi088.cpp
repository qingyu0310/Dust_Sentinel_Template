/**
 * @file bmi088.cpp
 * @author qingyu
 * @brief BMI088 IMU Source 实现。
 * @version 0.1
 * @date 2026-06-02
 */

#include "bmi088.hpp"
#include "bmi088_reg.hpp"

#include <math.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

namespace bmi088 {

namespace {

/**
 * @brief BMI088 寄存器初始化项。
 */
struct RegConfig {
    uint8_t reg;
    uint8_t value;
};

constexpr RegConfig kAccelConfig[] {
    {reg::kAccPwrCtrl,      reg::kAccPowerOn            },
    {reg::kAccPwrConf,      reg::kAccPowerActive        },
    {reg::kAccConf,         static_cast<uint8_t>(reg::kAccNormal | reg::kAcc800Hz | reg::kAccConfMustSet)},
    {reg::kAccRange,        reg::kAccRange6g            },
    {reg::kAccInt1IoCtrl,   static_cast<uint8_t>(reg::kAccInt1Enable | reg::kAccInt1PushPull | reg::kAccInt1Low)},
    {reg::kAccIntMapData,   reg::kAccInt1Drdy           },
};

constexpr RegConfig kGyroConfig[] {
    {reg::kGyroRange,       reg::kGyro2000Dps           },
    {reg::kGyroBandwidth,   reg::kGyro2000Hz230Hz       },
    {reg::kGyroLpm1,        reg::kGyroNormalMode        },
    {reg::kGyroCtrl,        reg::kGyroDrdyOn            },
    {reg::kGyroIoConf,      reg::kGyroInt3PushPullLow   },
    {reg::kGyroIoMap,       reg::kGyroDrdyInt3          },
};

static Bmi088 bmi088_ {};

} // namespace

Bmi088& Instance()
{
    return bmi088_;
}

/**
 * @brief 从设备树 alias 构造 BMI088 配置。
 */
bool ConfigureFromDevicetree(uint32_t period_ms, bool auto_calibration)
{
    constexpr uint32_t kSpiOperation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA;

    static const struct spi_dt_spec accel =
        SPI_DT_SPEC_GET(DT_ALIAS(bmi088_accel), kSpiOperation, 0);
    static const struct spi_dt_spec gyro =
        SPI_DT_SPEC_GET(DT_ALIAS(bmi088_gyro), kSpiOperation, 0);

    Bmi088::Config cfg {};
    cfg.accel     = &accel;
    cfg.gyro      = &gyro;
    cfg.period_ms = period_ms;
    cfg.auto_calibration = auto_calibration;

    for (uint8_t axis = 0; axis < 3; axis++) {
        cfg.static_calibration.gyro_offset[axis]  = reg::kStaticGyroOffset[axis];
        cfg.static_calibration.gyro_scale[axis]   = reg::kStaticGyroScale[axis];
        cfg.static_calibration.accel_offset[axis] = reg::kStaticAccelOffset[axis];
        cfg.static_calibration.accel_scale[axis]  = reg::kStaticAccelScale[axis];
    }

    bmi088_.Configure(cfg);
    return true;
}

Bmi088::Bmi088(const Config& config)
{
    Configure(config);
}

/**
 * @brief 更新 BMI088 当前运行配置。
 *
 * 重新配置时同步清空运行时自动校准结果，避免旧校准残留到新配置。
 */
void Bmi088::Configure(const Config& config)
{
    config_ = config;
    if (config_.period_ms == 0) {
        config_.period_ms = 1;
    }
}

uint32_t Bmi088::PeriodMs() const
{
    return config_.period_ms;
}

Error Bmi088::LastError() const
{
    return last_error_;
}

/**
 * @brief 初始化 BMI088 两个子器件，并在需要时执行自动校准。
 */
bool Bmi088::Init()
{
    last_error_ = Error::None;

    if (config_.accel == nullptr || !accel_.Init(*config_.accel)) {
        last_error_ = Error::AccelNotReady;
        return false;
    }

    if (config_.gyro  == nullptr || !gyro_.Init(*config_.gyro)) {
        last_error_ = Error::GyroNotReady;
        return false;
    }

    if (!InitAccel() || !InitGyro()) {
        return false;
    }

    if (config_.auto_calibration) {
        return AutoCalibrate();
    }

    /* 关闭自动校准时，仅使用配置里的静态校准值。 */
    return true;
}

/**
 * @brief 读取一帧原始样本并换算为工程单位。
 *
 * 输出前会叠加静态校准参数与启动阶段求得的运行时校准偏移。
 */
bool Bmi088::Read(Sample& sample)
{
    RawSample raw {};
    if (!ReadRaw(raw)) {
        return false;
    }

    for (uint8_t i = 0; i < 3; i++) {
        const float accel = static_cast<float>(raw.accel[i]) * kAccel6gSensitivity;
        const float gyro  = static_cast<float>(raw.gyro[i])  * kGyro2000Sensitivity;

        sample.accel[i] = Correct(accel,
                                 config_.static_calibration.accel_offset[i],
                                  config_.static_calibration.accel_scale[i]);
        sample.gyro[i]  = Correct(gyro,
                                  config_.static_calibration.gyro_offset[i],
                                  config_.static_calibration.gyro_scale[i]);
    }

    sample.temperature = static_cast<float>(raw.temperature) * kTemperatureFactor + kTemperatureOffset;
    sample.dt = static_cast<float>(config_.period_ms) * 0.001f;
    last_error_ = Error::None;

    return true;
}

/**
 * @brief 从 BMI088 寄存器读取一帧原始样本。
 */
bool Bmi088::ReadRaw(RawSample& raw)
{
    uint8_t accel_raw[6] {};
    uint8_t gyro_raw[6] {};
    uint8_t temp_raw[2] {};

    if (!ReadAccel(reg::kAccXoutL,  accel_raw, sizeof(accel_raw))  ||
        !ReadGyro (reg::kGyroXoutL, gyro_raw,  sizeof(gyro_raw))   ||
        !ReadAccel(reg::kAccTempM,  temp_raw,  sizeof(temp_raw)))  {
        last_error_ = Error::ReadFailed;
        return false;
    }

    raw.accel[0] = ToInt16(accel_raw[0], accel_raw[1]);
    raw.accel[1] = ToInt16(accel_raw[2], accel_raw[3]);
    raw.accel[2] = ToInt16(accel_raw[4], accel_raw[5]);

    raw.gyro[0]  = ToInt16(gyro_raw[0], gyro_raw[1]);
    raw.gyro[1]  = ToInt16(gyro_raw[2], gyro_raw[3]);
    raw.gyro[2]  = ToInt16(gyro_raw[4], gyro_raw[5]);

    raw.temperature = static_cast<int16_t>((static_cast<uint16_t>(temp_raw[0]) << 3) | (static_cast<uint16_t>(temp_raw[1]) >> 5));
    if (raw.temperature > 1023) {
        raw.temperature -= 2048;
    }

    last_error_ = Error::None;
    return true;
}

/**
 * @brief 初始化加速度计子器件。
 */
bool Bmi088::InitAccel()
{
    uint8_t chip_id = 0;
    (void)ReadAccelReg(reg::kAccChipId, chip_id);
    k_msleep(1);
    (void)ReadAccelReg(reg::kAccChipId, chip_id);
    k_msleep(1);

    if (!WriteAccel(reg::kAccSoftReset, reg::kAccResetValue)) {
        last_error_ = Error::AccelConfig;
        return false;
    }
    k_msleep(80);

    (void)ReadAccelReg(reg::kAccChipId, chip_id);
    k_msleep(1);
    if (!ReadAccelReg(reg::kAccChipId, chip_id) || chip_id != reg::kAccChipIdValue) {
        last_error_ = Error::AccelChipId;
        return false;
    }

    for (const auto& cfg : kAccelConfig) 
    {
        if (!WriteCheckedAccel(cfg.reg, cfg.value)) {
            last_error_ = Error::AccelConfig;
            return false;
        }
        k_msleep(1);
    }

    return true;
}

/**
 * @brief 初始化陀螺仪子器件。
 */
bool Bmi088::InitGyro()
{
    uint8_t chip_id = 0;
    (void)ReadGyroReg(reg::kGyroChipId, chip_id);
    k_msleep(1);
    (void)ReadGyroReg(reg::kGyroChipId, chip_id);
    k_msleep(1);

    if (!WriteGyro(reg::kGyroSoftReset, reg::kGyroResetValue)) {
        last_error_ = Error::GyroConfig;
        return false;
    }
    k_msleep(80);

    (void)ReadGyroReg(reg::kGyroChipId, chip_id);
    k_msleep(1);
    if (!ReadGyroReg(reg::kGyroChipId, chip_id) || chip_id != reg::kGyroChipIdValue) {
        last_error_ = Error::GyroChipId;
        return false;
    }

    for (const auto& cfg : kGyroConfig) 
    {
        if (!WriteCheckedGyro(cfg.reg, cfg.value)) {
            last_error_ = Error::GyroConfig;
            return false;
        }
        k_msleep(1);
    }

    return true;
}

/**
 * @brief 在启动阶段静止采样，自动估计零偏。
 *
 * 当前实现会：
 * 1. 平均陀螺输出作为运行时零偏；
 * 2. 将平均加速度模长校到 1 g。
 *
 * 因此前提是初始化阶段 IMU 尽量保持静止。
 */
bool Bmi088::AutoCalibrate()
{
    if (!config_.auto_calibration || config_.calibration_samples == 0U) {
        return true;
    }

    if (config_.calibration_settle_ms > 0U) {
        k_msleep(config_.calibration_settle_ms);
    }

    float gyro_sum[3]  = {0.0f, 0.0f, 0.0f};
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};

    for (uint16_t sample_idx = 0; sample_idx < config_.calibration_samples; sample_idx++) 
    {
        RawSample raw {};
        if (!ReadRaw(raw)) {
            return false;
        }

        for (uint8_t axis = 0; axis < 3; axis++) {
            gyro_sum[axis]  += static_cast<float>(raw.gyro[axis])  * kGyro2000Sensitivity;
            accel_sum[axis] += static_cast<float>(raw.accel[axis]) * kAccel6gSensitivity;
        }

        k_msleep(config_.period_ms);
    }

    const float inv_samples = 1.0f / static_cast<float>(config_.calibration_samples);
    float accel_mean[3] = {0.0f, 0.0f, 0.0f};
    float accel_norm_sq = 0.0f;

    for (uint8_t axis = 0; axis < 3; axis++) 
    {
        config_.static_calibration.gyro_offset[axis] = gyro_sum[axis] * inv_samples;
        accel_mean[axis] = accel_sum[axis]  * inv_samples;
        accel_norm_sq   += accel_mean[axis] * accel_mean[axis];
    }

    if (accel_norm_sq > 0.0f) 
    {
        const float accel_norm = sqrtf(accel_norm_sq);
        const float gravity_scale = kGravity / accel_norm;

        for (uint8_t axis = 0; axis < 3; axis++) {
            const float expected_gravity = accel_mean[axis] * gravity_scale;
            config_.static_calibration.accel_offset[axis] = accel_mean[axis] - expected_gravity;
        }
    }

    return true;
}

/**
 * @brief 向加速度计写一个寄存器。
 */
bool Bmi088::WriteAccel(uint8_t addr, uint8_t value)
{
    tx_[0] = addr & reg::kWriteFlag;
    tx_[1] = value;
    return accel_.Send(tx_, 2);
}

/**
 * @brief 向陀螺仪写一个寄存器。
 */
bool Bmi088::WriteGyro(uint8_t addr, uint8_t value)
{
    tx_[0] = addr & reg::kWriteFlag;
    tx_[1] = value;
    return gyro_.Send(tx_, 2);
}

/**
 * @brief 从加速度计读取连续寄存器。
 */
bool Bmi088::ReadAccel(uint8_t addr, uint8_t *data, uint32_t len)
{
    /* SPI 读寄存器时，片选保持有效，后续 dummy byte 只用于继续送时钟。 */
    constexpr uint8_t kReadDummyByte = 0x55U;
    /* 每次读之前清空接收缓冲区，避免残留旧数据影响调试观察。 */
    constexpr uint8_t kRxClearByte = 0x00U;

    if (data == nullptr || len == 0 || len + 2 > sizeof(tx_)) {
        return false;
    }

    memset(tx_, kReadDummyByte, len + 2);
    memset(rx_, kRxClearByte, len + 2);
    tx_[0] = addr | reg::kReadFlag;

    if (!accel_.Transceive(tx_, rx_, len + 2)) {
        return false;
    }

    memcpy(data, &rx_[2], len);
    return true;
}

/**
 * @brief 从陀螺仪读取连续寄存器。
 */
bool Bmi088::ReadGyro(uint8_t addr, uint8_t *data, uint32_t len)
{
    /* SPI 读寄存器时，片选保持有效，后续 dummy byte 只用于继续送时钟。 */
    constexpr uint8_t kReadDummyByte = 0x55U;
    /* 每次读之前清空接收缓冲区，避免残留旧数据影响调试观察。 */
    constexpr uint8_t kRxClearByte = 0x00U;

    if (data == nullptr || len == 0 || len + 1 > sizeof(tx_)) {
        return false;
    }

    memset(tx_, kReadDummyByte, len + 1);
    memset(rx_, kRxClearByte,   len + 1);
    tx_[0] = addr | reg::kReadFlag;

    if (!gyro_.Transceive(tx_, rx_, len + 1)) {
        return false;
    }

    memcpy(data, &rx_[1], len);
    return true;
}

/**
 * @brief 读取一个加速度计寄存器。
 */
bool Bmi088::ReadAccelReg(uint8_t addr, uint8_t& value)
{
    return ReadAccel(addr, &value, 1);
}

/**
 * @brief 读取一个陀螺仪寄存器。
 */
bool Bmi088::ReadGyroReg(uint8_t addr, uint8_t& value)
{
    return ReadGyro(addr, &value, 1);
}

/**
 * @brief 写入加速度计寄存器并读回校验。
 */
bool Bmi088::WriteCheckedAccel(uint8_t addr, uint8_t value)
{
    uint8_t readback = 0;
    if (!WriteAccel(addr, value)) {
        return false;
    }
    k_msleep(1);
    return ReadAccelReg(addr, readback) && readback == value;
}

/**
 * @brief 写入陀螺仪寄存器并读回校验。
 */
bool Bmi088::WriteCheckedGyro(uint8_t addr, uint8_t value)
{
    uint8_t readback = 0;
    if (!WriteGyro(addr, value)) {
        return false;
    }
    k_msleep(1);
    return ReadGyroReg(addr, readback) && readback == value;
}

/**
 * @brief 将 BMI088 低高字节拼成有符号 16 位数据。
 */
int16_t Bmi088::ToInt16(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low));
}

/**
 * @brief 应用 offset/scale 校准参数。
 */
float Bmi088::Correct(float value, float offset, float scale)
{
    return (value - offset) * scale;
}

} // namespace bmi088
