/**
 * @file bmi088.hpp
 * @author qingyu
 * @brief BMI088 IMU Source 接口与配置定义。
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include "imu.hpp"
#include "spi.hpp"

#include <cstdint>
#include <zephyr/drivers/spi.h>

namespace bmi088 {

enum class Error : uint8_t
{
    None = 0,
    AccelNotReady,
    GyroNotReady,
    AccelChipId,
    GyroChipId,
    AccelConfig,
    GyroConfig,
    ReadFailed,
};

class Bmi088 final : public Source
{
public:
    /**
     * @brief BMI088 原始寄存器样本。
     *
     * 这里保留传感器原始计数值，供校准和工程量换算共用。
     */
    struct RawSample {
        int16_t gyro[3]  = {0, 0, 0};
        int16_t accel[3] = {0, 0, 0};
        int16_t temperature = 0;
    };

    /**
     * @brief 传感器静态校准参数。
     */
    struct Calibration {
        float gyro_offset[3]  = {0.0f, 0.0f, 0.0f};
        float gyro_scale[3]   = {1.0f, 1.0f, 1.0f};
        float accel_offset[3] = {0.0f, 0.0f, 0.0f};
        float accel_scale[3]  = {1.0f, 1.0f, 1.0f};
    };

    /**
     * @brief BMI088 Source 运行配置。
     */
    struct Config {
        const struct spi_dt_spec *accel = nullptr;
        const struct spi_dt_spec *gyro  = nullptr;
        Calibration static_calibration {};
        uint32_t period_ms = 1;
        bool auto_calibration = true;
        uint16_t calibration_samples = 200;
        uint32_t calibration_settle_ms = 50;
    };

    Bmi088() = default;
    explicit Bmi088(const Config& config);

    /**
     * @brief 更新当前 BMI088 配置。
     */
    void Configure(const Config& config);

    /**
     * @brief 初始化 BMI088 加速度计、陀螺仪和可选自动校准。
     */
    bool Init() override;

    /**
     * @brief 读取并输出一帧工程单位样本。
     */
    bool Read(Sample& sample) override;

    /**
     * @brief 读取一帧 BMI088 原始寄存器样本。
     */
    bool ReadRaw(RawSample& raw);
    uint32_t PeriodMs() const override;
    Error LastError() const;

private:
    static constexpr float kAccel6gSensitivity  = 0.00179443359375f;
    static constexpr float kGyro2000Sensitivity = 0.0010652644360316953f;
    static constexpr float kTemperatureFactor   = 0.125f;
    static constexpr float kTemperatureOffset   = 23.0f;
    static constexpr float kGravity = 9.8f;

    bool InitAccel();
    bool InitGyro();
    bool AutoCalibrate();
    bool WriteAccel(uint8_t addr, uint8_t value);
    bool WriteGyro(uint8_t addr, uint8_t value);
    bool ReadAccel(uint8_t addr, uint8_t *data, uint32_t len);
    bool ReadGyro(uint8_t addr, uint8_t *data, uint32_t len);
    bool ReadAccelReg(uint8_t addr, uint8_t& value);
    bool ReadGyroReg(uint8_t addr, uint8_t& value);
    bool WriteCheckedAccel(uint8_t addr, uint8_t value);
    bool WriteCheckedGyro(uint8_t addr, uint8_t value);

    static int16_t ToInt16(uint8_t low, uint8_t high);
    static float Correct(float value, float offset, float scale);

    Config config_ {};
    Spi accel_ {};
    Spi gyro_  {};
    Error last_error_ = Error::None;
    uint8_t tx_[8] {};
    uint8_t rx_[8] {};
};

Bmi088& Instance();

/**
 * @brief 使用给定配置注册 BMI088 Source。
 */
bool Register(const Bmi088::Config& config);

/**
 * @brief 根据设备树 alias 配置 BMI088 Source。
 */
bool ConfigureFromDevicetree(uint32_t period_ms = 1, bool auto_calibration = true);

/**
 * @brief 根据设备树 alias 配置并注册 BMI088 Source。
 */
bool RegisterFromDevicetree(uint32_t period_ms = 1, bool auto_calibration = true);

} // namespace bmi088
