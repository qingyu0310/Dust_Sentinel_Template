/**
 * @file bmi088_reg.hpp
 * @author qingyu
 * @brief BMI088 register definitions used by the Zephyr source implementation
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include <cstdint>

namespace bmi088::reg {

static constexpr uint8_t kReadFlag              = 0x80;
static constexpr uint8_t kWriteFlag             = 0x7F;

static constexpr uint8_t kAccChipId             = 0x00;
static constexpr uint8_t kAccChipIdValue        = 0x1E;
static constexpr uint8_t kAccStatus             = 0x03;
static constexpr uint8_t kAccDataReady          = 0x80;
static constexpr uint8_t kAccXoutL              = 0x12;
static constexpr uint8_t kAccTempM              = 0x22;
static constexpr uint8_t kAccConf               = 0x40;
static constexpr uint8_t kAccRange              = 0x41;
static constexpr uint8_t kAccInt1IoCtrl         = 0x53;
static constexpr uint8_t kAccIntMapData         = 0x58;
static constexpr uint8_t kAccPwrConf            = 0x7C;
static constexpr uint8_t kAccPwrCtrl            = 0x7D;
static constexpr uint8_t kAccSoftReset          = 0x7E;

static constexpr uint8_t kAccNormal             = 0x20;
static constexpr uint8_t kAcc800Hz              = 0x0B;
static constexpr uint8_t kAccConfMustSet        = 0x80;
static constexpr uint8_t kAccRange6g            = 0x01;
static constexpr uint8_t kAccInt1Enable         = 0x08;
static constexpr uint8_t kAccInt1PushPull       = 0x00;
static constexpr uint8_t kAccInt1Low            = 0x00;
static constexpr uint8_t kAccInt1Drdy           = 0x04;
static constexpr uint8_t kAccPowerActive        = 0x00;
static constexpr uint8_t kAccPowerOn            = 0x04;
static constexpr uint8_t kAccResetValue         = 0xB6;

static constexpr uint8_t kGyroChipId            = 0x00;
static constexpr uint8_t kGyroChipIdValue       = 0x0F;
static constexpr uint8_t kGyroXoutL             = 0x02;
static constexpr uint8_t kGyroIntStat1          = 0x0A;
static constexpr uint8_t kGyroDataReady         = 0x80;
static constexpr uint8_t kGyroRange             = 0x0F;
static constexpr uint8_t kGyroBandwidth         = 0x10;
static constexpr uint8_t kGyroLpm1              = 0x11;
static constexpr uint8_t kGyroSoftReset         = 0x14;
static constexpr uint8_t kGyroCtrl              = 0x15;
static constexpr uint8_t kGyroIoConf            = 0x16;
static constexpr uint8_t kGyroIoMap             = 0x18;

static constexpr uint8_t kGyro2000Dps           = 0x00;
static constexpr uint8_t kGyro2000Hz230Hz       = 0x81;
static constexpr uint8_t kGyroNormalMode        = 0x00;
static constexpr uint8_t kGyroResetValue        = 0xB6;
static constexpr uint8_t kGyroDrdyOn            = 0x80;
static constexpr uint8_t kGyroInt3PushPullLow   = 0x00;
static constexpr uint8_t kGyroDrdyInt3          = 0x01;

/*
 * 静态校准参数默认入口。
 * 如果关闭自动校准，就直接使用这里的 offset/scale。
 * 单位：
 * - gyro_offset: rad/s
 * - accel_offset: m/s^2
 * - gyro_scale/accel_scale: 无量纲比例
 */
static constexpr float kStaticGyroOffset[3]  {
    0.00508663664f,   0.000101200138f,   0.00114515924f
};
static constexpr float kStaticGyroScale[3]   {
    1.0f, 1.0f, 1.0f
};
static constexpr float kStaticAccelOffset[3] {
    -0.000633206335f, 0.000943555031f,   -0.128417715f
};
static constexpr float kStaticAccelScale[3]  {
    1.0f, 1.0f, 1.0f
};

} // namespace bmi088::reg
