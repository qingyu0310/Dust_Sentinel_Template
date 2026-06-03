/**
 * @file icm42688p_reg.hpp
 * @author qingyu
 * @brief ICM42688P 寄存器定义
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include <cstdint>

namespace icm42688p::reg {

// 通用寄存器区
constexpr uint8_t kReadFlag             = 0x80;
constexpr uint8_t kSegSel               = 0x7F;
constexpr uint8_t kSegSelGeneral        = 0x00;
constexpr uint8_t kSegSelSpecial1       = 0x83;
constexpr uint8_t kSegSelSpecial2       = 0x8C;
constexpr uint8_t kSegSelSpecial3       = 0x90;

constexpr uint8_t kWhoAmI               = 0x01;
constexpr uint8_t kWhoAmIValue          = 0x6A;

constexpr uint8_t kOisConf              = 0x04;
constexpr uint8_t kComCfg               = 0x05;
constexpr uint8_t kComCfgDefault        = 0x50;
constexpr uint8_t kIntCfg1              = 0x06;
constexpr uint8_t kIntCfg2              = 0x07;
constexpr uint8_t kHpfLpfCfg            = 0x08;
constexpr uint8_t kHpfLpfCfgDefault     = 0x80;
constexpr uint8_t kDataStat             = 0x0B;

constexpr uint8_t kAccXH                = 0x0C;
constexpr uint8_t kAccXL                = 0x0D;
constexpr uint8_t kAccYH                = 0x0E;
constexpr uint8_t kAccYL                = 0x0F;
constexpr uint8_t kAccZH                = 0x10;
constexpr uint8_t kAccZL                = 0x11;
constexpr uint8_t kGyrXH                = 0x12;
constexpr uint8_t kGyrXL                = 0x13;
constexpr uint8_t kGyrYH                = 0x14;
constexpr uint8_t kGyrYL                = 0x15;
constexpr uint8_t kGyrZH                = 0x16;
constexpr uint8_t kGyrZL                = 0x17;
constexpr uint8_t kTimeH                = 0x18;
constexpr uint8_t kTimeM                = 0x19;
constexpr uint8_t kTimeL                = 0x1A;

constexpr uint8_t kFifoCfg0             = 0x1C;
constexpr uint8_t kFifoCfg1             = 0x1D;
constexpr uint8_t kFifoCfg1Default      = 0x07;
constexpr uint8_t kFifoCfg2             = 0x1E;
constexpr uint8_t kFifoCfg2Default      = 0xFF;
constexpr uint8_t kFifoStat0            = 0x1F;
constexpr uint8_t kFifoStat0Default     = 0x40;
constexpr uint8_t kFifoStat1            = 0x20;
constexpr uint8_t kFifoData             = 0x21;
constexpr uint8_t kTempH                = 0x22;
constexpr uint8_t kTempL                = 0x23;

constexpr uint8_t kAoi1Cfg              = 0x30;
constexpr uint8_t kAoi1Stat             = 0x31;
constexpr uint8_t kAoi1Ths              = 0x32;
constexpr uint8_t kAoi1Duration         = 0x33;
constexpr uint8_t kAoi2Cfg              = 0x34;
constexpr uint8_t kAoi2Stat             = 0x35;
constexpr uint8_t kAoi2Ths              = 0x36;
constexpr uint8_t kAoi2Duration         = 0x37;
constexpr uint8_t kClickCtrlReg         = 0x38;
constexpr uint8_t kClickSrc             = 0x39;
constexpr uint8_t kStepCfg              = 0x3A;
constexpr uint8_t kStepCfgDefault       = 0x08;
constexpr uint8_t kStepSrc              = 0x3B;
constexpr uint8_t kStepCounterL         = 0x3C;
constexpr uint8_t kStepCounterH         = 0x3D;
constexpr uint8_t kAoi12Cfg             = 0x3F;

constexpr uint8_t kAccConf              = 0x40;
constexpr uint8_t kAccConfDefault       = 0xA8;
constexpr uint8_t kAccRange             = 0x41;
constexpr uint8_t kAccRangeDefault      = 0x02;
constexpr uint8_t kGyrConf              = 0x42;
constexpr uint8_t kGyrConfDefault       = 0xA9;
constexpr uint8_t kGyrRange             = 0x43;
constexpr uint8_t kFifoDowns            = 0x45;
constexpr uint8_t kFifoDownsDefault     = 0x88;
constexpr uint8_t kSoftRstReg           = 0x4A;
constexpr uint8_t kAccSelfTest          = 0x6D;
constexpr uint8_t kGyrSelfTest          = 0x6F;
constexpr uint8_t kPwrCtrl              = 0x7D;

// Special Register 1
// 访问前先向 SEG_SEL 写入 0x83，完成后写回 0x00。
constexpr uint8_t kI2cUn                = 0x6F;

// Special Register 2
// 访问前先向 SEG_SEL 写入 0x8C，完成后写回 0x00。
constexpr uint8_t kDigCtrl              = 0x30;

// Special Register 3
// 访问前先向 SEG_SEL 写入 0x90，完成后写回 0x00。
constexpr uint8_t kWristSrc             = 0x3E;
constexpr uint8_t kClickCoeff1          = 0x40;
constexpr uint8_t kClickCoeff1Default   = 0x52;
constexpr uint8_t kClickCoeff2          = 0x41;
constexpr uint8_t kClickCoeff2Default   = 0x9A;
constexpr uint8_t kClickCoeff3          = 0x42;
constexpr uint8_t kClickCoeff3Default   = 0x04;
constexpr uint8_t kClickCoeff4          = 0x43;
constexpr uint8_t kClickCoeff4Default   = 0x57;
constexpr uint8_t kStepDelta            = 0x44;
constexpr uint8_t kStepDeltaDefault     = 0x01;
constexpr uint8_t kStepWtm              = 0x45;
constexpr uint8_t kStepWtmDefault       = 0x01;
constexpr uint8_t kPedoCoeff1           = 0x46;
constexpr uint8_t kPedoCoeff1Default    = 0x4F;
constexpr uint8_t kPedoCoeff2           = 0x47;
constexpr uint8_t kPedoCoeff2Default    = 0x23;
constexpr uint8_t kPedoCoeff3           = 0x48;
constexpr uint8_t kPedoCoeff3Default    = 0xA5;
constexpr uint8_t kPedoCoeff4           = 0x49;
constexpr uint8_t kPedoCoeff4Default    = 0x23;
constexpr uint8_t kPedoCoeff5           = 0x4A;
constexpr uint8_t kPedoCoeff5Default    = 0x04;
constexpr uint8_t kPedoCoeff6           = 0x4B;
constexpr uint8_t kPedoCoeff6Default    = 0x8C;
constexpr uint8_t kWristCtrl1           = 0x51;
constexpr uint8_t kWristCtrl1Default    = 0x30;
constexpr uint8_t kWristCtrl2           = 0x52;
constexpr uint8_t kWristCtrl2Default    = 0x0F;
constexpr uint8_t kWristCtrl3           = 0x53;
constexpr uint8_t kWristCtrl3Default    = 0x93;

} // namespace icm42688p::reg
