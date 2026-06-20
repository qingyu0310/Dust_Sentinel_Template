/**
 * @file power_ctrl.hpp
 * @author qingyu
 * @brief 电机功率控制器 — 模型预测 / RLS 在线辨识 / 隶属度分配 / 力矩限幅
 * @version 0.1
 * @date 2026-04-30
 */

#pragma once

#include <stdint.h>
#include "rls.hpp"

namespace alg::power_ctrl {

inline constexpr uint8_t kMaxMotors = 4;

/**
 * @brief  单电机状态
 */
struct MotorState {
    float torque        = 0.0f;         // τ = current × torqueK
    float omega         = 0.0f;         // ω = rpm / 9.55 (rad/s)
    float torque2       = 0.0f;         // τ²
    float omega2        = 0.0f;         // ω²
    float powerPred     = 0.0f;         // P_in = K1·τ² + K2·ω² + τ·ω + K3
    float pidErr        = 0.0f;         // PID 误差（隶属度计算用）
    float powerLimit    = 0.0f;         // 该电机分配的功率预算
    float targetCurrent = 0.0f;         // PID 目标电流（串级输入）
    float currentOut    = 0.0f;         // 限幅后的输出电流
};

/**
 * @brief  单组电机功率控制器（如同一底盘 4 个电机）
 *
 * @par 管线（每控制周期调用一次）：
 *       @n 1. SetMotorData(i, current, velocity, pidErr)   — 喂入各电机数据
 *       @n 2. Predict()                                    — 功率预测 + RLS
 *       @n 3. Allocate(totalBudget)                        — 隶属度分配 + 限幅
 *       @n 4. GetLimitedCurrent(i)                         — 读取结果
 *
 * @par RLS：
 *       在线辨识 K1（铜损系数）和 K2（铁损系数）。
 *       K3 为固定常数损耗，不参与辨识。
 */
class PowerCtrl final
{
public:
    struct Config {
        uint8_t motorCount = 4;                 // 该组电机数量（≤ kMaxMotors）
        float   torqueK    = 4.577e-5f;         // 电流 → 转矩（M3508 常数）
        float   k1Init     = 1.453e-7f;         // RLS 初始 K1（铜损系数）
        float   k2Init     = 1.453e-7f;         // RLS 初始 K2（铁损系数）
        float   k3         = 3.0f;              // 固定常数损耗
        float   errUpper   = 50.0f;             // 隶属度：全需求阈值
        float   errLower   = 0.01f;             // 隶属度：全功率阈值
        float   rlsLambda  = 0.99999f;          // RLS 遗忘因子
        bool    rlsEnable  = false;             // false = 使用固定 K1/K2
        bool    tauOmegaEnable = true;          // 是否包含 τ·ω 项
    };

    PowerCtrl() : PowerCtrl(Config{}) {}
    explicit PowerCtrl(const Config& cfg);
    void Init(const Config& cfg);

    void SetMotorData(uint8_t idx, float torque, float omega, float pidErr);
    void SetTarget(uint8_t idx, float current);
    void SetMeasuredPower(float power) { measuredPower_ = power; }

    void Predict();
    void Allocate(float totalBudget);

    float GetLimitedCurrent(uint8_t idx) const;
    float GetTotalPower() const;
    float GetK1() const { return k1_; }
    float GetK2() const { return k2_; }

private:
    Config cfg_;

    float k1_ = 0.0f;     // 铜损系数
    float k2_ = 0.0f;     // 铁损系数

    MotorState motors_[kMaxMotors]{};
    float      membership_[kMaxMotors]{};

    alg::rls::RLS<2, 1> rls_;   // 2 输入 / 1 输出在线辨识器
    bool rlsInited_ = false;

    float measuredPower_ = 0.0f;  // 功率计实测总功率

    // 二次方程求根：A·τ² + B·τ + C = 0
    static float SolveTorque(float a, float b, float c, bool positive);
};

} // namespace alg::power_ctrl
