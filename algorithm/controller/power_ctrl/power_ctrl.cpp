/**
 * @file power_ctrl.cpp
 * @author qingyu
 * @brief 电机功率控制器实现 — 模型预测 / RLS 辨识 / 隶属度分配 / 力矩限幅
 * @version 0.1
 * @date 2026-04-30
 */

#include "power_ctrl.hpp"
#include <math.h>

namespace alg::power_ctrl {

/**
 * @brief 构造功率控制器，初始化 RLS
 * @param cfg 配置参数
 */
PowerCtrl::PowerCtrl(const Config& cfg)
    : cfg_(cfg)
    , k1_(cfg.k1Init)
    , k2_(cfg.k2Init)
    , rls_(cfg.rlsLambda, 1e-5f)
{
    rlsInited_ = cfg.rlsEnable;
    if (rlsInited_) {
        float w[2] = { k1_, k2_ };
        rls_.SetWeights(w);
    }
}

/**
 * @brief 重配置功率控制器，清空状态
 * @param cfg 配置参数
 */
void PowerCtrl::Init(const Config& cfg)
{
    cfg_ = cfg;
    k1_ = cfg.k1Init;
    k2_ = cfg.k2Init;
    rlsInited_ = cfg.rlsEnable;
    if (rlsInited_) {
        float w[2] = { k1_, k2_ };
        rls_.SetWeights(w);
    }
    for (auto& m : motors_) m = MotorState{};
    for (auto& m : membership_) m = 0.0f;
    measuredPower_ = 0.0f;
}

/**
 * @brief 喂入单电机原始数据
 * @param idx     电机索引 [0, motorCount)
 * @param torque  实测转矩 (N·m)
 * @param omega   角速度 (rad/s)
 * @param pidErr  PID 误差（隶属度计算用）
 */
void PowerCtrl::SetMotorData(uint8_t idx, float torque, float omega, float pidErr)
{
    if (idx >= cfg_.motorCount) return;

    auto& m = motors_[idx];
    m.torque   = torque;
    m.omega    = omega;
    m.torque2  = m.torque * m.torque;
    m.omega2   = m.omega  * m.omega;
    m.pidErr   = pidErr;
}

/**
 * @brief 设置 PID 目标电流（串级输入）
 * @param idx      电机索引
 * @param current  目标电流
 */
void PowerCtrl::SetTarget(uint8_t idx, float current)
{
    if (idx >= cfg_.motorCount) return;
    motors_[idx].targetCurrent = current;
}

/**
 * @brief 功率预测 + RLS 在线更新
 *
 * 对每台电机计算 P = K1·τ² + K2·ω² + τ·ω(可选) + K3，
 * 再以 Στ²、Σω² 为输入，P_meas - K3 为期望输出，
 * 更新 K1（铜损系数）和 K2（铁损系数）。
 */
void PowerCtrl::Predict()
{
    float sumTorque2 = 0.0f;
    float sumOmega2  = 0.0f;

    for (uint8_t i = 0; i < cfg_.motorCount; i++) 
    {
        auto& m = motors_[i];

        // P = K1·τ² + K2·ω² + τ·ω(可选) + K3
        m.powerPred = k1_ * m.torque2 + k2_ * m.omega2 + cfg_.k3;
        if (cfg_.tauOmegaEnable) {
            m.powerPred += m.torque * m.omega;
        }

        sumTorque2 += m.torque2;
        sumOmega2  += m.omega2;
    }

    // RLS 在线更新 — 减掉已知的 K3，RLS 只管 K1·τ² + K2·ω²
    if (rlsInited_) 
    {
        float x[2] = { sumTorque2, sumOmega2 };
        float y[1] = { measuredPower_ - cfg_.k3 };
        rls_.Update(x, y);
        k1_ = rls_.GetWeights()[0];
        k2_ = rls_.GetWeights()[1];
    }
}

/**
 * @brief 功率分配：隶属度计算 + 力矩限幅
 *
 * 总预测未超预算时直通 PID 目标电流；
 * 超限时逐电机解二次方程求受限转矩。
 * @param totalBudget 该组总功率预算 (W)
 */
void PowerCtrl::Allocate(float totalBudget)
{
    if (cfg_.motorCount == 0) return;

    float sumAbsErr   = 0.0f;
    float sumPowerAbs = 0.0f;

    for (uint8_t i = 0; i < cfg_.motorCount; i++) {
        sumAbsErr   += fabsf(motors_[i].pidErr);
        sumPowerAbs += fabsf(motors_[i].powerPred);
    }

    // 权重 K（误差水平插值）
    float k;
    if (sumAbsErr >= cfg_.errUpper) {
        k = 1.0f;                                   // 全按需求分配
    } else if (sumAbsErr <= cfg_.errLower) {
        k = 0.0f;                                   // 全按功率分配
    } else {
        float range = cfg_.errUpper - cfg_.errLower;
        k = (sumAbsErr - cfg_.errLower) / (range > 0.0f ? range : 1.0f);
    }

    // 隶属度 + 功率上限
    for (uint8_t i = 0; i < cfg_.motorCount; i++) {
        float ratioErr   = (sumAbsErr   > 0.0f) ? fabsf(motors_[i].pidErr)     / sumAbsErr   : 0.0f;
        float ratioPower = (sumPowerAbs > 0.0f) ? fabsf(motors_[i].powerPred)  / sumPowerAbs : 0.0f;

        membership_[i] = k * ratioErr + (1.0f - k) * ratioPower;

        if (membership_[i] < 0.0f) membership_[i] = 0.0f;
        if (membership_[i] > 1.0f) membership_[i] = 1.0f;

        motors_[i].powerLimit = membership_[i] * totalBudget;
    }

    // 受限力矩求解
    float totalPred = GetTotalPower();

    if (totalPred <= totalBudget) {
        // 未超限：透传 PID 目标电流
        for (uint8_t i = 0; i < cfg_.motorCount; i++) {
            motors_[i].currentOut = motors_[i].targetCurrent;
        }
        return;
    }

    // 超限：逐电机解方程，限制 PID 目标
    for (uint8_t i = 0; i < cfg_.motorCount; i++) 
    {
        const auto& m = motors_[i];

        float a = k1_;                                          // K1
        float b = m.omega;                                      // ω
        float c = k2_ * m.omega2 + cfg_.k3 - m.powerLimit;      // K2·ω² + K3 - P_limit

        // τ = (-B ± √(B² - 4AC)) / (2A)
        bool positive = (m.targetCurrent >= 0.0f);
        float limitedTorque = SolveTorque(a, b, c, positive);

        float limitedCurrent = limitedTorque / cfg_.torqueK;
        // 取 PID 目标和限制值中绝对值较小的那个
        motors_[i].currentOut = (fabsf(m.targetCurrent) <= fabsf(limitedCurrent)) ? m.targetCurrent : limitedCurrent;
    }
}

/**
 * @brief 取限幅后的输出电流
 * @param idx  电机索引
 * @return 限幅后的电流值，越界返回 0
 */
float PowerCtrl::GetLimitedCurrent(uint8_t idx) const
{
    return (idx < cfg_.motorCount) ? motors_[idx].currentOut : 0.0f;
}

/**
 * @brief 所有电机总预测功率
 * @return 总功率 (W)
 */
float PowerCtrl::GetTotalPower() const
{
    float sum = 0.0f;
    for (uint8_t i = 0; i < cfg_.motorCount; i++) {
        sum += motors_[i].powerPred;
    }
    return sum;
}

/**
 * @brief 解二次方程 A·τ² + B·τ + C = 0，取与目标同号的根
 * @param a  K1
 * @param b  ω
 * @param c  K2·ω² + K3 - P_limit
 * @param positive  目标转矩方向
 * @return 受限转矩
 */
float PowerCtrl::SolveTorque(float a, float b, float c, bool positive)
{
    if (a == 0.0f) {
        // 退化成一元一次（不应发生，但防御）
        return (b != 0.0f) ? -c / b : 0.0f;
    }

    float delta = b * b - 4.0f * a * c;
    if (delta < 0.0f) delta = 0.0f;

    float sqrtDelta = sqrtf(delta);
    float t1 = (-b + sqrtDelta) / (2.0f * a);
    float t2 = (-b - sqrtDelta) / (2.0f * a);

    return positive ? t1 : t2;
}

} // namespace alg::power_ctrl
