/**
 * @file rls.hpp
 * @author qingyu
 * @brief 递归最小二乘(RLS)自适应滤波器 — 模板版本，编译期分配内存
 * @version 0.2
 * @date 2026-05-10
 *
 * @par 算法原理
 *      RLS 通过最小化指数加权误差平方和，递推更新权重向量:
 *      - E(n) = Y(n) - U(n)                        误差
 *      - K(n) = P(n-1)·X(n) / [λ + Xᵀ(n)·P(n-1)·X(n)]  卡尔曼增益
 *      - W(n) = W(n-1) + K(n)·E(n)                 权重更新
 *      - P(n) = [P(n-1) - K(n)·Xᵀ(n)·P(n-1)] / λ  协方差更新
 *
 * @tparam kXMax  输入向量最大维度（编译期分配）
 * @tparam kYMax  输出向量最大维度（编译期分配）
 *
 * @par 使用示例
 * @code
 *   // 2 输入 1 输出，遗忘因子 0.99，初始协方差 10.0
 *   alg::rls::RLS<2, 1> filter(0.99f, 10.0f);
 *
 *   float input[2] = {1.0f, 2.0f};
 *   float desired = 5.0f;
 *   filter.Update(input, &desired);
 * @endcode
 */

#pragma once

#include <stdint.h>
#include <string.h>

namespace alg::rls {

template <uint8_t kXMax = 6, uint8_t kYMax = 2>
class RLS final
{
public:
    /**
     * @brief 构造函数
     * @param lambda  遗忘因子 (0 < λ ≤ 1)，越小对过去遗忘越快
     * @param p_init  协方差矩阵 P 的初始对角值
     *
     * @note P 初始值建议:
     *       - SNR 较高 (信号可信): P_init = 1~10
     *       - SNR 较低 (噪声大):   P_init = 100~1000
     */
    RLS(float lambda, float p_init)
        : lambda_(lambda)
    {
        /* P 初始化为对角阵 P_init × I */
        for (uint16_t i = 0; i < kXMax * kXMax; i += (kXMax + 1)) {
            p_[i] = p_init;
        }
    }

    /// 禁止拷贝
    RLS(const RLS&) = delete;
    RLS& operator=(const RLS&) = delete;

    /**
     * @brief 执行一次 RLS 迭代
     * @param input    输入向量 X，长度 x_size (≤ kXMax)
     * @param desired  期望输出 Y，长度 y_size (≤ kYMax)
     */
    void Update(const float* input, const float* desired)
    {
        const uint8_t m = kXMax;
        const uint16_t n = static_cast<uint16_t>(kXMax) * kXMax;

        /* 拷贝输入 / 期望输出到内部缓冲区 */
        memcpy(x_, input,  kXMax * sizeof(float));
        memcpy(y_, desired, kYMax * sizeof(float));

        /* ---- 1. 模型输出 U = Wᵀ · X ---- */
        for (uint8_t i = 0; i < kYMax; i++) {
            u_[i] = 0.0f;
            for (uint8_t j = 0; j < kXMax; j++) {
                u_[i] += w_[j * kYMax + i] * x_[j];
            }
        }

        /* ---- 2. 输入转置 XT = Xᵀ ---- */
        Transpose(xt_, x_, m, 1);

        /* ---- 3. 误差 E = Y - U ---- */
        Subtract(e_, y_, u_, kYMax, 1);

        /* ---- 4. 增益分子 KN = P · X ---- */
        Multiply(kn_, p_, x_, m, m, 1);

        /* ---- 5. C1 = KN · XT = P·X·Xᵀ ---- */
        Multiply(cache1_, kn_, xt_, m, 1, m);

        /* ---- 6. C2 = C1 · P = P·X·Xᵀ·P ---- */
        Multiply(cache2_, cache1_, p_, m, m, m);

        /* ---- 7. C0 = XT · P ---- */
        Multiply(cache0_, xt_, p_, 1, m, m);

        /* ---- 8. C1 = C0 · X = Xᵀ·P·X (标量) ---- */
        Multiply(cache1_, cache0_, x_, 1, m, 1);

        /* ---- 9. C0 = λ + Xᵀ·P·X ---- */
        cache0_[0] = lambda_ + cache1_[0];

        /* ---- 10. 增益分母 KD = 1 / (λ + Xᵀ·P·X) ---- */
        kd_ = 1.0f / cache0_[0];

        /* ---- 11. K = KN · KD = P·X / (λ + Xᵀ·P·X) ---- */
        for (uint8_t i = 0; i < m; i++) {
            k_[i] = kn_[i] * kd_;
        }

        /* ---- 12. C0 = K · E ---- */
        Multiply(cache0_, k_, e_, m, 1, kYMax);

        /* ---- 13. W = W + K·E ---- */
        Add(output_, w_, cache0_, m, 1);

        /* 写回权重 */
        memcpy(w_, output_, m * sizeof(float));

        /* ---- 14. 协方差更新 ---- */
        for (uint16_t i = 0; i < n; i++) {
            cache0_[i] = cache2_[i] * kd_;
        }

        Subtract(cache1_, p_, cache0_, m, m);

        const float inv_lambda = 1.0f / lambda_;
        for (uint16_t i = 0; i < n; i++) {
            p_[i] = inv_lambda * cache1_[i];
        }
    }

    /** @brief 获取当前权重向量 W */
    const float* GetWeights() const { return w_; }

    /** @brief 设置初始权重 W */
    void SetWeights(const float* w)
    {
        memcpy(w_, w, kXMax * kYMax * sizeof(float));
    }

    /** @brief 获取模型估计输出 U */
    const float* GetOutput()   const { return u_; }

    /** @brief 获取当前误差 E = Y - U */
    const float* GetError()    const { return e_; }

    /** @brief 获取输入维度 */
    uint8_t GetXSize() const { return kXMax; }

    /** @brief 获取输出维度 */
    uint8_t GetYSize() const { return kYMax; }

private:
    /* ---------- 矩阵运算辅助 ---------- */

    /** C[m×n] = A[m×k] × B[k×n] (列优先) */
    void Multiply(float* c, const float* a, const float* b, uint8_t m, uint8_t k, uint8_t n) const
    {
        for (uint8_t col = 0; col < n; col++) {
            for (uint8_t row = 0; row < m; row++) {
                float sum = 0.0f;
                for (uint8_t i = 0; i < k; i++) {
                    sum += a[i * m + row] * b[col * k + i];
                }
                c[col * m + row] = sum;
            }
        }
    }

    /** C[m×n] = A[m×n] + B[m×n] */
    void Add(float* c, const float* a, const float* b, uint8_t rows, uint8_t cols) const
    {
        const uint16_t len = static_cast<uint16_t>(rows) * cols;
        for (uint16_t i = 0; i < len; i++) {
            c[i] = a[i] + b[i];
        }
    }

    /** C[m×n] = A[m×n] - B[m×n] */
    void Subtract(float* c, const float* a, const float* b, uint8_t rows, uint8_t cols) const
    {
        const uint16_t len = static_cast<uint16_t>(rows) * cols;
        for (uint16_t i = 0; i < len; i++) {
            c[i] = a[i] - b[i];
        }
    }

    /** B[n×m] = Aᵀ[m×n] */
    void Transpose(float* b, const float* a, uint8_t rows, uint8_t cols) const
    {
        for (uint8_t r = 0; r < rows; r++) {
            for (uint8_t c = 0; c < cols; c++) {
                b[r * cols + c] = a[c * rows + r];
            }
        }
    }

    /* ---------- 状态 ---------- */

    float lambda_;

    /* 所有矩阵以 flat float 数组存储，按列优先 (column-major) */

    float w_[kXMax * kYMax]{};       ///< 权重向量
    float x_[kXMax]{};               ///< 当前输入向量
    float xt_[kXMax]{};              ///< 输入转置
    float p_[kXMax * kXMax]{};       ///< 协方差矩阵
    float y_[kYMax]{};               ///< 期望输出
    float u_[kYMax]{};               ///< 模型估计输出
    float e_[kYMax]{};               ///< 误差
    float k_[kXMax]{};               ///< 卡尔曼增益
    float kn_[kXMax]{};              ///< 增益分子 P·X
    float kd_ = 0.0f;                ///< 增益分母标量 λ + Xᵀ·P·X
    float cache0_[kXMax * kXMax]{};  ///< 临时工作区
    float cache1_[kXMax * kXMax]{};  ///< 临时工作区
    float cache2_[kXMax * kXMax]{};  ///< 临时工作区
    float output_[kXMax]{};          ///< 权重更新暂存
};

}; // namespace alg::rls
