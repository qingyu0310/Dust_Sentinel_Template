/**
 * @file hpf.hpp
 * @author qingyu
 * @brief 一阶高通滤波器（基于截止频率）
 * @version 0.1
 * @date 2026-06-01
 *
 * @par 使用示例
 * @code
 *   alg::filter::HighPassFilter hpf(100.0f, 0.001f);  // 100 Hz, 1 ms
 *   float out = hpf.Update(raw);
 * @endcode
 */

#pragma once

namespace alg::filter {

/**
 * @brief 一阶 RC 高通滤波器
 *
 *        高通输出 = 输入 - 低通输出
 *
 *        lpf_out[n] = alpha * in[n] + (1-alpha) * lpf_out[n-1]
 *        hpf_out[n] = in[n] - lpf_out[n]
 *
 *        alpha = dt / (dt + RC),  RC = 1 / (2π · fc)
 */
class HighPassFilter final
{
public:
    HighPassFilter() = default;

    /**
     * @brief 构造并初始化
     * @param cutoffHz  截止频率（Hz），≤ 0 表示直通
     * @param dt        采样周期（秒）
     */
    HighPassFilter(float cutoffHz, float dt) { Init(cutoffHz, dt); }

    /**
     * @brief（重新）配置滤波器
     * @param cutoffHz  截止频率（Hz），≤ 0 表示直通
     * @param dt        采样周期（秒）
     */
    void Init(float cutoffHz, float dt)
    {
        if (cutoffHz <= 0.0f) {
            alpha_ = 1.0f;      // 直通模式（输出恒为 0）
        } else {
            float rc = 1.0f / (k2Pi * cutoffHz);
            alpha_ = dt / (dt + rc);
        }
        lpf_out_ = 0.0f;
        initialized_ = true;
    }

    /**
     * @brief 用新采样值更新滤波器
     * @param input  原始输入值
     * @return       高通滤波后的输出值
     */
    float Update(float input)
    {
        if (!initialized_) return 0.0f;
        lpf_out_ = alpha_ * input + (1.0f - alpha_) * lpf_out_;
        output_ = input - lpf_out_;
        return output_;
    }

    float GetOutput() const { return output_; }
    void  Reset(float val = 0.0f) { lpf_out_ = val; output_ = 0.0f; }

private:
    static constexpr float k2Pi = 6.283185307179586f;

    float alpha_       = 0.0f;
    float lpf_out_     = 0.0f;
    float output_      = 0.0f;
    bool  initialized_ = false;
};

} // namespace alg::filter
