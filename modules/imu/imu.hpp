/**
 * @file imu.hpp
 * @author qingyu
 * @brief IMU 模块公共接口与通用采样结构。
 * @version 0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>

/**
 * @brief IMU 原始样本。
 *
 * 陀螺、加速度和温度由具体传感器 Source 填充，
 * `dt` 表示本次样本对应的真实采样间隔。
 */
struct Sample
{
    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float temperature = 0.0f;
    float dt = 0.001f;
};

/**
 * @brief IMU 数据源抽象接口。
 *
 * 不同芯片只需要实现初始化、读样本和建议周期，
 * 上层姿态线程不直接依赖具体传感器类型。
 */
class Source
{
public:
    virtual ~Source() = default;
    virtual bool Init() = 0;
    virtual bool Read(Sample& sample) = 0;
    virtual uint32_t PeriodMs() const { return 1; }
};

namespace thread::imu {

/**
 * @brief 注册当前 IMU 线程使用的数据源。
 *
 * @param source 已完成板级配置的 IMU Source。
 * @return 注册成功返回 true。
 */
bool RegisterSource(Source& source);

/**
 * @brief 初始化 IMU 模块。
 *
 * 当前默认选择 BMI088 作为数据源，并完成姿态滤波器初始化。
 */
void thread_init();

/**
 * @brief 启动 IMU 线程。
 *
 * @param prio Zephyr 线程优先级。
 */
void thread_start(uint8_t prio = 5);

} // namespace thread::imu
