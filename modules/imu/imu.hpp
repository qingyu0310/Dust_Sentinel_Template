/**
 * @file imu.hpp
 * @author qingyu
 * @brief IMU 模块公共接口与采样数据结构
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include <cstdint>
#include "imu_to.hpp"
#include "quaternion_ekf.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "thread.hpp"
#include "timer.hpp"

/**
 * @brief 一帧 IMU 工程量样本
 *
 * 具体 IMU 驱动负责完成寄存器读取、单位换算与驱动侧校准，
 * 上层姿态线程统一消费这个结构。
 */
struct Sample
{
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float temperature = 0.0f;
    float dt = 0.001f;
};

/**
 * @brief 具体 IMU 驱动需要实现的统一接口
 */
class Source
{
public:
    virtual ~Source() = default;

    /**
     * @brief 初始化具体传感器与相关总线资源
     */
    virtual bool Init() = 0;

    /**
     * @brief 读取一帧工程量样本
     */
    virtual bool Read(Sample& sample) = 0;

    /**
     * @brief 按当前驱动配置执行一次静态校准
     */
    virtual bool Calibrate() { return true; }

    /**
     * @brief 返回建议的线程轮询周期，单位 ms
     */
    virtual uint32_t PeriodMs() const { return 1; }
};

namespace attitude {

class Processor final
{
public:
    void Init();
    void Process(const Sample& sample, topic::imu_to::Message& pub);

private:
    alg::attitude::QuaternionEkf ekf_ {};
};

} // namespace attitude

namespace heater {

class Heater final
{
public:
    bool Init();
    void Update(float temperature);
    bool IsStable(float temperature, float tolerance = 0.5f) const;
    float GetDuty() const { return duty_; }

private:
    static constexpr float kMaxDuty = 0.95f;
    static constexpr float target_temperature_ = 40.0f;

    static constexpr alg::pid::Pid::Config kDefaultPidConfig {
        .kp         = 0.13f,
        .ki         = 0.03f,
        .kd         = 0.0f,
        .iOutMax    = 0.5f,
        .outMax     = kMaxDuty,
        .dt         = 0.001f,
    };

    Pwm heater_pwm_ {};
    alg::pid::Pid pid_ {};
    float duty_ = 0.0f;
    bool  initialized_ = false;
};

} // namespace heater

namespace imu {

class ImuManager final
{
public:
    void Init(bool enable_auto_calibration = false);
    void Start(uint8_t prio);
    bool IsReady() const { return ready_; }

private:
    bool SelectSource();
    bool PrepareCalibration();
    bool Process(const Sample& sample);
    void Task();

    static void TaskEntry(void *p1, void *p2, void *p3)
    {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto *self = static_cast<ImuManager*>(p1);
        self->Task();
    }

    Thread<4096>                    thread_     {};
    Source                         *source_     = nullptr;
    Sample                          sample_     {};
    attitude::Processor             attitude_   {};
    Timer                           timer_      {100};
    heater::Heater                  heater_     {};
    topic::imu_to::Message          pub_        {};
    uint32_t                        last_sample_cycle_ = 0;
    bool                            ready_      = false;
};

} // namespace imu
