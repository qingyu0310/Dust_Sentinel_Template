/**
 * @file imu.cpp
 * @author qingyu
 * @brief IMU 线程：采样、姿态解算与统一消息发布。
 * @version 0.1
 * @date 2026-06-01
 */

#include "imu.hpp"
#include "imu_to.hpp"
#include "quaternion_ekf.hpp"
#include "thread.hpp"
#include "zephyr/sys/printk.h"

#ifdef CONFIG_MOD_DEV_IMU_BMI088
#include "bmi088.hpp"
#endif

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

namespace {

/**
 * @brief 用真实经过时间修正本次样本的积分时间。
 *
 * `sample.dt` 的默认值来自传感器配置周期，但线程调度、SPI 访问和
 * 调试输出都可能让实际采样间隔变长，因此这里优先使用 cycle counter
 * 计算相邻样本的真实 dt。
 */
float MeasuredDtSeconds(uint32_t now_cycle, uint32_t last_cycle, float fallback_dt)
{
    if (last_cycle == 0U) {
        return fallback_dt;
    }

    const uint32_t delta_cycles = now_cycle - last_cycle;
    const uint64_t delta_ns = k_cyc_to_ns_floor64(delta_cycles);
    if (delta_ns == 0U) {
        return fallback_dt;
    }

    return static_cast<float>(delta_ns) * 1.0e-9f;
}

} // namespace

namespace thread::imu {

enum class GyroType : uint8_t
{
    BMI088 = 0,
    None,
};

class ImuWorker final
{
public:
    void Init(GyroType type, bool enable_auto_calibration);
    void Start(uint8_t prio);
    bool Process(const Sample& sample);

private:
    bool SelectSource(GyroType type, bool enable_auto_calibration);
    void Task();

    static void TaskEntry(void *p1, void *p2, void *p3)
    {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto *self = static_cast<ImuWorker*>(p1);
        self->Task();
    }

    Thread<4096>                  thread_ {};
    Source                       *source_ = nullptr;
    Sample                        sample_ {};
    alg::attitude::QuaternionEkf  ekf_ {};
    topic::imu_to::Message        pub_ {};
    uint32_t                      last_sample_cycle_ = 0U;
    bool                          ready_ = false;
};

void ImuWorker::Init(GyroType type, bool enable_auto_calibration)
{
    ready_ = false;
    last_sample_cycle_ = 0U;

    if (source_ == nullptr && !SelectSource(type, enable_auto_calibration)) {
        return;
    }

    if (source_ == nullptr) {
        return;
    }

    if (!source_->Init()) {
        return;
    }

    alg::attitude::QuaternionEkf::Config cfg {};
    ekf_.Init(cfg);
    ready_ = true;
}

void ImuWorker::Start(uint8_t prio)
{
    if (!ready_) {
        return;
    }

    thread_.Start(TaskEntry, prio, this);
}

bool ImuWorker::Process(const Sample& sample)
{
    ekf_.Update(sample.gyro[0],  sample.gyro[1],  sample.gyro[2],
                sample.accel[0], sample.accel[1], sample.accel[2],
                sample.dt);

    const auto& state = ekf_.GetState();

    memcpy(pub_.quaternion, state.q,        sizeof(pub_.quaternion));
    memcpy(pub_.gyro,       sample.gyro,    sizeof(pub_.gyro));
    memcpy(pub_.accel,      sample.accel,   sizeof(pub_.accel));
    memcpy(pub_.gyro_bias,  state.GyroBias, sizeof(pub_.gyro_bias));

    pub_.temperature = sample.temperature;
    pub_.roll        = state.Roll;
    pub_.pitch       = state.Pitch;
    pub_.yaw         = state.Yaw;
    pub_.yaw_total   = state.YawTotalAngle;

    printk("%f,%f,%f\n", pub_.roll, pub_.pitch, pub_.yaw);

    // return zbus_chan_pub(&pub_imu_to, &pub_, K_MSEC(1)) == 0;
    return true;
}

bool ImuWorker::SelectSource(GyroType type, bool enable_auto_calibration)
{
    switch (type)
    {
        case GyroType::BMI088:
        {
        #ifdef CONFIG_MOD_DEV_IMU_BMI088
            if (!bmi088::ConfigureFromDevicetree(1, enable_auto_calibration)) {
                return false;
            }
            source_ = &bmi088::Instance();
            return true;
        #else
            return false;
        #endif
        }

        case GyroType::None:
            return false;

        default:
            return false;
    }
}

void ImuWorker::Task()
{
    for (;;)
    {
        if (source_ != nullptr && source_->Read(sample_)) {
            const uint32_t now_cycle = k_cycle_get_32();
            sample_.dt = MeasuredDtSeconds(now_cycle, last_sample_cycle_, sample_.dt);
            last_sample_cycle_ = now_cycle;
            (void)Process(sample_);
        }

        k_msleep(source_ != nullptr ? source_->PeriodMs() : 1);
    }
}

static ImuWorker imu_ {};

void thread_init()
{
    imu_.Init(GyroType::BMI088, true);
}

void thread_start(uint8_t prio)
{
    imu_.Start(prio);
}

} // namespace thread::imu
