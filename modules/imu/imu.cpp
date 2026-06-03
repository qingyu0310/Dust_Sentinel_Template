/**
 * @file imu.cpp
 * @author qingyu
 * @brief IMU 工作线程：选择数据源、采样、姿态更新与消息发布。
 * @version 0.1
 * @date 2026-06-01
 */

#include "imu.hpp"
#include "imu_to.hpp"
#include "bmi088/bmi088.hpp"
#include "icm42688p/icm42688p.hpp"
#include "quaternion_ekf.hpp"
#include "thread.hpp"
#include "zephyr/sys/printk.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

namespace {

// 通过设备树注册数据源时使用的默认轮询周期，单位：ms。
constexpr uint32_t kDefaultImuPeriodMs = 1U;

/**
 * @brief 用真实经过时间修正本次样本的积分周期。
 *
 * `sample.dt` 初始来自驱动侧标称周期，但线程调度、总线访问和日志输出
 * 都可能拉长真实采样间隔，所以这里优先使用 cycle counter 计算相邻样本
 * 之间的真实时间，给姿态解算提供更准确的 `dt`。
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

using RegisterFromDevicetreeFn = bool (*)(uint32_t period_ms, bool auto_calibration);

/**
 * @brief 通过驱动提供的设备树注册函数配置 IMU 数据源。
 *
 * 这样 `SelectSource()` 只负责“选哪个 IMU”，具体 alias、SPI 规格和
 * 驱动侧配置细节留在各自驱动里。
 */
bool RegisterSourceFromDevicetree(RegisterFromDevicetreeFn register_fn, bool enable_auto_calibration)
{
    if (register_fn == nullptr ||
        !register_fn(kDefaultImuPeriodMs, enable_auto_calibration)) {
        return false;
    }

    return true;
}

} // namespace

namespace thread::imu {

/**
 * @brief IMU 工作对象，负责串起数据源、姿态解算和消息发布。
 *
 * 运行流程：
 * 1. 选择并初始化具体 IMU 数据源。
 * 2. 周期性读取一帧样本。
 * 3. 用真实经过时间修正样本 `dt`。
 * 4. 更新四元数 EKF。
 * 5. 发布本项目使用的 IMU topic。
 */
class ImuWorker final
{
public:
    void Init(ImuType type, bool enable_auto_calibration);
    void Start(uint8_t prio);
    bool Process(const Sample& sample);

private:
    bool SelectSource(ImuType type, bool enable_auto_calibration);
    void Task();

    static void TaskEntry(void *p1, void *p2, void *p3)
    {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto *self = static_cast<ImuWorker*>(p1);
        self->Task();
    }

    Thread<4096>                  thread_ {};
    Source                       *source_ = nullptr;            // 当前选中的 IMU 驱动实现
    Sample                        sample_ {};                   // 线程复用的一帧样本缓冲
    alg::attitude::QuaternionEkf  ekf_ {};                      // 姿态解算器
    topic::imu_to::Message        pub_ {};                      // 对外发布的 IMU 消息
    uint32_t                      last_sample_cycle_ = 0U;      // 上一帧成功样本的 cycle 时间戳
    bool                          ready_ = false;               // 数据源和 EKF 都初始化成功后置位
};

/**
 * @brief 选择数据源并完成源侧与 EKF 初始化。
 */
void ImuWorker::Init(ImuType type, bool enable_auto_calibration)
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

/**
 * @brief 初始化成功后启动 IMU 工作线程。
 */
void ImuWorker::Start(uint8_t prio)
{
    if (!ready_) {
        return;
    }

    thread_.Start(TaskEntry, prio, this);
}

/**
 * @brief 把一帧样本送入 EKF，并发布处理后的 IMU topic。
 *
 * 对外发布的 `gyro` 是 EKF 状态里的扣偏角速度，不是驱动刚读出来的原始角速度。
 */
bool ImuWorker::Process(const Sample& sample)
{
    ekf_.Update(sample);

    const auto& state = ekf_.GetState();

    memcpy(pub_.quaternion, state.q,    sizeof(pub_.quaternion));
    memcpy(pub_.gyro,       state.Gyro, sizeof(pub_.gyro));

    pub_.roll        = state.Roll;
    pub_.pitch       = state.Pitch;
    pub_.yaw         = state.Yaw;
    pub_.yaw_total   = state.YawTotalAngle;
    pub_.temperature = sample.temperature;

    printk("%f,%f,%f\n", pub_.roll, pub_.pitch, pub_.yaw);

    return zbus_chan_pub(&pub_imu_to, &pub_, K_MSEC(1)) == 0;
}

/**
 * @brief 主循环：采样、修正 dt、更新 EKF、发布消息。
 */
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

/**
 * @brief 按类型把工作线程绑定到具体 IMU 数据源。
 */
bool ImuWorker::SelectSource(ImuType type, bool enable_auto_calibration)
{
    switch (type)
    {
    #if CONFIG_MOD_DEV_IMU_ICM42688P
        case ImuType::ICM42688P:
            source_ = &icm42688p::Instance();
            return source_ != nullptr &&
                   RegisterSourceFromDevicetree(icm42688p::RegisterFromDevicetree, enable_auto_calibration);
    #endif

    #if CONFIG_MOD_DEV_IMU_BMI088
        case ImuType::BMI088:
            source_ = &bmi088::Instance();
            return source_ != nullptr &&
                   RegisterSourceFromDevicetree(bmi088::RegisterFromDevicetree, enable_auto_calibration);
    #endif

        case ImuType::None:
            return false;

        default:
            return false;
    }
}

/**
 * @brief 根据当前编译配置自动选择默认 IMU 数据源。
 *
 * @return 本次启动应使用的默认 IMU 类型。
 */
ImuType AutoChooseImuType()
{
#if CONFIG_MOD_DEV_IMU_ICM42688P
    return ImuType::ICM42688P;

#elif CONFIG_MOD_DEV_IMU_BMI088
    return ImuType::BMI088;

#else
    return ImuType::None;

#endif
}

static ImuWorker imu_ {};

void thread_init()
{
    imu_.Init(AutoChooseImuType(), true);
}

void thread_start(uint8_t prio)
{
    imu_.Start(prio);
}

} // namespace thread::imu
