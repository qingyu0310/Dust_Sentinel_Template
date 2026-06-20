/**
 * @file trd_imu.cpp
 * @author qingyu
 * @brief IMU 线程 — 数据采集、加热控温、姿态解算
 * @version 0.1
 * @date 2026-06-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_imu.hpp"
#include "imu.hpp"

namespace thread::imu {

static ::imu::ImuManager imu_ {};

void thread_init()
{
    imu_.Init(true);
}

void thread_start(uint8_t prio)
{
    if (!imu_.IsReady()) {
        return;
    }

    imu_.Start(prio);
}

} // namespace thread::imu
