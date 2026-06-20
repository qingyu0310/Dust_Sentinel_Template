/**
 * @file trd_imu.hpp
 * @author qingyu
 * @brief IMU 线程 — 数据采集、加热控温、姿态解算
 * @version 0.1
 * @date 2026-06-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>

namespace thread::imu {
    void thread_init();
    void thread_start(uint8_t prio = 5);
};
