/**
 * @file trd_pc.hpp
 * @author qingyu
 * @brief PC 通信线程 — USB CDC ACM 回环测试
 * @version 0.1
 * @date 2026-06-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>

namespace thread::pc {
    void thread_init();
    void thread_start(uint8_t prio = 6);
};
