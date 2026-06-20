/**
 * @file trd_remote.hpp
 * @author qingyu
 * @brief 遥控器线程 — UART 接收、协议解析
 * @version 0.1
 * @date 2026-06-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>

namespace thread::remote {
    void thread_init();
    void thread_start(uint8_t prio = 5);
};
