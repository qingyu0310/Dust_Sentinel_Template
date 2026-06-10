/**
 * @file trd_test.hpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-06-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>

namespace thread::test {
    void thread_init();
    void thread_start(uint8_t prio = 6);
};
