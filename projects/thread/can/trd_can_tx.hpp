/**
 * @file trd_can_tx.hpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "stdint.h"

namespace thread::can {
    void thread_init();
    void thread_start(uint8_t prio = 5);
}
