/**
 * @file trd_tflm.cpp
 * @author qingyu
 * @brief TFLM Hello World inference thread
 * @version 0.1
 * @date 2026-05-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_tflm.hpp"
#include "thread.hpp"
#include "tflm.hpp"
#include "zephyr/kernel.h"

namespace thread::ml {

static Thread<8192> thread_{};

static void Task(void*, void*, void*)
{
    for (;;)
    {
        k_msleep(1000);
    }
}

void thread_init()
{
    tflm::init();
    tflm::print_info();
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}

} // namespace thread::tflm_demo
