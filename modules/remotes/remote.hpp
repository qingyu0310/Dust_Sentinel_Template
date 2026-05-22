/**
 * @file remote.hpp
 * @author qingyu
 * @brief 遥控器接收机 — 线程内协议解析
 * @version 0.3
 * @date 2026-05-13
 */

#pragma once

#include <cstdint>

enum class RemoteType : uint8_t
{
    DR16 = 0,
    VT12,
    VT13,
    Auto,
    None,
};

namespace thread::remote {

void thread_init();
void thread_start(uint8_t prio = 5);

}


