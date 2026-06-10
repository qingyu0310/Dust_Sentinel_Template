/**
 * @file remote_to.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include <zephyr/zbus/zbus.h>

namespace topic::remote_to {

enum class ChassisMode : uint8_t
{
    Normal = 0,
    Spin,
};

enum class StartMode : uint8_t
{
    Off = 0,
    On,
};

struct Channel
{
    float chassisx  = 0.0f, chassisy  = 0.0f;
    float pitch = 0.0f, yaw = 0.0f;
};

struct Message 
{
    uint32_t version = 0;

    /* 语义字段（归一化 [-1, 1]） */
    float chassisx = 0.0f;
    float chassisy = 0.0f;
    float yaw      = 0.0f;
    float pitch    = 0.0f;

    /* 模式标志 */
    ChassisMode chassis_mode = ChassisMode::Normal;

    StartMode shoot_ctrl     = StartMode::Off;
    StartMode reload_ctrl    = StartMode::Off;
    StartMode autoaim_ctrl   = StartMode::Off;
    StartMode supercap_ctrl  = StartMode::Off;
};

}

ZBUS_CHAN_DECLARE(pub_remote_to);

ZBUS_OBS_DECLARE(sub_remote_to);
