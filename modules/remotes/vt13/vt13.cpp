/**
 * @file vt13.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-05-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "vt13.hpp"
#include "protocol_base.hpp"

using namespace topic::remote_to;

namespace vt13 {

enum SwitchCNS
{
    SWITCH_C = 0,
    SWITCH_N,
    SWITCH_S,
};

struct RawData
{
    uint8_t start_of_frame_1 = 0xA9;
    uint8_t start_of_frame_2 = 0x53;
    uint64_t channel_0  : 11;
    uint64_t channel_1  : 11;
    uint64_t channel_2  : 11;
    uint64_t channel_3  : 11;
    uint64_t cns        : 2;
    uint64_t pause      : 1;
    uint64_t fn_1       : 1;
    uint64_t fn_2       : 1;
    uint64_t wheel      : 11;
    uint64_t trigger    : 1;
    uint64_t reserved_1 : 3;
    int16_t  mouse_x;
    int16_t  mouse_y;
    int16_t  mouse_z;
    uint8_t  mouse_l    : 2;
    uint8_t  mouse_r    : 2;
    uint8_t  mouse_m    : 2;
    uint8_t  reserved_2 : 2;
    uint16_t keyboard;
    uint16_t crc16;
} __attribute__((packed));

struct Switch
{
    uint8_t fn1, fn2;
    uint8_t trigger;
    uint8_t pause;
    uint8_t cns;
    float wheel;
};

struct OutputData
{
    Channel ch;
    Switch sw;
    Mouse mouse;
    Keyboard keyboard;
};

// 将解析后的数据发布到 zbus 通道
inline static void publishOutputData(const OutputData& od, Message& pub)
{
    processChannel(pub, od);

    switch (od.sw.cns)
    {
        case SWITCH_C:
            pub.chassis_mode = ChassisMode::Spin;
            break;
        case SWITCH_N:
            pub.chassis_mode = ChassisMode::Normal;
            break;
        case SWITCH_S:
            break;
        default:
            pub.chassis_mode = ChassisMode::Normal;
            break;
    }

    pub.reload_ctrl   = od.sw.fn1     ? StartMode::On : StartMode::Off;
    pub.shoot_ctrl    = od.sw.fn2     ? StartMode::On : StartMode::Off;
    pub.autoaim_ctrl  = od.sw.fn2     ? StartMode::On : StartMode::Off;
    pub.supercap_ctrl = od.keyboard.v ? StartMode::On : StartMode::Off;

    pub.version++;
    zbus_chan_pub(&pub_remote_to, &pub, K_MSEC(1));
}

// 解码 + 校验 → 返回 false 表示帧错位
bool dataprocess(uint8_t* buffer, uint8_t len, Message& pub)
{
    if (len < 21) return false;

    // 校验：帧头必须为 0xA9 0x53
    if (buffer[0] != 0xA9 || buffer[1] != 0x53) {
        return false;
    }

    // 端序问题：reinterpret_cast 不处理字节序，VT13 发小端 + ARM 小端才刚好能用
    const RawData* raw_data = reinterpret_cast<RawData const*>(buffer);

    static KeyboardState keyboard_state_{};
    
    OutputData od{};

    constexpr int16_t kCenter = 1024;
    constexpr int16_t kMax    = 1684;

    od.ch.chassisx  = normChannel(raw_data->channel_0, kCenter, kMax);
    od.ch.chassisy  = normChannel(raw_data->channel_1, kCenter, kMax);
    od.ch.yaw       = normChannel(raw_data->channel_2, kCenter, kMax);
    od.ch.pitch     = normChannel(raw_data->channel_3, kCenter, kMax);

    od.sw.fn1       = raw_data->fn_1;
    od.sw.fn2       = raw_data->fn_2;
    od.sw.trigger   = raw_data->trigger;
    od.sw.pause     = raw_data->pause;
    od.sw.cns       = raw_data->cns;
    od.sw.wheel     = normChannel(raw_data->wheel, kCenter, kMax);

    constexpr float kMouseScaleX = 30.0f;
    constexpr float kMouseScaleY = 2.0f;
    constexpr float kMouseScaleZ = 1.0f;

    od.mouse.x      = normMouse(static_cast<float>(raw_data->mouse_x), kMouseScaleX);
    od.mouse.y      = normMouse(static_cast<float>(raw_data->mouse_y), kMouseScaleY);
    od.mouse.z      = normMouse(static_cast<float>(raw_data->mouse_z), kMouseScaleZ);

    od.mouse.left   = raw_data->mouse_l != 0;
    od.mouse.right  = raw_data->mouse_r != 0;

    Keyboard cur_raw { .all = raw_data->keyboard };
    keyboard_state_.Process(od.keyboard, cur_raw);

    publishOutputData(od, pub);
    return true;
}

}
