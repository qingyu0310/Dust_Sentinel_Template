/**
 * @file vt12.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-05-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "vt12.hpp"
#include "protocol_base.hpp"

using namespace topic::remote_to;

namespace vt12 {

struct OutputData
{
    Mouse mouse;
    Keyboard keyboard;
};

// 将解析后的数据发布到 zbus 通道
inline static void publishOutputData(const OutputData& od, Message& pub)
{
    pub.chassisy = od.keyboard.w ? 1.0f : od.keyboard.s ? -1.0f : 0.0f;
    pub.chassisx = od.keyboard.a ? 1.0f : od.keyboard.d ? -1.0f : 0.0f;
    pub.pitch    = od.mouse.y;
    pub.yaw      = od.mouse.x;

    if (od.keyboard.shift) {
        pub.chassis_mode = ChassisMode::Spin;
    }  else {
        pub.chassis_mode = ChassisMode::Normal;
    }

    pub.shoot_ctrl    = od.keyboard.f  ? StartMode::On : StartMode::Off;
    pub.reload_ctrl   = od.mouse.left  ? StartMode::On : StartMode::Off;
    pub.autoaim_ctrl  = od.mouse.right ? StartMode::On : StartMode::Off;
    pub.supercap_ctrl = od.keyboard.v  ? StartMode::On : StartMode::Off;

    pub.version++;
    zbus_chan_pub(&pub_remote_to, &pub, K_MSEC(1));
}

// 解码原始遥控器数据 → 返回 false 表示帧错位
bool dataprocess(uint8_t* buffer, uint8_t len, Message& pub)
{
    if (len < 16) return false;

    //  协议头（6 字节）
    uint8_t  start_of_frame = buffer[0];
    // 老裁判系统通用数据，未用上
    // uint16_t data_length    = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);  
    // uint8_t  seq            = buffer[3];
    // uint8_t  crc8           = buffer[4];
    // uint8_t  cmd_id         = buffer[5];

    // 校验：帧头
    if (start_of_frame != 0xA5) return false;

    static KeyboardState keyboard_state_{};

    OutputData od{};

    int16_t dx = (int16_t)((uint16_t)buffer[6]  | ((uint16_t)buffer[7]  << 8));
    int16_t dy = (int16_t)((uint16_t)buffer[8]  | ((uint16_t)buffer[9]  << 8));
    int16_t dz = (int16_t)((uint16_t)buffer[10] | ((uint16_t)buffer[11] << 8));

    constexpr float kMouseScaleX = 30.0f;
    constexpr float kMouseScaleY = 2.0f;
    constexpr float kMouseScaleZ = 1.0f;

    od.mouse.x      = normMouse(static_cast<float>(dx), kMouseScaleX);
    od.mouse.y      = normMouse(static_cast<float>(dy), kMouseScaleY);
    od.mouse.z      = normMouse(static_cast<float>(dz), kMouseScaleZ);
    od.mouse.left   = buffer[12] != 0;
    od.mouse.right  = buffer[13] != 0;

    Keyboard cur_raw { .all = static_cast<uint16_t>((uint16_t)buffer[14] | ((uint16_t)buffer[15] << 8)) };
    keyboard_state_.Process(od.keyboard, cur_raw);

    publishOutputData(od, pub);
    return true;
}

}









