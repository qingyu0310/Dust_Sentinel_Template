/**
 * @file dm.cpp
 * @author qingyu
 * @brief 达妙（DM）系列电机驱动
 *
 * ## 协议说明
 *
 * ### 反馈帧（RX — 8 字节）
 * | 字节 | 内容 |
 * |------|------|
 * | 0    | bit[3:0] = CAN_ID, bit[7:4] = 错误状态 |
 * | 1-2  | 编码器值（16bit） |
 * | 3-4  | 速度（12bit，高 4bit 在字节 3 低半字节） |
 * | 4-5  | 转矩（12bit，低 4bit 在字节 4 高半字节） |
 * | 6    | MOS 温度 |
 * | 7    | 线圈温度 |
 *
 * ### 控制帧（TX — 因模式而异）
 *
 * 达妙电机控制 CAN_ID = 基础 CAN_ID + 模式偏移量：
 * | 模式 | 偏移 | 说明 |
 * |------|------|------|
 * | MIT  | 0x000 | 刚度-阻尼位置控制 |
 * | Pos  | 0x100 | 位置-速度控制（float32×2） |
 * | Spd  | 0x200 | 速度控制（float32） |
 * | Psi  | 0x300 | 力位混合控制 |
 *
 * 注意：不同于 DJI 电机固定 1（TX）对 4（RX）的 ID 映射，
 * DM 的 TX ID 随模式自动偏移，同一电机在不同模式下
 * 使用不同的 CAN ID 发送控制帧。
 *
 * @version 0.1
 * @date 2026-05-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "dm.hpp"
#include <cstdint>

namespace {

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

}

/**
 * @brief 初始化电机配置
 *
 * 根据控制模式附加 CAN_ID 偏移（Mit=0x000 / Pos=0x100 / Spd=0x200 / Psi=0x300），
 * 同一电机在不同模式下使用不同的 TX ID 发送控制帧。
 *
 * @param cfg 电机配置参数
 */
void DmMotor::Init(Config cfg)
{
    cfg_ = cfg;

    switch (cfg_.ctrl_met)
    {
        case ControlMethon::Mit:
            cfg_.can_id += 0x000;           // MIT: 刚度-阻尼位置控制
            break;
        case ControlMethon::Pos:
            cfg_.can_id += 0x100;           // Pos: 位置-速度控制
            break;
        case ControlMethon::Spd:
            cfg_.can_id += 0x200;           // Spd: 纯速度控制
            break;
        case ControlMethon::Psi:
            cfg_.can_id += 0x300;           // Psi: 力位混合控制
            break;
    }
}

/**
 * @brief 定时掉线检测
 *
 */
void DmMotor::PwrLossCheck()
{
    power_lost_ = (flag_ == pre_flag_);
    pre_flag_ = flag_;
}

/**
 * @brief CAN 接收中断回调，解析电机反馈数据
 *
 * 反馈帧格式（字节0包含 ID + 错误状态，字节1-7为编码器/速度/转矩/温度）：
 *   [0] bit[3:0] = ID, bit[7:4] = err
 *   [1-2] 编码器 (16bit)
 *   [3-4] 速度 (12bit)
 *   [4-5] 转矩 (12bit)
 *   [6] MOS 温度
 *   [7] 线圈温度
 *
 * @param buffer CAN 数据帧 8 字节
 */
void DmMotor::CanCpltRxCallback(uint8_t* buffer)
{
    const uint8_t* data  =  buffer;

    const uint8_t  id    =  data[0] & 0x0F;      // ID 在低半字节
    if (id != cfg_.can_id) return;

    const uint8_t  err   =   data[0] >> 4;        // 错误状态在高半字节
    const uint16_t enc   = (static_cast<uint16_t>(data[1]) << 8) | data[2];           // 16bit 编码器
    const int32_t  v_int = ( data[3] << 4)         | (data[4] >> 4);                  // 12bit 速度
    const int32_t  t_int = ((data[4] & 0x0F) << 8) |  data[5];                        // 12bit 转矩

    // 多圈角度追踪
    int32_t delta = static_cast<int32_t>(enc) - static_cast<int32_t>(pre_encoder_);
    if (delta > (1 << 15)) {                            // 编码器正向溢出（跳回0）
        total_round_--;
    } else if (delta < -(1 << 15)) {                    // 编码器反向溢出（跳回65535）
        total_round_++;
    }
    total_encoder_ = total_round_ * (1 << 16) + enc     // 多圈绝对编码器值
                     - ((1 << 15) - 1);                 // 去零偏（中心0x8000 → 0）

    /* seqlock 写锁：允许其他中断，线程读到冲突时会自旋重试 */
    atomic_inc(&seq_);
    now_rad_   = static_cast<float>(total_encoder_) / static_cast<float>((1 << 16) - 1)
               * cfg_.pos_max * 2.0f;                   // 缩放到 [-pos_max, +pos_max]
    now_ang_   = now_rad_ * kRad2Deg;                   // rad → °

    const float omega_motor = uint_to_float(v_int, -cfg_.vel_max, cfg_.vel_max, 12);
    now_omg_   = omega_motor;                           // 电机轴角速度 (rad/s)
    const float omega_out   = (cfg_.gearbox_ratio != 0.0f) ? omega_motor / cfg_.gearbox_ratio : omega_motor;
    now_vel_   = (cfg_.wheel_r != 0.0f) ? omega_out * cfg_.wheel_r * 0.5f : 0.0f;     // v = ω_out * r / 2  —— 差速单轮贡献

    now_tor_   = uint_to_float(t_int, -cfg_.tor_max, cfg_.tor_max, 12);
    now_tmos_  = static_cast<float>(data[6]);
    now_tcoil_ = static_cast<float>(data[7]);

    now_err_   = static_cast<DmErrorStatus>(err);
    atomic_inc(&seq_);

    pre_encoder_ = enc;

    flag_++;    // 窗口滑窗，用于检测电机是否断电
}

/**
 * @brief 根据控制模式打包控制帧数据
 *
 * @param data 输出 8 字节 CAN 数据
 */
void DmMotor::CtrlData(uint8_t (&data)[8])
{
    switch (cfg_.ctrl_met)
    {
        case ControlMethon::Mit:
        {
            // MIT协议: KP ∈ [0,500], KD ∈ [0,5] (12bit量化)
            constexpr float kKpMin = 0.0f, kKpMax = 500.0f;
            constexpr float kKdMin = 0.0f, kKdMax = 5.0f;

            const uint16_t pos_tmp = float_to_uint(ctrl.target_rad_, -cfg_.pos_max, cfg_.pos_max, 16);
            const uint16_t vel_tmp = float_to_uint(ctrl.target_omg_, -cfg_.vel_max, cfg_.vel_max, 12);
            const uint16_t kp_tmp  = float_to_uint(cfg_.kp,          kKpMin,        kKpMax,       12);
            const uint16_t kd_tmp  = float_to_uint(cfg_.kd,          kKdMin,        kKdMax,       12);
            const uint16_t tor_tmp = float_to_uint(ctrl.target_tor_, -cfg_.tor_max, cfg_.tor_max, 12);

            data[0] =   pos_tmp >> 8;
            data[1] =   pos_tmp;
            data[2] =   vel_tmp >> 4;
            data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
            data[4] =   kp_tmp;
            data[5] =   kd_tmp  >> 4;
            data[6] = ((kd_tmp  & 0xF) << 4) | (tor_tmp >> 8);
            data[7] =   tor_tmp;

            break;
        }
        case ControlMethon::Pos:
        {
            const uint8_t* pos_bytes = reinterpret_cast<const uint8_t*>(&ctrl.target_rad_);
            const uint8_t* vel_bytes = reinterpret_cast<const uint8_t*>(&ctrl.target_omg_);

            data[0] = pos_bytes[0];
            data[1] = pos_bytes[1];
            data[2] = pos_bytes[2];
            data[3] = pos_bytes[3];
            data[4] = vel_bytes[0];
            data[5] = vel_bytes[1];
            data[6] = vel_bytes[2];
            data[7] = vel_bytes[3];

            break;
        }
        case ControlMethon::Spd:
        {
            const uint8_t* vel_bytes = reinterpret_cast<const uint8_t*>(&ctrl.target_omg_);

            data[0] = vel_bytes[0];
            data[1] = vel_bytes[1];
            data[2] = vel_bytes[2];
            data[3] = vel_bytes[3];
            data[4] = 0;
            data[5] = 0;
            data[6] = 0;
            data[7] = 0;

            break;
        }
        case ControlMethon::Psi:
        {
            const uint8_t* pos_bytes = reinterpret_cast<const uint8_t*>(&ctrl.target_rad_);
            uint16_t u16_vel = static_cast<uint16_t> (ctrl.target_omg_  * 100);                             // 100 为协议固定系数 vel单位为(rad/s)
            uint16_t u16_cur = static_cast<uint16_t>((ctrl.target_tor_ / cfg_.tor_max) * 10000.0f);           // 转矩→标幺值×10000

            data[0] = pos_bytes[0];
            data[1] = pos_bytes[1];
            data[2] = pos_bytes[2];
            data[3] = pos_bytes[3];
            data[4] = u16_vel;
            data[5] = u16_vel >> 8;
            data[6] = u16_cur;
            data[7] = u16_cur >> 8;

            break;
        }
    }
}

/**
 * @brief 使能电机命令帧
 *
 * @param data 输出 8 字节 CAN 数据
 */
void DmMotor::EnableData(uint8_t (&data)[8])
{
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;             // DM 协议使能指令
}

/**
 * @brief 失能电机命令帧
 *
 * @param data 输出 8 字节 CAN 数据
 */
void DmMotor::DisableData(uint8_t (&data)[8])
{
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;             // DM 协议失能指令
}

/**
 * @brief 保存当前位置为零点命令帧
 *
 * @param data 输出 8 字节 CAN 数据
 */
void DmMotor::SaveZeroData(uint8_t (&data)[8])
{
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;             // DM 协议保存零点指令
}

/**
 * @brief 清除电机错误命令帧
 *
 * @param data 输出 8 字节 CAN 数据
 */
void DmMotor::ClearErrData(uint8_t (&data)[8])
{
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFB;             // DM 协议清除错误指令
}
