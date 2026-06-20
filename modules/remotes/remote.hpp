/**
 * @file remote.hpp
 * @author qingyu
 * @brief 遥控器接收机 — 线程内协议解析
 * @version 0.3
 * @date 2026-05-13
 */

#pragma once

#include <cstdint>
#include "thread.hpp"
#include "uart.hpp"
#include "remote_to.hpp"

enum class RemoteType : uint8_t
{
    DR16 = 0,
    VT12,
    VT13,
    Auto,
    None,
};

namespace remote {

class Remote final
{
public:
    using ValidateFunc = bool (*)(const uint8_t *buffer, uint8_t len);
    using DecodeFunc = bool (*)(const uint8_t *buffer, uint8_t len, topic::remote_to::Message &pub);

    struct Protocol {
        RemoteType    type;
        const char    *name;
        uint16_t      frame_size;
        ValidateFunc  validate;
        DecodeFunc    decode;
        uint8_t       lock_score;
    };

    enum class DetectState : uint8_t
    {
        Detecting,
        Locked,
    };

    void Init(RemoteType type, RxStream &uart);
    void Start(uint8_t prio = 5);
    bool IsReady() const { return ready_; }

private:
    Thread<1024 * 8> thread_ {};
    RxStream *uart_ = nullptr;

    RemoteType  configured_type_ = RemoteType::Auto;
    RemoteType  active_type_     = RemoteType::None;
    DetectState detect_state_    = DetectState::Detecting;

    static constexpr uint16_t kFrameBufSize     = 64;
    static constexpr uint8_t  kUnlockFailLimit  = 5;
    static constexpr uint32_t kRemoteTimeoutMs  = 100;
    static constexpr uint8_t  kProtocolCount    = static_cast<uint8_t>(RemoteType::Auto);

    uint8_t  frame_buf_[kFrameBufSize] {};
    uint16_t frame_pos_ = 0;
    k_sem    rx_sem_;

    topic::remote_to::Message  pub_ {};
    Protocol proto_ {};
    uint8_t  hit_count_[kProtocolCount] {};
    uint8_t  fail_count_     = 0;
    uint32_t last_valid_ms_  = 0;
    uint16_t min_frame_size_ = 0;
    uint16_t max_frame_size_ = 0;

    bool ready_ = false;

    void InitFrameSizeRange();
    void GetProcessFunc();
    const Protocol *FindProtocol(RemoteType type);
    void ResetDetect();
    void Consume(uint16_t len);
    void DropOneByte();
    void ClearPubData();
    void Publish();
    void HandleLocked();
    void HandleDetecting();
    void ProcessBuffered();
    void ProcessChunk(const uint8_t *data, uint16_t len);
    void Task();

    static void TaskEntry(void *p1, void *p2, void *p3)
    {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto self = static_cast<Remote *>(p1);
        self->Task();
    }
};

} // namespace remote
