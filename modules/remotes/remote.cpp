/**
 * @file remote.cpp
 * @author qingyu
 * @brief Remote thread driven by UART RX notifications.
 * @version 0.3
 * @date 2026-05-13
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "remote.hpp"
#include "thread.hpp"
#include "uart.hpp"

#include "dr16.hpp"
#include "vt12.hpp"
#include "vt13.hpp"

#include <zephyr/sys/printk.h>

using namespace topic::remote_to;

namespace thread::remote {

static bool remote_ready_ = false;

class Remote final
{
public:
    using DataProcessFunc = bool (*)(uint8_t *buffer, uint8_t len, Message &pub);

    struct Protocol {
        DataProcessFunc func;
        uint16_t frame_size;
    };

    void Init(RemoteType type, RxStream &uart)
    {
        type_ = type;
        k_sem_init(&rx_sem_, 0, 1);
        uart_ = &uart;
        uart_->SetNotify(&rx_sem_);
        GetProcessFunc();
    }

    void Start(uint8_t prio = 5)
    {
        thread_.Start(TaskEntry, prio, this);
    }

private:
    Thread<1024 * 8> thread_ {};
    RxStream *uart_ = nullptr;

    RemoteType type_ = RemoteType::None;

    static constexpr uint16_t kFrameBufSize = 64;
    uint8_t frame_buf_[kFrameBufSize] {};
    uint16_t frame_pos_ = 0;
    k_sem rx_sem_;

    Message pub_ {};
    Protocol proto_ { nullptr, 0 };

    void GetProcessFunc();

    void ClearPubData()
    {
        pub_ = {};
    }

    void ProcessChunk(const uint8_t *data, uint16_t len)
    {
        uint16_t cap = kFrameBufSize - frame_pos_;
        if (len > cap) {
            len = cap;
        }

        memcpy(frame_buf_ + frame_pos_, data, len);
        frame_pos_ += len;

        while (proto_.frame_size > 0 && frame_pos_ >= proto_.frame_size) {
            if (proto_.func(frame_buf_, proto_.frame_size, pub_)) {
                uint16_t rem = frame_pos_ - proto_.frame_size;
                if (rem > 0) {
                    memmove(frame_buf_, frame_buf_ + proto_.frame_size, rem);
                }
                frame_pos_ = rem;
            } else {
                frame_pos_--;
                memmove(frame_buf_, frame_buf_ + 1, frame_pos_);
            }
        }
    }

    void Task()
    {
        for (;;) 
        {
            if (k_sem_take(&rx_sem_, K_MSEC(50)) == 0) {
                uint8_t tmp[32];
                uint32_t start = k_cycle_get_32();
                uint32_t total_bytes = 0;
                uint32_t read_count = 0;

                while (true) {
                    uint16_t n = uart_->Read(tmp, sizeof(tmp));
                    if (n == 0) {
                        break;
                    }

                    total_bytes += n;
                    read_count++;
                    ProcessChunk(tmp, n);
                }

                uint32_t cost_us = k_cyc_to_us_floor32(k_cycle_get_32() - start);
                // printk("%u us, %u bytes, %u reads, frame_pos=%u\n",
                //        cost_us, total_bytes, read_count, frame_pos_);
            } else {
                frame_pos_ = 0;
                ClearPubData();
                zbus_chan_pub(&pub_remote_to, &pub_, K_MSEC(1));
            }
        }
    }

    static void TaskEntry(void *p1, void *p2, void *p3)
    {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto self = static_cast<Remote *>(p1);
        self->Task();
    }
};

static constexpr Remote::Protocol kProtocols[] = {
    { dr16::dataprocess, dr16::kFrameSizeDR16 },
    { vt12::dataprocess, vt12::kFrameSizeVT12 },
    { vt13::dataprocess, vt13::kFrameSizeVT13 },
    { nullptr, 0 },
};

static Remote remote_ {};

void thread_init()
{
    static UartDma rx {};

    RxStream::Config cfg {};
    cfg.buf_size = dr16::kFrameSizeDR16;
    cfg.rx_timeout = 1000;
    if (!rx.Init(DEVICE_DT_GET(DT_ALIAS(uart_remote)), cfg)) {
        remote_ready_ = false;
        return;
    }

    remote_.Init(RemoteType::DR16, rx);
    remote_ready_ = true;
}

void thread_start(uint8_t prio)
{
    if (!remote_ready_) {
        return;
    }

    remote_.Start(prio);
}

void Remote::GetProcessFunc()
{
    uint8_t idx = static_cast<uint8_t>(type_);
    if (idx < sizeof(kProtocols) / sizeof(kProtocols[0])) {
        proto_ = kProtocols[idx];
    }
}

} // namespace thread::remote
