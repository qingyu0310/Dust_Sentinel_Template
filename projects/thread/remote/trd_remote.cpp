/**
 * @file trd_remote.cpp
 * @author qingyu
 * @brief 遥控器线程 — UART 接收、协议解析
 * @version 0.1
 * @date 2026-06-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_remote.hpp"
#include "remote.hpp"
#include "uart.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(remote, LOG_LEVEL_INF);

namespace thread::remote {

static ::remote::Remote remote_ {};

/**
 * @brief 初始化 UART DMA，并将遥控器解码器配置为自动识别模式。
 */
void thread_init()
{
    static UartDma rx {};
    RxStream::Config cfg {};

    constexpr uint16_t kBufferSize = 128;
    constexpr uint16_t kTimeour    = 1000;

    cfg.buf_size   = kBufferSize;
    cfg.rx_timeout = kTimeour;

    if (!rx.Init(DEVICE_DT_GET(DT_ALIAS(remote_uart)), cfg)) {
        LOG_ERR("uart init failed");
        return;
    }

    remote_.Init(RemoteType::Auto, rx);
}

/**
 * @brief 如果初始化成功，则启动遥控器线程。
 *
 * @param prio Zephyr 线程优先级。
 */
void thread_start(uint8_t prio)
{
    if (!remote_.IsReady()) {
        return;
    }

    remote_.Start(prio);
}

} // namespace thread::remote
