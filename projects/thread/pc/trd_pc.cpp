/**
 * @file trd_pc.cpp
 * @author qingyu
 * @brief PC 通信线程 — USB CDC ACM 回环测试
 * @version 0.2
 * @date 2026-06-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_pc.hpp"
#include "thread.hpp"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "usb.hpp"
#include <zephyr/devicetree.h>

namespace thread::pc {

static Thread<2048> thread_ {};

static Usb usb_ {};
static const uint8_t tick_data[] = { "tick\r\n" };

static void Task(void*, void*, void*)
{
    uint8_t rx_buf[128] {};
    int64_t next_tick_ms = k_uptime_get() + 1000;

    for (;;)
    {
        if (usb_.IsConfigured() && !usb_.IsTxBusy())
        {
            uint16_t len = usb_.Read(rx_buf, sizeof(rx_buf));
            if (len > 0) {
                (void)usb_.Send(rx_buf, len);
                continue;
            }
        }

        int64_t now = k_uptime_get();
        if (now >= next_tick_ms) {
            next_tick_ms = now + 1000;
            (void)usb_.Send(tick_data, sizeof(tick_data));
        }

        k_sleep(K_MSEC(10));
    }
}

void thread_init()
{
    Usb::Config cfg {};
    cfg.busid = 0;
    cfg.reg_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));
    cfg.buf_size = 512;

    printk("cherryusb cdc_acm test begin base=0x%x\n", cfg.reg_base);

    if (usb_.Init(cfg)) {
        printk("cherryusb cdc_acm test ready\n");
    } else {
        printk("cherryusb cdc_acm test init failed\n");
    }
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}

} // namespace thread::pc
