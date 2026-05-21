/**
 * @file trd_gpio.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-04-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_gpio.hpp"
#include "thread.hpp"
#if CONFIG_MOD_DEV_WS2812B
#include "ws2812b.hpp"
#else
#include "output.hpp"
#include "input.hpp"
#endif

#define GPIO_GET(node_id)   GPIO_DT_SPEC_GET(DT_NODELABEL(node_id), gpios)

namespace thread::output {

static Thread<2048> thread_{};

#if CONFIG_MOD_DEV_WS2812B
static Ws2812b led_alert{};
#else
static Output led_alert{};
#endif

static void led_cycle()
{
#if CONFIG_MOD_DEV_WS2812B

    static uint8_t phase = 0;
    static constexpr Ws2812b::Color colors[] {
        {32, 0,  0 },
        {0,  32, 0 },
        {0,  0,  32},
        {0,  0,  0 },
    };
    led_alert.set(colors[phase++ % ARRAY_SIZE(colors)]);

#else

    led_alert.Toggle();
    
#endif
}

static void Task(void*, void*, void*)
{
    static constexpr uint32_t kPeriodMs = 500;

    for (;;)
    {
        const int64_t tick_start = k_uptime_get();

        led_cycle();

        const int64_t elapsed = k_uptime_get() - tick_start;
        const int64_t remain  = static_cast<int64_t>(kPeriodMs) - elapsed;
        if (remain > 0) {
            k_msleep(remain);
        }
    }
}

void thread_init()
{
#if CONFIG_MOD_DEV_WS2812B
    led_alert.init(HPM_GPIO0, 0, 10);
    led_alert.set_color_order(Ws2812b::ColorOrder::RGB);
#else
    led_alert.init(GPIO_GET(led_alert));
#endif
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}

} // namespace thread::output

namespace thread::input {

static Thread<> thread_{};

/*  占位：预留用于按键/传感器输入，待实现  */
static void Task(void*, void*, void*)
{
    for (;;)
    {
        k_msleep(100);
    }
}

void thread_init()
{
    
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}


} // namespace thread::input
