/**
 * @file trd_test.cpp
 * @author qingyu
 * @brief PC USB CDC loopback test thread.
 * @version 0.1
 * @date 2026-06-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "trd_test.hpp"

#include "thread.hpp"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#ifdef CONFIG_COM_USB
#include "usb.hpp"
#endif

#ifdef CONFIG_TRD_TEST_CHERRYUSB
#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#include <zephyr/devicetree.h>
#endif

#ifdef CONFIG_TRD_TEST_CHERRYUSB
static volatile bool g_cdc_dtr_ready;
static volatile bool g_cdc_rts_ready;
static cdc_line_coding g_cdc_line_coding {
    115200,
    0,
    0,
    8,
};

extern "C" void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    (void)busid;
    (void)intf;

    g_cdc_line_coding = *line_coding;
    printk("cherryusb line coding baud=%u data=%u parity=%u stop=%u\n",
           g_cdc_line_coding.dwDTERate,
           g_cdc_line_coding.bDataBits,
           g_cdc_line_coding.bParityType,
           g_cdc_line_coding.bCharFormat);
}

extern "C" void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    (void)busid;
    (void)intf;

    *line_coding = g_cdc_line_coding;
}

extern "C" void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;

    g_cdc_dtr_ready = dtr;
    printk("cherryusb dtr=%u\n", dtr ? 1U : 0U);
}

extern "C" void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{
    (void)busid;
    (void)intf;

    g_cdc_rts_ready = rts;
    printk("cherryusb rts=%u\n", rts ? 1U : 0U);
}
#endif

namespace thread::test {

static Thread<2048> thread_ {};

#ifdef CONFIG_TRD_TEST_CHERRYUSB

#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x83

#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const uint8_t config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const uint8_t other_speed_config_descriptor_hs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t other_speed_config_descriptor_fs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const char lang_id[] = { 0x09, 0x04 };

static const char *string_descriptors[] = {
    lang_id,
    "HPMicro",
    "HPMicro CDC DEMO",
    "2024051702",
};

static const uint8_t *DeviceDescriptorCallback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

static const uint8_t *ConfigDescriptorCallback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return config_descriptor_hs;
    }

    if (speed == USB_SPEED_FULL) {
        return config_descriptor_fs;
    }

    return nullptr;
}

static const uint8_t *DeviceQualityDescriptorCallback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}

static const uint8_t *OtherSpeedConfigDescriptorCallback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return other_speed_config_descriptor_hs;
    }

    if (speed == USB_SPEED_FULL) {
        return other_speed_config_descriptor_fs;
    }

    return nullptr;
}

static const char *StringDescriptorCallback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
        return nullptr;
    }

    return string_descriptors[index];
}

static const struct usb_descriptor cdc_descriptor = {
    DeviceDescriptorCallback,
    ConfigDescriptorCallback,
    DeviceQualityDescriptorCallback,
    OtherSpeedConfigDescriptorCallback,
    StringDescriptorCallback,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t read_buffer[2][512];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t tick_buffer[] = "tick\r\n";
static volatile uint8_t read_buffer_index;
static volatile bool cdc_configured;
static volatile bool cdc_tx_busy;

static void UsbdEventHandler(uint8_t busid, uint8_t event)
{
    printk("cherryusb event=%u\n", event);

    switch (event) {
    case USBD_EVENT_RESET:
    case USBD_EVENT_DISCONNECTED:
        cdc_configured = false;
        cdc_tx_busy = false;
        g_cdc_dtr_ready = false;
        return;
    case USBD_EVENT_CONFIGURED:
        cdc_configured = true;
        cdc_tx_busy = false;
        break;
    default:
        return;
    }

    read_buffer_index = 0;
    usbd_ep_start_read(busid, CDC_OUT_EP, &read_buffer[0][0], usbd_get_ep_mps(busid, CDC_OUT_EP));
}

static void UsbdCdcAcmBulkOut(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    uint8_t index = read_buffer_index;

    read_buffer_index = (index == 0U) ? 1U : 0U;
    usbd_ep_start_write(busid, CDC_IN_EP, &read_buffer[index][0], nbytes);
    usbd_ep_start_read(busid, ep, &read_buffer[read_buffer_index][0], usbd_get_ep_mps(busid, ep));
}

static void UsbdCdcAcmBulkIn(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0U && nbytes != 0U) {
        usbd_ep_start_write(busid, ep, nullptr, 0);
        return;
    }

    cdc_tx_busy = false;
}

static struct usbd_endpoint cdc_out_ep = {
    CDC_OUT_EP,
    UsbdCdcAcmBulkOut,
};

static struct usbd_endpoint cdc_in_ep = {
    CDC_IN_EP,
    UsbdCdcAcmBulkIn,
};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

static void CdcAcmInit(uint8_t busid, uint32_t reg_base)
{
    printk("cherryusb cdc desc register\n");
    usbd_desc_register(busid, &cdc_descriptor);
    printk("cherryusb cdc add intf0\n");
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    printk("cherryusb cdc add intf1\n");
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    printk("cherryusb cdc add out ep\n");
    usbd_add_endpoint(busid, &cdc_out_ep);
    printk("cherryusb cdc add in ep\n");
    usbd_add_endpoint(busid, &cdc_in_ep);
    printk("cherryusb cdc usbd_initialize\n");
    usbd_initialize(busid, reg_base, UsbdEventHandler);
    printk("cherryusb cdc usbd_initialize done\n");
}

static void Task(void*, void*, void*)
{
    for (;;) {
        k_sleep(K_SECONDS(1));

        if (!cdc_configured || cdc_tx_busy) {
            continue;
        }

        cdc_tx_busy = true;
        if (usbd_ep_start_write(0, CDC_IN_EP, tick_buffer, sizeof(tick_buffer) - 1U) != 0) {
            cdc_tx_busy = false;
        }
    }
}

void thread_init()
{
    const uint32_t usb_base = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));

    printk("cherryusb cdc_acm test begin base=0x%x\n", usb_base);
    CdcAcmInit(0, usb_base);
    printk("cherryusb cdc_acm test ready\n");
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}

#elif defined(CONFIG_COM_USB)

#define PC_USB_NODE DT_ALIAS(pc_usb)
BUILD_ASSERT(DT_NODE_HAS_STATUS(PC_USB_NODE, okay),
             "pc-usb alias must point to an enabled CDC ACM UART");

static UsbUart usb_ {};
static K_SEM_DEFINE(usb_rx_sem_, 0, 1);

static void Task(void*, void*, void*)
{
    uint8_t rx[64] {};

    for (;;) {
        if (k_sem_take(&usb_rx_sem_, K_FOREVER) != 0) {
            continue;
        }

        uint16_t len = usb_.Read(rx, sizeof(rx));
        if (len > 0U) {
            printk("zephyr_usb rx len=%u\n", len);
            (void)usb_.Send(rx, len);
        }
    }
}

void thread_init()
{
    UsbUart::Config cfg {};
    cfg.buf_size = 256;
    cfg.wait_dtr = false;
    cfg.assert_line_state = true;

    const struct device* dev = DEVICE_DT_GET(PC_USB_NODE);

    if (!usb_.Init(dev, cfg)) {
        printk("zephyr_usb init failed dev=%s\n", dev->name);
        return;
    }

    usb_.SetNotify(&usb_rx_sem_);
    printk("zephyr_usb init ok dev=%s\n", dev->name);
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}

#else

static void Task(void*, void*, void*)
{
    for (;;) {
        k_sleep(K_FOREVER);
    }
}

void thread_init()
{
    printk("trd_test needs CONFIG_COM_USB=y\n");
}

void thread_start(uint8_t prio)
{
    thread_.Start(Task, prio);
}

#endif

} // namespace thread::test
