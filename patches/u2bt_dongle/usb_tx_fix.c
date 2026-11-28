/*
 * 8BitDo Ultimate 2 Bluetooth 2.4G Receiver (JieLi BR22, stock public v1.02/v1.03)
 * Replacement for stock usb_g_intr_write() @ 0x1e02384.
 *
 * Fixes two stock bugs that deadlock/timeout runtime mode switching (especially XInput -> Switch):
 * 1. Stock b[0x69b0+0x40] stays 1 across USB re-enumeration; gate writes on USB_CON0 bit 2
 *    and true USB 2.0 device state b[*(0x6b04)+0x214] == 4 (CONFIGURED).
 * 2. Stock timeout uses Priority-1 jiffies (*(0x2b3c)), which freezes inside Priority-3
 *    sys_hi_timer ISR (IRQ 5), causing permanent WDT deadlock while starving USB ISR (IRQ 9).
 *    Use Priority-6 Timer0 (*(0x6568), 2ms) + per-EP non-blocking backoff once an unpolled
 *    endpoint times out (~8-10ms in ISR), auto-clearing as soon as the host drains the FIFO.
 *
 * Placed in verified-zero flash cave at 0x1e32300 (360 bytes compiled with pi32v2 clang -Os).
 */

#include <stdint.h>

#ifdef HOST_TEST
#include "host_test_env.h"
#else
#define usb_read_txcsr  ((uint32_t (*)(uint32_t))0x1e02358u)
#define usb_write_txcsr ((void (*)(uint32_t, uint32_t))0x1e002cau)

#define EP_BUSY_MASK    (*(volatile uint32_t *)0x4c04u) /* boot-zeroed RAM (flash 0x1e32fa0) */
#define T0_TICK_2MS     (*(volatile uint32_t *)0x6568u) /* Priority-6 Timer0 tick (0..499) */
#define USB_DEV_TABLE   ((volatile uint32_t *)0x6afcu)
#define USB_HDL_PTR     (*(volatile uint32_t *)0x6b04u)
#define USB_CON0_REG    (*(volatile uint32_t *)0x1e1800u)
#define USB_TXCOUNT_REG ((volatile uint32_t *)0x1e1808u)
#define USB_STATE_BYTE(hdl) (*(volatile uint8_t *)((hdl) + 0x214u))
#define USB_DEV_DMA_BUF(dev, ep, ep_off) \
    ((ep) == 0u ? *(volatile uint32_t *)(dev) : *(volatile uint32_t *)((dev) + (ep_off) + 4u))
#define USB_DEV_MAXP(dev, ep_off) (*(volatile uint32_t *)((dev) + (ep_off) + 0x44u))
#define USB_DMA_PTR(dma_buf) ((volatile uint8_t *)(dma_buf))

static inline uint32_t read_icfg(void)
{
    uint32_t v;
    __asm__ volatile("%0 = icfg" : "=r"(v));
    return v;
}
#endif

static inline int usb_is_configured(void)
{
    uint32_t usb_hdl = USB_HDL_PTR;
    return (USB_CON0_REG & 4u) != 0u && usb_hdl != 0u && USB_STATE_BYTE(usb_hdl) == 4u;
}

void fix_usb_g_intr_write(uint32_t id, uint32_t ep, const uint8_t *buf, uint32_t len)
{
    if (!usb_is_configured()) {
        EP_BUSY_MASK = 0u;
        return;
    }

    uint32_t dev = USB_DEV_TABLE[id];
    if (!dev || len == 0u)
        return;

    uint32_t ep_off = ep << 2;
    uint32_t dma_buf = USB_DEV_DMA_BUF(dev, ep, ep_off);
    uint32_t maxp = USB_DEV_MAXP(dev, ep_off);
    if (maxp == 0u)
        return;

    uint32_t in_isr = read_icfg() & 0xffu;
    uint32_t ep_bit = 1u << (ep & 7u);
    uint32_t t0_start = T0_TICK_2MS;
    uint32_t max_ticks;
    if (EP_BUSY_MASK & ep_bit) {
        max_ticks = 0u;
    } else if (in_isr) {
        max_ticks = 5u;
    } else {
        max_ticks = ((len >> 6) + 2u) * 5u;
    }

    uint32_t spins = 0u;
    while (len != 0u) {
        if (!usb_is_configured()) {
            EP_BUSY_MASK = 0u;
            return;
        }
        uint32_t txcsr = usb_read_txcsr(ep);
        if ((txcsr & 0x4000u) == 0u) {
            if (max_ticks == 0u && (txcsr & 0x0003u) != 0u)
                return;
            if (txcsr & 0x0004u) {
                usb_write_txcsr(ep, txcsr & ~0x0005u);
                if ((txcsr & 0x0003u) != 0u)
                    goto check_timeout;
                txcsr = usb_read_txcsr(ep);
                if (txcsr & 0x0003u)
                    goto check_timeout;
            } else if (txcsr & 0x0002u) {
                usb_write_txcsr(ep, txcsr & ~0x0003u);
                goto check_timeout;
            } else if (txcsr & 0x0001u) {
                goto check_timeout;
            }
        }

        if (!usb_is_configured()) {
            EP_BUSY_MASK = 0u;
            return;
        }

        EP_BUSY_MASK &= ~ep_bit;

        uint32_t n = (len < maxp) ? len : maxp;
        if (buf != 0) {
            volatile uint8_t *dst = USB_DMA_PTR(dma_buf);
            for (uint32_t i = 0u; i < n; ++i)
                dst[i] = buf[i];
            buf += n;
        }
        USB_TXCOUNT_REG[ep] = n;
        txcsr = usb_read_txcsr(ep);
        usb_write_txcsr(ep, txcsr | 0x0001u);
        len -= n;
        continue;

    check_timeout:
        if (max_ticks == 0u)
            return;
        int32_t elapsed = (int32_t)T0_TICK_2MS - (int32_t)t0_start;
        if (elapsed < 0)
            elapsed += 500;
        if ((uint32_t)elapsed >= max_ticks || ++spins > 20000u) {
            EP_BUSY_MASK |= ep_bit;
            return;
        }
    }
}
