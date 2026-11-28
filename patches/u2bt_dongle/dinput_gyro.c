/*
 * 8BitDo Ultimate 2 Bluetooth 2.4G Receiver (JieLi BR22, stock public v1.02/v1.03)
 * D-Input 34-byte 6-axis gyro + battery report packer & runtime mode-switch hotkeys.
 *
 * Placed in verified-zero flash cave at 0x1e32000 (540 bytes compiled with pi32v2 clang -Os).
 */

#include <stdint.h>

#ifdef HOST_TEST
#include "host_test_env.h"
#else
#define BASE       0x69b0u
#define USB_MODE   (*(volatile uint32_t *)(BASE + 0x0ccu))
#define ACT_MODE   (*(volatile uint32_t *)(BASE + 0x0c8u))
#define USB_READY  (*(volatile uint8_t  *)(BASE + 0x0e0u))
#define RAD_MODE   (*(volatile uint32_t *)(BASE + 0x108u))
#define SAV_MODE   (*(volatile uint32_t *)(BASE + 0x114u))
#define TX_FLAGS   (*(volatile uint8_t  *)(BASE + 0x01au))
#define MODE_ACT   (*(volatile uint8_t  *)(BASE + 0x003u))
#define BATT_BYTE  (*(volatile uint8_t  *)(BASE + 0x3bau))
#define IMU_PTR    ((const volatile uint8_t *)(BASE + 0x3c4u))
#define RX_BTN     (*(volatile uint32_t *)(BASE + 0x3d8u))

/* Unused zeroed BSS gap inside 0x69b0 struct (+0x514..+0x52b) */
#define HK_COMBO   (*(volatile uint32_t *)(BASE + 0x514u))
#define HK_COUNT   (*(volatile uint32_t *)(BASE + 0x518u))

typedef void (*orig_pack_fn)(uint32_t btn, uint32_t sticks, uint32_t trig, uint8_t *dst);
typedef void (*usb_send_fn)(uint32_t usb_id, uint8_t *buf, uint32_t len);
typedef void (*post_key_fn)(uint32_t ev);
typedef void (*nvm_save_fn)(uint32_t mode);

#define orig_pack  ((orig_pack_fn)0x1e05842u)
#define usb_send   ((usb_send_fn)0x1e02446u)
#define post_key   ((post_key_fn)0x1e0076au)
#define nvm_save   ((nvm_save_fn)0x1e0b13cu)
#define SEC_PACK   __attribute__((section(".text.pack"), noinline))
#define SEC_SEND   __attribute__((section(".text.send"), noinline))
#define SEC_HOTKEY __attribute__((section(".text.hotkey"), noinline))
#endif

SEC_PACK
void dinput_pack_hook(uint32_t btn, uint32_t sticks, uint32_t trig, uint8_t *dst) {
    if (USB_MODE != 1u) {
        orig_pack(btn, sticks, trig, dst);
        return;
    }

    if (USB_READY != 0u && ACT_MODE == 1u && RAD_MODE == 1u) {
        RAD_MODE = 0u;
        TX_FLAGS |= 0x04u;
        post_key(0x0fu);
    }

    uint8_t hat = 8u;
    uint32_t d = btn & 0x3c0u;
    if      (d == 0x200u) hat = 0u;
    else if (d == 0x240u) hat = 1u;
    else if (d == 0x040u) hat = 2u;
    else if (d == 0x140u) hat = 3u;
    else if (d == 0x100u) hat = 4u;
    else if (d == 0x180u) hat = 5u;
    else if (d == 0x080u) hat = 6u;
    else if (d == 0x280u) hat = 7u;

    dst[0] = hat;
    dst[1] = (uint8_t)(sticks);
    dst[2] = (uint8_t)(sticks >> 8);
    dst[3] = (uint8_t)(sticks >> 16);
    dst[4] = (uint8_t)(sticks >> 24);
    dst[5] = (uint8_t)(trig);       /* wire data[6] = RT */
    dst[6] = (uint8_t)(trig >> 8);  /* wire data[7] = LT */

    uint8_t b0 = 0u, b1 = 0u, b2 = 0u;
    if (btn & 0x00002000u) b0 |= 0x01u; /* A */
    if (btn & 0x00001000u) b0 |= 0x02u; /* B */
    if (btn & 0x04000000u) b0 |= 0x04u; /* PR (wire data[8] bit 2) */
    if (btn & 0x00000010u) b0 |= 0x08u; /* X */
    if (btn & 0x00000020u) b0 |= 0x10u; /* Y */
    if (btn & 0x02000000u) b0 |= 0x20u; /* PL (wire data[8] bit 5) */
    if (btn & 0x00000400u) b0 |= 0x40u; /* LB */
    if (btn & 0x00000800u) b0 |= 0x80u; /* RB */

    if (btn & 0x00004000u) b1 |= 0x01u; /* LT */
    if (btn & 0x00008000u) b1 |= 0x02u; /* RT */
    if (btn & 0x00000008u) b1 |= 0x04u; /* SELECT */
    if (btn & 0x00000001u) b1 |= 0x08u; /* START */
    if (btn & 0x00020000u) b1 |= 0x10u; /* HOME */
    if (btn & 0x00000002u) b1 |= 0x20u; /* L3 */
    if (btn & 0x00000004u) b1 |= 0x40u; /* R3 */

    if (btn & 0x00200000u) b2 |= 0x01u; /* L4 (wire data[10] bit 0) */
    if (btn & 0x40000000u) b2 |= 0x02u; /* R4 (wire data[10] bit 1) */

    dst[7]  = b0;
    dst[8]  = b1;
    dst[9]  = b2;
    dst[10] = 0u;
    dst[11] = 0u;
    dst[12] = 0u;
    dst[13] = BATT_BYTE;

    const volatile uint8_t *imu = IMU_PTR;
    for (uint32_t i = 0; i < 12u; i++) {
        dst[14u + i] = imu[i];
    }
    for (uint32_t i = 26u; i < 33u; i++) {
        dst[i] = 0u;
    }
}

SEC_SEND
void dinput_send_hook(uint32_t usb_id, uint8_t *buf, uint32_t len) {
    if (USB_MODE == 1u) {
        len = 34u;
    }
    usb_send(usb_id, buf, len);
}

SEC_HOTKEY
void hotkey_hook(uint32_t ev) {
    uint32_t combo = RX_BTN & 0x3039u;
    if (combo == 0x1008u || combo == 0x0018u || combo == 0x0028u || combo == 0x0009u) {
        if (HK_COMBO == combo) {
            uint32_t cnt = HK_COUNT;
            if (cnt != 0xFFFFFFFFu) {
                cnt += 1u;
                HK_COUNT = cnt;
                if (cnt >= 220u) {
                    HK_COUNT = 0xFFFFFFFFu;
                    uint32_t mode = 0u;           /* SELECT+Y -> Switch */
                    if      (combo == 0x1008u) mode = 1u; /* SELECT+B -> D-Input */
                    else if (combo == 0x0018u) mode = 3u; /* SELECT+X -> XInput */
                    else if (combo == 0x0009u) mode = 6u; /* SELECT+START -> Default */
                    nvm_save(mode);
                    SAV_MODE = mode;
                    uint32_t m = (mode == 6u) ? 0u : mode;
                    ACT_MODE = m;
                    RAD_MODE = m;
                    TX_FLAGS |= 0x04u;
                    MODE_ACT = 1u;
                    post_key(0x0fu);
                    post_key(0x19u);
                    return;
                }
            }
        } else {
            HK_COMBO = combo;
            HK_COUNT = 0u;
        }
    } else {
        HK_COUNT = 0u;
    }
    post_key(ev);
}
