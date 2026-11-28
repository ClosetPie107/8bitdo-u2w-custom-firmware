#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "host_test_env.h"

/* Globals backing host_test_env.h */
uint32_t g_usb_mode;
uint32_t g_act_mode;
uint8_t  g_usb_ready;
uint32_t g_rad_mode;
uint32_t g_sav_mode;
uint8_t  g_tx_flags;
uint8_t  g_mode_act;
uint8_t  g_batt_byte;
uint8_t  g_imu_bytes[12];
uint32_t g_rx_btn;
uint32_t g_hk_combo;
uint32_t g_hk_count;

uint32_t g_ep_busy_mask;
uint32_t g_t0_tick_2ms;
uint32_t g_usb_dev_table[4];
uint32_t g_usb_hdl_ptr;
uint8_t  g_usb_state_byte;
uint32_t g_usb_con0_reg;
uint32_t g_usb_txcount_reg[8];
uint32_t g_icfg;
uint32_t g_ep_maxp[8];
uint8_t  g_ep_dma_buf[8][64];

/* Call recorders */
static int g_orig_pack_calls;
static int g_usb_send_calls;
static uint32_t g_usb_send_last_len;
static uint8_t g_usb_send_last_buf[64];
static int g_post_key_calls;
static uint32_t g_post_key_log[16];
static int g_nvm_save_calls;
static uint32_t g_nvm_save_last;

/* Mock TXCSR behavior (per-EP) */
static uint32_t g_txcsr_val[8];
static int g_txcsr_reads;
static int g_txcsr_writes;
static int g_txcsr_advance_tick_on_read;
static int g_txcsr_drop_usb_after_reads;
static uint32_t g_tx_packets_sent;
static uint8_t g_tx_stream[256];
static uint32_t g_tx_stream_len;

static void reset_env(void) {
    g_usb_mode = 0;
    g_act_mode = 0;
    g_usb_ready = 0;
    g_rad_mode = 0;
    g_sav_mode = 0;
    g_tx_flags = 0;
    g_mode_act = 0;
    g_batt_byte = 0;
    memset(g_imu_bytes, 0, sizeof(g_imu_bytes));
    g_rx_btn = 0;
    g_hk_combo = 0;
    g_hk_count = 0;

    g_ep_busy_mask = 0;
    g_t0_tick_2ms = 0;
    memset(g_usb_dev_table, 0, sizeof(g_usb_dev_table));
    g_usb_hdl_ptr = 1;
    g_usb_state_byte = 4;
    g_usb_con0_reg = 4;
    memset(g_usb_txcount_reg, 0, sizeof(g_usb_txcount_reg));
    g_icfg = 0;
    memset(g_ep_maxp, 0, sizeof(g_ep_maxp));
    memset(g_ep_dma_buf, 0, sizeof(g_ep_dma_buf));

    g_orig_pack_calls = 0;
    g_usb_send_calls = 0;
    g_usb_send_last_len = 0;
    memset(g_usb_send_last_buf, 0, sizeof(g_usb_send_last_buf));
    g_post_key_calls = 0;
    memset(g_post_key_log, 0, sizeof(g_post_key_log));
    g_nvm_save_calls = 0;
    g_nvm_save_last = 0;

    memset(g_txcsr_val, 0, sizeof(g_txcsr_val));
    g_txcsr_reads = 0;
    g_txcsr_writes = 0;
    g_txcsr_advance_tick_on_read = 0;
    g_txcsr_drop_usb_after_reads = 0;
    g_tx_packets_sent = 0;
    g_tx_stream_len = 0;
}

void orig_pack(uint32_t btn, uint32_t sticks, uint32_t trig, uint8_t *dst) {
    (void)btn;
    (void)sticks;
    (void)trig;
    (void)dst;
    g_orig_pack_calls++;
}

void usb_send(uint32_t usb_id, uint8_t *buf, uint32_t len) {
    (void)usb_id;
    g_usb_send_calls++;
    g_usb_send_last_len = len;
    assert(len <= sizeof(g_usb_send_last_buf));
    memcpy(g_usb_send_last_buf, buf, len);
}

void post_key(uint32_t ev) {
    if (g_post_key_calls < 16)
        g_post_key_log[g_post_key_calls] = ev;
    g_post_key_calls++;
}

void nvm_save(uint32_t mode) {
    g_nvm_save_calls++;
    g_nvm_save_last = mode;
}

uint32_t read_icfg(void) {
    return g_icfg;
}

uint32_t usb_read_txcsr(uint32_t ep) {
    g_txcsr_reads++;
    if (g_txcsr_advance_tick_on_read) {
        g_t0_tick_2ms = (g_t0_tick_2ms + 1u) % 500u;
    }
    if (g_txcsr_drop_usb_after_reads > 0 && g_txcsr_reads >= g_txcsr_drop_usb_after_reads) {
        /* Simulate mid-poll USB bus reset: USB_STATE drops to DEFAULT and TXCSR reads 0 */
        g_usb_state_byte = 2;
        return 0u;
    }
    return g_txcsr_val[ep & 7u];
}

void usb_write_txcsr(uint32_t ep, uint32_t val) {
    g_txcsr_writes++;
    if (val & 0x0001u) {
        uint32_t n = g_usb_txcount_reg[ep];
        assert(g_tx_stream_len + n <= sizeof(g_tx_stream));
        memcpy(g_tx_stream + g_tx_stream_len, g_ep_dma_buf[ep], n);
        g_tx_stream_len += n;
        g_tx_packets_sent++;
    }
    g_txcsr_val[ep & 7u] = val & ~0x0001u;
}

static void test_dinput_pack_and_send(void) {
    reset_env();
    /* Model the stock 34-byte on-wire buffer: wire[0] = Report ID 0x01, wire[1..33] = payload */
    uint8_t wire[34];
    memset(wire, 0xaa, sizeof(wire));
    wire[0] = 0x01u;

    /* Non-D-Input mode delegates to orig_pack */
    g_usb_mode = 3;
    dinput_pack_hook(0, 0, 0, &wire[1]);
    assert(g_orig_pack_calls == 1);

    /* D-Input mode packs 33-byte payload (+ Report ID = 34 B) and triggers IMU enable once */
    g_usb_mode = 1;
    g_act_mode = 1;
    g_rad_mode = 1;
    g_usb_ready = 1;
    g_batt_byte = 0x5a;
    for (uint8_t i = 0; i < 12; ++i)
        g_imu_bytes[i] = (uint8_t)(0x10u + i);

    uint32_t btn = 0x00000240u   /* Hat = 1 (NE) */
                 | 0x00002000u   /* A -> b0 bit 0 */
                 | 0x02000000u   /* PL -> b0 bit 5 (0x20) */
                 | 0x00000008u   /* SELECT -> b1 bit 2 */
                 | 0x00200000u   /* L4 -> b2 bit 0 (0x01) */
                 | 0x40000000u;  /* R4 -> b2 bit 1 (0x02) */
    dinput_pack_hook(btn, 0x44332211u, 0x0000aabb, &wire[1]);
    dinput_send_hook(0, wire, 12);

    assert(g_rad_mode == 0);
    assert((g_tx_flags & 0x04u) != 0);
    assert(g_post_key_calls == 1 && g_post_key_log[0] == 0x0fu);
    assert(g_usb_send_calls == 1 && g_usb_send_last_len == 34);

    assert(g_usb_send_last_buf[0] == 0x01u); /* Report ID */
    assert(g_usb_send_last_buf[1] == 1u);    /* Hat NE */
    assert(g_usb_send_last_buf[2] == 0x11u && g_usb_send_last_buf[3] == 0x22u);
    assert(g_usb_send_last_buf[4] == 0x33u && g_usb_send_last_buf[5] == 0x44u);
    assert(g_usb_send_last_buf[6] == 0xbbu && g_usb_send_last_buf[7] == 0xaau); /* RT=0xbb, LT=0xaa */
    assert(g_usb_send_last_buf[8] == (0x01u | 0x20u)); /* A + PL */
    assert(g_usb_send_last_buf[9] == 0x04u);
    assert(g_usb_send_last_buf[10] == 0x03u); /* L4 + R4 */
    assert(g_usb_send_last_buf[11] == 0u && g_usb_send_last_buf[12] == 0u && g_usb_send_last_buf[13] == 0u);
    assert(g_usb_send_last_buf[14] == 0x5au);
    for (uint8_t i = 0; i < 12; ++i)
        assert(g_usb_send_last_buf[15u + i] == (uint8_t)(0x10u + i));
    for (uint8_t i = 27; i < 34; ++i)
        assert(g_usb_send_last_buf[i] == 0u);

    /* Second call does not re-post 0x0f once RAD_MODE is 0 */
    dinput_pack_hook(0, 0, 0, &wire[1]);
    assert(g_post_key_calls == 1);
    assert(wire[1] == 8u); /* Neutral hat */

    /* Non-D-Input send keeps original length */
    g_usb_mode = 3;
    dinput_send_hook(0, wire, 20);
    assert(g_usb_send_calls == 2 && g_usb_send_last_len == 20);
}

static void test_hotkey_hook(void) {
    const struct {
        uint32_t combo;
        uint32_t exp_sav;
        uint32_t exp_act;
    } cases[] = {
        { 0x1008u, 1u, 1u }, /* SELECT + B -> D-Input */
        { 0x0018u, 3u, 3u }, /* SELECT + X -> XInput */
        { 0x0028u, 0u, 0u }, /* SELECT + Y -> Switch */
        { 0x0009u, 6u, 0u }, /* SELECT + START -> Default */
    };

    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
        reset_env();
        g_rx_btn = cases[c].combo;
        for (int i = 0; i < 220; ++i) {
            hotkey_hook(0x42u);
            assert(g_nvm_save_calls == 0);
        }
        assert(g_post_key_calls == 220);

        hotkey_hook(0x42u);
        assert(g_nvm_save_calls == 1 && g_nvm_save_last == cases[c].exp_sav);
        assert(g_sav_mode == cases[c].exp_sav);
        assert(g_act_mode == cases[c].exp_act);
        assert(g_rad_mode == cases[c].exp_act);
        assert(g_mode_act == 1u);
        assert((g_tx_flags & 0x04u) != 0);
        assert(g_hk_count == 0xFFFFFFFFu);

        /* Holding longer does not re-trigger */
        hotkey_hook(0x42u);
        assert(g_nvm_save_calls == 1);

        /* Releasing resets counter */
        g_rx_btn = 0;
        hotkey_hook(0x42u);
        assert(g_hk_count == 0u);
    }

    /* Undocumented / multi-button combos MUST NOT trigger a mode switch */
    const uint32_t invalid_combos[] = {
        0x2008u, /* SELECT + A */
        0x1018u, /* SELECT + B + X */
        0x1028u, /* SELECT + B + Y */
        0x0038u, /* SELECT + X + Y */
        0x0008u, /* SELECT alone */
    };
    for (size_t c = 0; c < sizeof(invalid_combos) / sizeof(invalid_combos[0]); ++c) {
        reset_env();
        g_rx_btn = invalid_combos[c];
        for (int i = 0; i < 300; ++i) {
            hotkey_hook(0x42u);
        }
        assert(g_nvm_save_calls == 0);
        assert(g_hk_count == 0u);
    }
}

static void test_usb_tx_fix(void) {
    uint8_t payload[70];
    for (size_t i = 0; i < sizeof(payload); ++i)
        payload[i] = (uint8_t)i;

    /* 1. Unconfigured USB state drops write and clears EP_BUSY_MASK */
    reset_env();
    g_usb_dev_table[0] = 1;
    g_ep_maxp[1] = 64;
    g_ep_busy_mask = 0x02u;
    g_usb_state_byte = 3; /* ADDRESS, not CONFIGURED */
    fix_usb_g_intr_write(0, 1, payload, 34);
    assert(g_ep_busy_mask == 0u);
    assert(g_tx_packets_sent == 0u);

    /* USB_CON0 bit 2 clear also drops write and clears EP_BUSY_MASK */
    g_usb_state_byte = 4;
    g_usb_con0_reg = 0;
    g_ep_busy_mask = 0x04u;
    fix_usb_g_intr_write(0, 1, payload, 34);
    assert(g_ep_busy_mask == 0u);
    assert(g_tx_packets_sent == 0u);

    /* 2. Normal multi-packet transfer (70 B across 64 B + 6 B) */
    reset_env();
    g_usb_dev_table[0] = 1;
    g_ep_maxp[1] = 64;
    g_txcsr_val[1] = 0;
    fix_usb_g_intr_write(0, 1, payload, sizeof(payload));
    assert(g_tx_packets_sent == 2u);
    assert(g_tx_stream_len == sizeof(payload));
    assert(memcmp(g_tx_stream, payload, sizeof(payload)) == 0);
    assert(g_ep_busy_mask == 0u);

    /* 3. ISR timeout with Timer0 wraparound (498 -> 3 = 5 ticks) sets EP_BUSY_MASK for EP1 only */
    reset_env();
    g_usb_dev_table[0] = 1;
    g_ep_maxp[1] = 64;
    g_ep_maxp[2] = 64;
    g_icfg = 0x48u; /* Inside Priority-3 ISR */
    g_t0_tick_2ms = 498u;
    g_txcsr_val[1] = 0x0001u; /* EP1 TXPKTRDY stuck */
    g_txcsr_val[2] = 0x0000u; /* EP2 idle */
    g_txcsr_advance_tick_on_read = 1;
    fix_usb_g_intr_write(0, 1, payload, 34);
    assert(g_tx_packets_sent == 0u);
    assert(g_txcsr_reads == 5);
    assert((g_ep_busy_mask & (1u << 1)) != 0);

    /* EP2 is unaffected by EP1 backoff */
    g_txcsr_advance_tick_on_read = 0;
    fix_usb_g_intr_write(0, 2, payload, 16);
    assert(g_tx_packets_sent == 1u);
    assert((g_ep_busy_mask & (1u << 1)) != 0);

    /* 4. Subsequent EP1 call while EP_BUSY_MASK set fast-drops on first TXCSR read */
    g_txcsr_reads = 0;
    fix_usb_g_intr_write(0, 1, payload, 34);
    assert(g_tx_packets_sent == 1u);
    assert(g_txcsr_reads == 1);

    /* 5. Once host drains EP1 FIFO (TXCSR=0), EP_BUSY_MASK clears and transfer succeeds */
    g_txcsr_val[1] = 0;
    fix_usb_g_intr_write(0, 1, payload, 34);
    assert(g_tx_packets_sent == 2u);
    assert((g_ep_busy_mask & (1u << 1)) == 0);

    /* 6. Mid-poll USB bus reset (TXCSR reads 0 while USB_STATE drops to DEFAULT) aborts cleanly */
    reset_env();
    g_usb_dev_table[0] = 1;
    g_ep_maxp[1] = 64;
    g_txcsr_val[1] = 0x0001u;
    g_txcsr_drop_usb_after_reads = 2;
    fix_usb_g_intr_write(0, 1, payload, 34);
    assert(g_tx_packets_sent == 0u);
    assert(g_ep_busy_mask == 0u);
}

int main(void) {
    test_dinput_pack_and_send();
    test_hotkey_hook();
    test_usb_tx_fix();
    puts("PASS: all U2BT dongle C host tests passed.");
    return 0;
}
