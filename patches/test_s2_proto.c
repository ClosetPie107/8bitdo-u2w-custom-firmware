/*
 * Host unit test for Switch 2 Pro Controller GATT table, AES-128-ECB golden
 * pairing vector, command framing, flash reads, pending-response retry (H5),
 * LTK callback, and 63-byte input report packing (H2/H3/H4/M3/L2).
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef HOST_TEST
#define HOST_TEST
#endif
#include "s2_state.h"

s2_state_t g_s2_test_state;

static uint16_t last_noti_handle;
static uint8_t  last_noti_buf[128];
static int      last_noti_len;
static int      noti_fail_count;
static uint8_t  mock_csa2_remap[38];
static int      ltk_reply_called;
static uint8_t  ltk_replied[16];
static int      ltk_neg_called;
static uint8_t  last_adv_buf[31];
static unsigned adv_busy_once;
static uint32_t mock_btn;
static uint32_t mock_ls;
static uint32_t mock_rs;
static uint16_t mock_trig;
static uint8_t  mock_bat = 100;
static int16_t  mock_imu[6];
static uint32_t mock_sys_tick;
static uint8_t mock_rumble[2];
static unsigned mock_rumble_calls;
static uint8_t mock_bond_flash[2][32];
#ifdef S2_BOND_DIAGNOSTICS
static uint8_t mock_diag_flash[32];
#endif
static int mock_flash_erases;
static int mock_flash_writes;
static int mock_interrupt_write;

static uint8_t *mock_flash_record(uint32_t addr) {
#ifdef S2_BOND_DIAGNOSTICS
    if (addr >= 0x2b100u && addr < 0x2b100u + sizeof(mock_diag_flash)) {
        return &mock_diag_flash[addr - 0x2b100u];
    }
#endif
    assert(addr == 0x0002b000u || addr == 0x0002c000u);
    return mock_bond_flash[addr == 0x0002c000u];
}

void host_flash_read(uint32_t addr, uint32_t len, uint8_t *buf) {
    assert(len <= sizeof(mock_bond_flash[0]));
    memcpy(buf, mock_flash_record(addr), len);
}

void host_flash_write(uint32_t addr, uint32_t len, const uint8_t *buf) {
    assert(len <= sizeof(mock_bond_flash[0]));
    uint8_t *dest = mock_flash_record(addr);
    if (mock_interrupt_write) len /= 2u;
    for (uint32_t i = 0; i < len; i++) dest[i] &= buf[i];
    mock_flash_writes++;
}

void host_flash_erase(uint32_t addr) {
    memset(mock_flash_record(addr), 0xff, sizeof(mock_bond_flash[0]));
    mock_flash_erases++;
}

const uint8_t *host_csa2_remap_table(void) {
    return mock_csa2_remap;
}

int host_gatt_notify(uint16_t conn_h, uint16_t att_h, const uint8_t *p, int len) {
    assert(conn_h == BLS_CONN_HANDLE);
    assert(len <= (int)sizeof(last_noti_buf));
    if (noti_fail_count > 0) {
        noti_fail_count--;
        return 0x81; /* TX FIFO full */
    }
    last_noti_handle = att_h;
    last_noti_len = len;
    memcpy(last_noti_buf, p, (size_t)len);
    return 0;
}

int host_ll_ltk_reply(uint16_t conn_h, const uint8_t *ltk) {
    assert(conn_h == BLS_CONN_HANDLE);
    ltk_reply_called++;
    memcpy(ltk_replied, ltk, 16);
    return 0;
}

int host_ll_ltk_neg_reply(uint16_t conn_h) {
    assert(conn_h == BLS_CONN_HANDLE);
    ltk_neg_called++;
    return 0;
}

int host_set_adv_data(const uint8_t *p, uint8_t len) {
    assert(len == 31);
    if (adv_busy_once) {
        adv_busy_once = 0;
        return 12;
    }
    memcpy(last_adv_buf, p, 31);
    return 0;
}

void host_get_local_bdaddr(uint8_t out[6]) {
    const uint8_t mac[6] = { 0x09, 0xb6, 0xbc, 0xd8, 0x17, 0xe4 };
    memcpy(out, mac, 6);
}

uint32_t host_get_buttons(void) { return mock_btn; }
uint32_t host_get_lstick_raw(void) { return mock_ls; }
uint32_t host_get_rstick_raw(void) { return mock_rs; }
uint16_t host_get_triggers_raw(void) { return mock_trig; }
uint8_t  host_get_battery_pct(void) { return mock_bat; }
void host_get_imu(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz) {
    *ax = mock_imu[0]; *ay = mock_imu[1]; *az = mock_imu[2];
    *gx = mock_imu[3]; *gy = mock_imu[4]; *gz = mock_imu[5];
}
uint32_t host_get_system_tick(void) { return mock_sys_tick; }

static uint32_t test_get_bits(const uint8_t *buf, unsigned start, unsigned width) {
    uint32_t w = (uint32_t)buf[start >> 3]
               | ((uint32_t)buf[(start >> 3) + 1] << 8)
               | ((uint32_t)buf[(start >> 3) + 2] << 16)
               | ((uint32_t)buf[(start >> 3) + 3] << 24);
    return (w >> (start & 7u)) & ((1u << width) - 1u);
}

static int32_t test_sext(uint32_t v, unsigned width) {
    uint32_t sign = 1u << (width - 1u);
    return (v & sign) ? (int32_t)(v | ~(sign - 1u)) : (int32_t)v;
}

void host_set_rumble(uint8_t left, uint8_t right, uint8_t duration) {
    assert(duration > 0u && duration <= 100u);
    mock_rumble[0] = left;
    mock_rumble[1] = right;
    mock_rumble_calls++;
}

static void send_vib_cmd(uint8_t cmd, uint8_t sub, const uint8_t *payload, uint16_t plen) {
    uint8_t pkt[128];
    memset(pkt, 0, sizeof(pkt));
    pkt[33 + 0] = cmd;
    pkt[33 + 1] = 0x91;
    pkt[33 + 2] = 0x01;
    pkt[33 + 3] = sub;
    if (plen > 0) {
        memcpy(&pkt[33 + 8], payload, plen);
    }
    uint8_t att[160];
    memset(att, 0, sizeof(att));
    uint16_t val_len = (uint16_t)(33 + 8 + plen);
    uint16_t l2cap_len = (uint16_t)(3 + val_len);
    att[2] = (uint8_t)(l2cap_len & 0xff);
    att[3] = (uint8_t)(l2cap_len >> 8);
    memcpy(&att[9], pkt, val_len);
    last_noti_len = -1;
    s2_vib_cmd_write_cb(att);
}

int main(void) {
    memset(mock_bond_flash, 0xff, sizeof(mock_bond_flash));

    /* 1. The last byte of GATT RAM is reserved for the stock BT latch. */
    assert(sizeof(s2_state_t) <= 111);
    printf("PASS: sizeof(s2_state_t) = %zu bytes (<= 111)\n", sizeof(s2_state_t));

    /* 2. Verify GATT handle map & independent CCCDs */
    assert(s2_gatt_table[0].attNum == 0x3b);
    assert(s2_gatt_table[0x01].attNum == 7);
    assert(s2_gatt_table[0x08].attNum == 43);
    assert(s2_gatt_table[0x33].attNum == 5);
    assert(s2_gatt_table[0x38].attNum == 4);
    assert(s2_gatt_table[S2_H_COMMAND].w == s2_cmd_write_cb);
    assert(s2_gatt_table[S2_H_VIB_COMMAND].w == s2_vib_cmd_write_cb);
    assert(s2_gatt_table[0x23].pAttrValue != s2_gatt_table[0x27].pAttrValue);
    assert(s2_gatt_table[0x27].pAttrValue != s2_gatt_table[0x2f].pAttrValue);
    assert(s2_gatt_table[0x2f].pAttrValue != s2_gatt_table[0x3b].pAttrValue);
    printf("PASS: GATT handle table verified (0x0001..0x003b)\n");

    /* 2b. Verify C CSA #2 channel selector (BLE 5.0 Vol 6 Part C §3.1) & PHY resolver */
    {
        const uint8_t all37[5] = { 0xff, 0xff, 0xff, 0xff, 0x1f };
        const uint8_t exp1[4] = { 25, 20, 6, 21 };
        for (uint16_t c = 0; c < 4; c++) {
            assert(csa2_select_channel(all37, c, 0x305fu) == exp1[c]);
        }
        const uint8_t ch9[5] = { 0x00, 0x06, 0xe0, 0x00, 0x1e };
        const uint8_t remap9[9] = { 9, 10, 21, 22, 23, 33, 34, 35, 36 };
        memset(mock_csa2_remap, 0, sizeof(mock_csa2_remap));
        memcpy(mock_csa2_remap, remap9, sizeof(remap9));
        mock_csa2_remap[37] = sizeof(remap9);
        const uint8_t exp2[4] = { 35, 9, 33, 21 };
        for (uint16_t c = 0; c < 4; c++) {
            assert(csa2_select_channel(ch9, c, 0x305fu) == exp2[c]);
        }
        assert(s2_resolve_phy_update(1u, 2u, 2u) == 2u);
        assert(s2_resolve_phy_update(2u, 1u, 1u) == 1u);
        assert(s2_resolve_phy_update(1u, 0u, 0u) == 0u);
        assert(s2_resolve_phy_update(1u, 1u, 1u) == 0u);
        assert(s2_resolve_phy_update(1u, 2u, 1u) == 0u);
        assert(s2_resolve_phy_update(1u, 1u, 2u) == 0u);
        assert(s2_resolve_phy_update(1u, 2u, 0u) == 0u);
        assert(s2_resolve_phy_update(1u, 0u, 2u) == 0u);
        printf("PASS: C CSA #2 selector and symmetric PHY resolver verified!\n");
    }

    /* 3. Verify Switch 2 golden pairing vector (A1, B1, A2 -> LTK, B2) + H5 retry */
    memset(&g_s2_test_state, 0, sizeof(g_s2_test_state));

    const uint8_t addr_req[8] = { 0x00, 0x01, 0x5f, 0x11, 0x85, 0xeb, 0xf1, 0x48 };
    send_vib_cmd(0x15, 0x01, addr_req, sizeof(addr_req));
    assert(last_noti_handle == S2_H_CMD_RESP2 && last_noti_len == 14 + 8 + 9);

    const uint8_t a1_req[17] = {
        0x00,
        0x35, 0x03, 0xe9, 0x29, 0x82, 0x87, 0x71, 0x24,
        0xbe, 0xa8, 0x0c, 0x66, 0x46, 0x15, 0x83, 0x4b
    };
    const uint8_t expected_ltk[16] = {
        0x69, 0xf5, 0x07, 0x50, 0xae, 0x58, 0x74, 0xc5,
        0x04, 0x83, 0x6f, 0x43, 0x82, 0x0f, 0xdc, 0x5b
    };
    send_vib_cmd(0x15, 0x04, a1_req, sizeof(a1_req));
    assert(memcmp(g_s2_test_state.scratch.pair.pending_ltk, expected_ltk, 16) == 0);

    const uint8_t a2_req[17] = {
        0x00,
        0x6f, 0xc6, 0xdf, 0x8a, 0xd8, 0xfe, 0xdf, 0x15,
        0xbb, 0x8c, 0x15, 0xe9, 0x1f, 0x32, 0x05, 0x44
    };
    const uint8_t expected_b2[16] = {
        0x13, 0x4c, 0x97, 0xf5, 0x11, 0xb9, 0xb6, 0xdd,
        0x4d, 0x86, 0xfd, 0x40, 0xf5, 0x36, 0xe9, 0xed
    };
    /* Simulate TX FIFO busy on 0x15/02 -> must queue and flush in main loop */
    noti_fail_count = 1;
    send_vib_cmd(0x15, 0x02, a2_req, sizeof(a2_req));
    assert(last_noti_len == -1 && g_s2_test_state.pending_rsp_valid == 1);
    s2_send_input_report();
    assert(g_s2_test_state.pending_rsp_valid == 0);
    assert(last_noti_len == 14 + 8 + 17 && last_noti_buf[22] == 0x01);
    assert(memcmp(&last_noti_buf[23], expected_b2, 16) == 0);
    printf("PASS: Golden pairing vector + pending-response retry verified!\n");

    noti_fail_count = 1;
    send_vib_cmd(0x15, 0x03, NULL, 0);
    assert(last_noti_len == -1 && g_s2_test_state.have_ltk == (S2_BOND_VALID | S2_BOND_DIRTY));
    s2_service_ble(0u, 1u, 0u, 0u);
    assert(mock_flash_writes == 0);
    s2_send_input_report();
    assert(last_noti_len == 14 + 8 + 1 && (g_s2_test_state.have_ltk & S2_BOND_READY));
    s2_ltk_req_cb(BLS_CONN_HANDLE, NULL, 0);
    assert(ltk_reply_called == 1 && memcmp(ltk_replied, expected_ltk, 16) == 0);

    /* 4. Verify 0x02 flash read (64B @ 0x13000) WITH retry & preserve-first on consecutive busy */
    const uint8_t fl_req[8] = { 0x40, 0x7e, 0x00, 0x00, 0x00, 0x30, 0x01, 0x00 };
    noti_fail_count = 1;
    send_vib_cmd(0x02, 0x04, fl_req, sizeof(fl_req));
    assert(last_noti_len == -1 && g_s2_test_state.pending_rsp_valid == 2);
    s2_send_input_report();
    assert(g_s2_test_state.pending_rsp_valid == 0);
    assert(last_noti_len == 14 + 8 + 8 + 64);
    assert(last_noti_buf[22] == 0x40 && last_noti_buf[30] == 0x01 && last_noti_buf[32] == 'H');
    /* Consecutive busy burst: both 0x11/03 and 0x02 flash read are delivered in FIFO order. */
    noti_fail_count = 2;
    send_vib_cmd(0x11, 0x03, NULL, 0);
    assert(last_noti_len == -1 && g_s2_test_state.pending_rsp_valid == 3);
    send_vib_cmd(0x02, 0x04, fl_req, sizeof(fl_req));
    assert(last_noti_len == -1 && g_s2_test_state.pending_rsp_valid == 3 && g_s2_test_state.pending_q.valid == 2);
    s2_send_input_report();
    assert(last_noti_len == 14 + 8 + 29 && last_noti_buf[14] == 0x11u && last_noti_buf[17] == 0x03u);
    assert(g_s2_test_state.pending_rsp_valid == 2 && g_s2_test_state.pending_q.valid == 0);
    s2_send_input_report();
    assert(g_s2_test_state.pending_rsp_valid == 0 && last_noti_len == 14 + 8 + 8 + 64);
    assert(last_noti_buf[14] == 0x02u && last_noti_buf[32] == 'H');
    printf("PASS: 0x02 flash read + 2-slot FIFO retry verified!\n");

    /* 5. Verify L2 incoming length bound (l2cap_len > 162 ignored) */
    uint8_t bad_att[16] = { 0, 0, 200, 0 };
    last_noti_len = -1;
    s2_vib_cmd_write_cb(bad_att);
    assert(last_noti_len == -1);
    printf("PASS: L2 oversized l2cap_len guard verified!\n");

    /* 6. Verify H2 (D-pad), H3 (stick X=byte1 inv, Y=byte3 inv), H4 (triggers), M3 (Capture/GL/GR) */
    g_s2_test_state.cccd_pro2[0] = 0x01;
    g_s2_test_state.enabled_features = 0x2f;
    mock_btn = (1u << 12) | (1u << 9) | (1u << 6) | (1u << 16);
    /* LS: X=0x00 (byte 1 -> inverted to 0xff -> 0xfff), Y=0xff (byte 3 -> inverted to 0x00 -> 0x000) */
    mock_ls = (0x00u << 8) | (0xffu << 24);
    mock_rs = (0x80u << 8) | (0x80u << 24);
    mock_trig = (0x80u << 8) | 0x00u; /* High byte > 0x4d -> ZR only, not ZL */
    s2_send_input_report();
    assert(last_noti_handle == S2_H_PRO2_INPUT && last_noti_len == 63);
    assert(last_noti_buf[2] == (0x02u | 0x20u)); /* A + ZR */
    assert(last_noti_buf[3] == (0x08u | 0x02u)); /* Up (bit 9) + Right (bit 6), no ZL */
    assert(last_noti_buf[4] == 0x02u); /* Capture */
    /* LS packed 12-bit: xv=0xfff, yv=0x000 -> [0xff, 0x0f, 0x00] */
    assert(last_noti_buf[5] == 0xff && last_noti_buf[6] == 0x0f && last_noti_buf[7] == 0x00);
    mock_btn = 1u << 26;
    s2_send_input_report();
    assert(last_noti_buf[4] == 0x08u); /* Left paddle sends GL alone. */
    mock_btn = 1u << 25;
    s2_send_input_report();
    assert(last_noti_buf[4] == 0x04u); /* Right paddle sends GR alone. */
    printf("PASS: H2/H3/H4/M3 input report mapping verified!\n");

    /* 6b. Verify 40-byte (0x28, hdr=0xf0) motion encoding and quaternion integration */
    mock_imu[0] = -526; mock_imu[1] = -100; mock_imu[2] = 4120; /* u2w accel */
    mock_imu[3] = 200;  mock_imu[4] = -300; mock_imu[5] = 400;  /* u2w gyro */
    mock_sys_tick = 160000u; /* 10 ms */
    g_s2_test_state.scratch.imu.valid = 0u;
    s2_send_input_report();
    assert(last_noti_buf[0x0e] == 0x28u);
    assert(test_get_bits(last_noti_buf, 140u, 8u) == 0xf0u);
    assert(test_sext(test_get_bits(last_noti_buf, 220u, 14u), 14u) == 100);  /* s2_ax = -u_ay */
    assert(test_sext(test_get_bits(last_noti_buf, 234u, 14u), 14u) == -526); /* s2_ay = +u_ax */
    assert(test_sext(test_get_bits(last_noti_buf, 248u, 14u), 14u) == 4120); /* s2_az = +u_az */
    assert(test_sext(test_get_bits(last_noti_buf, 263u, 15u), 15u) == 600);  /* s2_gx = -2*u_gy */
    assert(test_sext(test_get_bits(last_noti_buf, 279u, 15u), 15u) == 400);  /* s2_gy = +2*u_gx */
    assert(test_sext(test_get_bits(last_noti_buf, 295u, 15u), 15u) == 800);  /* s2_gz = +2*u_gz */
    assert(test_sext(test_get_bits(last_noti_buf, 310u, 13u), 13u) * 2 == 100);
    assert(test_sext(test_get_bits(last_noti_buf, 397u, 14u), 14u) == 100);
    assert(test_get_bits(last_noti_buf, 154u, 2u) == 0u); /* identity omits w (q3) */
    /* Advance 5 ms (80000 ticks @ 16 MHz) with pure +Z yaw rotation */
    mock_imu[3] = 0; mock_imu[4] = 0; mock_imu[5] = 1638; /* ~100 deg/s yaw */
    mock_sys_tick += 80000u;
    s2_send_input_report();
    assert(test_get_bits(last_noti_buf, 132u, 8u) >= 4u && test_get_bits(last_noti_buf, 132u, 8u) <= 6u);
    assert(g_s2_test_state.scratch.imu.q[3] > 3000 && g_s2_test_state.scratch.imu.q[3] < 6000);
    /* Verify 32-bit system timer rollover produces a normal 4..5 tick delta, not 255. */
    mock_sys_tick = 0xffff0000u;
    g_s2_test_state.scratch.imu.valid = 0u;
    s2_send_input_report();
    mock_sys_tick = (uint32_t)(0xffff0000u + 80000u); /* wraps past 2^32 */
    s2_send_input_report();
    {
        uint32_t dt_wrap = test_get_bits(last_noti_buf, 132u, 8u);
        assert(dt_wrap >= 4u && dt_wrap <= 5u);
    }
    printf("PASS: 40-byte Switch 2 motion packing, quaternion, and timer rollover verified!\n");

    /* Exercise both BLE rumble paths with independent left and right levels. */
    uint8_t rumble_att[9 + 42] = {0};
    rumble_att[2] = 3 + 33;
    uint8_t *rumble = rumble_att + 9;
    rumble[1] = 0x50;
    rumble[4] = 0x08;   /* Left low-band amplitude 512 -> 255 */
    rumble[17] = 0x50;
    rumble[22] = 0x40;  /* Right high-band amplitude 256 -> 128 */
    s2_gatt_table[0x0012].w(rumble_att);
    assert(mock_rumble_calls == 1 && mock_rumble[0] == 255 && mock_rumble[1] == 128);
    /* A following silence packet must not erase the short pulse before the 10 ms motor tick. */
    uint8_t silent_att[9 + 33] = {0};
    silent_att[2] = 3 + 33;
    silent_att[9 + 1] = 0x50;
    silent_att[9 + 17] = 0x50;
    s2_gatt_table[0x0012].w(silent_att);
    assert(mock_rumble_calls == 1 && mock_rumble[0] == 255 && mock_rumble[1] == 128);
    rumble_att[2] = 3 + 32;
    s2_gatt_table[0x0012].w(rumble_att);
    assert(mock_rumble_calls == 1);
    rumble_att[2] = 3 + 42;
    rumble[33] = 0x09;
    rumble[34] = 0x91;
    rumble[35] = 0x01;
    rumble[36] = 0x07;
    rumble[41] = 0x03;
    s2_gatt_table[S2_H_VIB_COMMAND].w(rumble_att);
    assert(mock_rumble_calls == 2 && last_noti_handle == S2_H_CMD_RESP2);
    /* Command 0x0a/02 built-in vibration presets (connect buzz, Find Controllers, stop). */
    uint8_t vib_sample = 3u;
    send_vib_cmd(0x0a, 0x02, &vib_sample, 1);
    assert(mock_rumble_calls == 3 && mock_rumble[0] == 110 && mock_rumble[1] == 110);
    vib_sample = 0u;
    send_vib_cmd(0x0a, 0x02, &vib_sample, 1);
    assert(mock_rumble_calls == 4 && mock_rumble[0] == 0 && mock_rumble[1] == 0);
    /* Re-arm rumble_active so disconnect test verifies rumble stop. */
    vib_sample = 2u;
    send_vib_cmd(0x0a, 0x02, &vib_sample, 1);
    assert(mock_rumble_calls == 5 && mock_rumble[0] == 255 && mock_rumble[1] == 255);
    printf("PASS: Rumble channels and presets verified!\n");

    s2_service_ble(1u, 0u, 0u, 0u);
    assert(mock_flash_writes == 0);
#ifdef S2_BOND_DIAGNOSTICS
    assert((g_s2_test_state.bond_diagnostics & 0x03u) == 0x03u);
    assert((g_s2_test_state.bond_diagnostics & 0x78u) == 0u);
#endif
    s2_service_ble(2u, 1u, 0u, 0u);
#ifdef S2_BOND_DIAGNOSTICS
    assert((g_s2_test_state.bond_diagnostics & 0x7bu) == 0x73u);
#endif

    /* 7. Verify reconnect ADV_IND, CCCD full clear, and flash bond persistence + console wake */
    g_s2_test_state.cccd_pro2[1] = 0x01u;
    g_s2_test_state.cccd_unk1[0] = 0x01u;
    g_s2_test_state.cccd_svc[1]  = 0x02u;
    s2_on_ble_disconnect(10u, 0u);
    assert(g_s2_test_state.cccd_pro2[0] == 0u && g_s2_test_state.cccd_pro2[1] == 0u);
    assert(g_s2_test_state.cccd_unk1[0] == 0u && g_s2_test_state.cccd_svc[1] == 0u);
    assert(mock_rumble_calls == 6 && mock_rumble[0] == 0 && mock_rumble[1] == 0);
    assert(last_adv_buf[5] == 0x53 && last_adv_buf[6] == 0x05);
    assert(last_adv_buf[16] == 0x00u); /* disconnect uses plain reconnect, not wake */
    assert(memcmp(&last_adv_buf[17], &addr_req[2], 6) == 0);
    assert(mock_flash_erases == 0 && mock_flash_writes == 1);

    /* Ordinary reconnect must NOT allow replacing an existing bond outside Pair window. */
    {
        uint8_t spoof_addr[8] = { 0x00, 0x01, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };
        s2_on_ble_connect();
        send_vib_cmd(0x15, 0x01, spoof_addr, sizeof(spoof_addr));
        assert(last_noti_len == -1 && g_s2_test_state.pairing_stage == 0u);
        assert(memcmp(g_s2_test_state.host_addr, &addr_req[2], 6) == 0);
        /* Opening the Pair window allows pairing; timeout closes it again. */
        s2_pairing_advertising(100u);
        assert(g_s2_test_state.flags & S2_FLAG_PAIRING_OPEN);
        s2_service_ble(15000u, 0u, 0u, 0u);
        assert((g_s2_test_state.flags & S2_FLAG_PAIRING_OPEN) == 0u);
        send_vib_cmd(0x15, 0x01, spoof_addr, sizeof(spoof_addr));
        assert(last_noti_len == -1);
    }

    /* Simulate full power-off (RAM wiped) and reboot with s2_bond_load() */
    uint8_t saved_ltk[16];
    memcpy(saved_ltk, g_s2_test_state.ltk, 16);
    memset(&g_s2_test_state, 0, sizeof(g_s2_test_state));
    s2_bond_load();
    assert(g_s2_test_state.have_ltk == 0x01u);
    assert(memcmp(g_s2_test_state.ltk, saved_ltk, 16) == 0);
    assert(memcmp(g_s2_test_state.host_addr, &addr_req[2], 6) == 0);

    /* Wake ADV_IND, matching the Pro 2 capture with the test console address. */
    uint8_t adv[31];
    const uint8_t expected_wake[31] = {
        0x02, 0x01, 0x06, 0x1b, 0xff, 0x53, 0x05, 0x01,
        0x00, 0x03, 0x7e, 0x05, 0x69, 0x20, 0x00, 0x01,
        0x81, 0x5f, 0x11, 0x85, 0xeb, 0xf1, 0x48, 0x0f,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    s2_build_adv_data(adv, S2_ADV_WAKE_FLAG);
    assert(memcmp(adv, expected_wake, sizeof(adv)) == 0);

    /* Keep wake enabled during a transient link drop, including a timer wrap. */
    s2_startup_advertising(65530u, 0u);
    assert(last_adv_buf[16] == S2_ADV_WAKE_FLAG);
    s2_on_ble_connect();
    s2_service_ble(4u, 1u, 0u, 0u);
    adv_busy_once = 1;
    s2_on_ble_disconnect(5u, 0u);
    assert(g_s2_test_state.flags & S2_FLAG_ADV_PENDING);
    s2_service_ble(10u, 0u, 0u, 0u);
    assert((g_s2_test_state.flags & S2_FLAG_ADV_PENDING) == 0u && last_adv_buf[16] == S2_ADV_WAKE_FLAG);
    s2_service_ble(2000u, 0u, 0u, 0u);
    assert(last_adv_buf[16] == 0u);

    /* A secured link ends wake immediately. Later disconnects stay passive. */
    s2_startup_advertising(2100u, 0u);
    s2_service_ble(2110u, 1u, 1u, 0u);
    s2_on_ble_disconnect(2120u, 0u);
    assert(last_adv_buf[16] == 0u);

    /* Force-discovery ADV_IND (Pair button held even when bonded) */
    static const uint8_t zero6[6] = {0, 0, 0, 0, 0, 0};
    s2_build_adv_data(adv, S2_ADV_FORCE_DISCOVERY);
    assert(adv[16] == 0x00u);
    assert(memcmp(&adv[17], zero6, 6) == 0);
    s2_on_ble_disconnect(2500u, 1u);
    assert(memcmp(&last_adv_buf[17], zero6, 6) == 0);
    assert(g_s2_test_state.flags & S2_FLAG_PAIRING_OPEN);
    s2_startup_advertising(3000u, 0u);
    s2_pairing_advertising(3005u);
    assert((g_s2_test_state.flags & S2_FLAG_WAKE_ACTIVE) == 0u && last_adv_buf[16] == 0u);
    assert(g_s2_test_state.flags & S2_FLAG_PAIRING_OPEN);
    assert(memcmp(&last_adv_buf[17], zero6, 6) == 0);

    /* A partially written replacement leaves the previous bond readable. */
    g_s2_test_state.host_addr[0] ^= 1u;
    g_s2_test_state.have_ltk |= S2_BOND_DIRTY | S2_BOND_READY;
    mock_interrupt_write = 1;
    s2_service_ble(1u, 1u, 1u, 0u);
    assert(g_s2_test_state.have_ltk & S2_BOND_DIRTY);
    s2_service_ble(2u, 1u, 1u, 0u);
    assert(mock_flash_writes == 2);
    memset(&g_s2_test_state, 0, sizeof(g_s2_test_state));
    s2_bond_load();
    assert(memcmp(g_s2_test_state.host_addr, &addr_req[2], 6) == 0);

    /* A successful retry makes the newer record authoritative on boot. */
    mock_interrupt_write = 0;
    const uint8_t new_host_first_byte = g_s2_test_state.host_addr[0] ^ 1u;
    g_s2_test_state.host_addr[0] = new_host_first_byte;
    g_s2_test_state.have_ltk |= S2_BOND_DIRTY | S2_BOND_READY;
    s2_service_ble(3u, 1u, 1u, 0u);
    assert(g_s2_test_state.have_ltk == S2_BOND_VALID);
    assert(mock_flash_erases == 1 && mock_flash_writes == 3);
    memset(&g_s2_test_state, 0, sizeof(g_s2_test_state));
    s2_bond_load();
    assert(g_s2_test_state.host_addr[0] == new_host_first_byte);

    /* A duplicate commit leaves flash untouched. */
    s2_on_ble_connect();
    g_s2_test_state.have_ltk |= S2_BOND_DIRTY | S2_BOND_READY;
    s2_service_ble(4u, 1u, 1u, 0u);
    assert(g_s2_test_state.have_ltk == S2_BOND_VALID);
    assert(mock_flash_erases == 1 && mock_flash_writes == 3);

    /* Persisted BT/2.4G mode flags preserve the Switch 2 bond and vice versa. */
    uint8_t cfg_flags = S2_CFG_BT_STOCK | (1u << S2_CFG_24G_SHIFT); /* 8BitDo BLE + 2.4G DInput */
    s2_cfg_save_flags(cfg_flags);
    assert(s2_cfg_load() == cfg_flags);
    memset(&g_s2_test_state, 0, sizeof(g_s2_test_state));
    s2_bond_load();
    assert(g_s2_test_state.have_ltk == S2_BOND_VALID);
    assert(g_s2_test_state.host_addr[0] == new_host_first_byte);
    /* Saving a new bond preserves existing mode flags. */
    g_s2_test_state.host_addr[0] ^= 2u;
    g_s2_test_state.have_ltk |= S2_BOND_DIRTY | S2_BOND_READY;
    s2_service_ble(5u, 1u, 1u, 0u);
    assert(s2_cfg_load() == cfg_flags);

    /* Corrupt checksum -> s2_bond_load must reject */
    mock_bond_flash[0][28] ^= 0x01u;
    mock_bond_flash[1][28] ^= 0x01u;
    memset(&g_s2_test_state, 0, sizeof(g_s2_test_state));
    s2_bond_load();
    assert(g_s2_test_state.have_ltk == 0u);
    assert(s2_cfg_load() == 0u);

    /* Saving mode flags before any Switch 2 bond exists leaves have_ltk == 0. */
    memset(mock_bond_flash, 0xff, sizeof(mock_bond_flash));
    s2_cfg_save_flags(1u << S2_CFG_24G_SHIFT);
    assert(s2_cfg_load() == (1u << S2_CFG_24G_SHIFT));
    s2_bond_load();
    assert(g_s2_test_state.have_ltk == 0u);
    printf("PASS: Reconnect/Wake/Discovery ADV_IND + flash bond & mode persistence verified!\n");

#ifdef S2_BOND_DIAGNOSTICS
    memset(mock_diag_flash, 0xff, sizeof(mock_diag_flash));
    mock_diag_flash[0] = 0x01u;
    g_s2_test_state.bond_diagnostics = 0x0fu;
    g_s2_test_state.report_seq = 191u;
    int writes_before_diag = mock_flash_writes;
    s2_save_bond_diagnostics();
    assert(mock_diag_flash[1] == 0xffu);
    g_s2_test_state.report_seq++;
    s2_save_bond_diagnostics();
    s2_save_bond_diagnostics();
    assert(mock_diag_flash[0] == 0x01u && mock_diag_flash[1] == 0x0fu);
    assert(mock_diag_flash[2] == 0xffu && mock_flash_writes == writes_before_diag + 1);
#endif

    return 0;
}
