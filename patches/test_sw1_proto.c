/*
 * Host unit tests for Switch 1 Pro Controller (057e:2009) BLE HOGP GATT table,
 * advertising data, subcommand handling (0x02/0x03/0x10/0x30/0x31/0x40/0x48/0x50),
 * 4-slot reply FIFO retry, 0x3F simple HID, 0x30 full state + IMU frames,
 * Switch 1 HD rumble decoding, CSA #2 / PHY resolver, and flash slot persistence.
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
static int      noti_calls;
static int      noti_fail_count;
static uint8_t  mock_csa2_remap[38];
static uint32_t mock_btn;
static uint32_t mock_raw_btn;
static uint32_t mock_ls;
static uint32_t mock_rs;
static uint16_t mock_trig;
static uint8_t  mock_bat = 100;
static int16_t  mock_imu[6];
static uint8_t  mock_rumble[3]; /* left, right, duration */
static unsigned mock_rumble_calls;
static uint8_t  mock_bond_flash[2][108];
static int      mock_flash_erases;
static int      mock_flash_writes;
static int      mock_interrupt_write;

static uint8_t *mock_flash_record(uint32_t addr) {
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
    noti_calls++;
    return 0;
}

void host_get_local_bdaddr(uint8_t out[6]) {
    const uint8_t mac[6] = { 0x09, 0xb6, 0xbc, 0xd8, 0x17, 0xe4 };
    memcpy(out, mac, 6);
}

uint32_t host_get_buttons(void)      { return mock_btn; }
uint32_t host_get_raw_buttons(void)  { return mock_raw_btn; }
uint32_t host_get_lstick_raw(void)   { return mock_ls; }
uint32_t host_get_rstick_raw(void)   { return mock_rs; }
uint16_t host_get_triggers_raw(void) { return mock_trig; }
uint8_t  host_get_battery_pct(void)  { return mock_bat; }
void host_get_imu(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz) {
    *ax = mock_imu[0]; *ay = mock_imu[1]; *az = mock_imu[2];
    *gx = mock_imu[3]; *gy = mock_imu[4]; *gz = mock_imu[5];
}

void host_set_rumble(uint8_t left, uint8_t right, uint8_t duration) {
    assert(duration > 0u && duration <= 100u);
    mock_rumble[0] = left;
    mock_rumble[1] = right;
    mock_rumble[2] = duration;
    mock_rumble_calls++;
}

static void send_out01(uint8_t seq, const uint8_t rumble[8], uint8_t subcmd,
                       const uint8_t *args, uint16_t arg_len) {
    uint8_t att[80];
    memset(att, 0, sizeof(att));
    uint16_t val_len = (uint16_t)(10u + arg_len);
    uint16_t l2cap_len = (uint16_t)(3u + val_len);
    att[2] = (uint8_t)(l2cap_len & 0xffu);
    att[3] = (uint8_t)(l2cap_len >> 8);
    att[9] = seq & 0x0fu;
    if (rumble) {
        memcpy(&att[10], rumble, 8u);
    } else {
        const uint8_t neutral[8] = { 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
        memcpy(&att[10], neutral, 8u);
    }
    att[18] = subcmd;
    if (arg_len > 0u) {
        memcpy(&att[19], args, arg_len);
    }
    last_noti_len = -1;
    sw1_out01_write_cb(att);
}

static void send_out10(uint8_t seq, const uint8_t rumble[8]) {
    uint8_t att[24];
    memset(att, 0, sizeof(att));
    att[2] = (uint8_t)(3u + 9u);
    att[3] = 0u;
    att[9] = seq & 0x0fu;
    memcpy(&att[10], rumble, 8u);
    sw1_out10_write_cb(att);
}

static uint16_t unpack_stick12_x(const uint8_t *d) {
    return (uint16_t)(d[0] | ((d[1] & 0x0fu) << 8));
}

static uint16_t unpack_stick12_y(const uint8_t *d) {
    return (uint16_t)((d[1] >> 4) | (d[2] << 4));
}

static int16_t get_i16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int main(void) {
    memset(mock_bond_flash, 0xff, sizeof(mock_bond_flash));

    /* 1. Verify RAM state budget */
    assert(sizeof(s2_state_t) <= 111u);
    printf("PASS: sizeof(s2_state_t) = %zu bytes (<= 111)\n", sizeof(s2_state_t));

    /* 2. Verify Switch 1 HOGP GATT table layout, UUIDs, PnP ID, Report Refs, Advertising */
    assert(sw1_gatt_table[0].attNum == 0x2du);
    assert(sw1_gatt_table[0x01].attNum == 5u);
    assert(sw1_gatt_table[0x06].attNum == 4u);
    assert(sw1_gatt_table[0x0a].attNum == 5u);
    assert(sw1_gatt_table[0x0f].attNum == 4u);
    assert(sw1_gatt_table[0x13].attNum == 27u);
    assert(sw1_gatt_table[0x03].attrLen == 14u && memcmp(sw1_gatt_table[0x03].pAttrValue, "Pro Controller", 14) == 0);
    const uint8_t exp_pnp[7] = { 0x02, 0x7e, 0x05, 0x09, 0x20, 0x10, 0x02 };
    assert(sw1_gatt_table[0x0e].attrLen == 7u && memcmp(sw1_gatt_table[0x0e].pAttrValue, exp_pnp, 7) == 0);
    assert(sw1_gatt_table[0x1f].pAttrValue[0] == 0x30u && sw1_gatt_table[0x1f].pAttrValue[1] == 0x01u);
    assert(sw1_gatt_table[0x23].pAttrValue[0] == 0x21u && sw1_gatt_table[0x23].pAttrValue[1] == 0x01u);
    assert(sw1_gatt_table[0x27].pAttrValue[0] == 0x3fu && sw1_gatt_table[0x27].pAttrValue[1] == 0x01u);
    assert(sw1_gatt_table[0x2a].pAttrValue[0] == 0x01u && sw1_gatt_table[0x2a].pAttrValue[1] == 0x02u);
    assert(sw1_gatt_table[0x2d].pAttrValue[0] == 0x10u && sw1_gatt_table[0x2d].pAttrValue[1] == 0x02u);
    assert(sw1_gatt_table[SW1_H_OUT01].w == sw1_out01_write_cb);
    assert(sw1_gatt_table[SW1_H_OUT10].w == sw1_out10_write_cb);
    assert(sw1_gatt_table[0x1e].pAttrValue != sw1_gatt_table[0x22].pAttrValue);
    assert(sw1_gatt_table[0x22].pAttrValue != sw1_gatt_table[0x26].pAttrValue);
    assert(sw1_gatt_table[0x12].pAttrValue != sw1_gatt_table[0x09].pAttrValue);
    assert(sizeof(sw1_adv_data) == 29u && sw1_adv_data[5] == 0xc4u && sw1_adv_data[6] == 0x03u);
    assert(memcmp(&sw1_adv_data[15], "Pro Controller", 14) == 0);
    assert(sizeof(sw1_scan_rsp) == 16u && memcmp(&sw1_scan_rsp[2], "Pro Controller", 14) == 0);
    printf("PASS: Switch 1 HOGP GATT table & advertising verified!\n");

    /* 2b. Verify CSA #2 selector & symmetric PHY update resolver */
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
        assert(s2_resolve_phy_update(1u, 2u, 1u) == 0u);
        printf("PASS: CSA #2 & PHY update resolver verified!\n");
    }

    /* 3. Connect lifecycle, subcommand 0x02 (Device Info), and SPI flash reads (0x10) */
    memset(&g_s2_test_state, 0, sizeof(g_s2_test_state));
    mock_bat = 82u;
    sw1_on_ble_connect();
    assert(g_s2_test_state.input_mode == 0x3fu && g_s2_test_state.vib_enabled == 1u);
    assert(g_s2_test_state.bat_level == 82u && g_s2_test_state.last_bat_pct == 0xffu);
    g_s2_test_state.cccd_in21[0] = 0x01u;
    g_s2_test_state.cccd_in30[0] = 0x01u;
    g_s2_test_state.cccd_in3f[0] = 0x01u;

    send_out01(0u, NULL, 0x02u, NULL, 0u);
    assert(last_noti_handle == SW1_H_IN21 && last_noti_len == 48);
    assert(last_noti_buf[12] == 0x82u && last_noti_buf[13] == 0x02u);
    assert(last_noti_buf[14] == 0x03u && last_noti_buf[15] == 0x48u);
    assert(last_noti_buf[16] == 0x03u); /* Pro Controller */
    const uint8_t exp_be_mac[6] = { 0xe4, 0x17, 0xd8, 0xbc, 0xb6, 0x09 };
    assert(memcmp(&last_noti_buf[18], exp_be_mac, 6) == 0);
    assert(last_noti_buf[25] == 0x01u); /* SPI colors */

    /* Read factory IMU calibration @ 0x6020 (24 bytes) */
    const uint8_t spi_imu[5] = { 0x20, 0x60, 0x00, 0x00, 24 };
    send_out01(1u, NULL, 0x10u, spi_imu, sizeof(spi_imu));
    assert(last_noti_buf[12] == 0x90u && last_noti_buf[13] == 0x10u);
    assert(last_noti_buf[14] == 0x20u && last_noti_buf[15] == 0x60u && last_noti_buf[18] == 24u);
    assert(get_i16(&last_noti_buf[19 + 0]) == 0);       /* accel origin X */
    assert(get_i16(&last_noti_buf[19 + 6]) == 0x4000);  /* accel coeff X */
    assert(get_i16(&last_noti_buf[19 + 12]) == 0);      /* gyro origin X */
    assert(get_i16(&last_noti_buf[19 + 18]) == 0x343b); /* gyro coeff X */

    /* Read factory Stick calibration @ 0x603D (18 bytes) & verify macOS full-range quirk (max-min <= 1 -> 0x7FF) */
    const uint8_t spi_stk[5] = { 0x3d, 0x60, 0x00, 0x00, 18 };
    send_out01(2u, NULL, 0x10u, spi_stk, sizeof(spi_stk));
    assert(last_noti_buf[18] == 18u);
    assert(unpack_stick12_x(&last_noti_buf[19 + 0]) == 0x7ffu); /* L max X */
    assert(unpack_stick12_y(&last_noti_buf[19 + 0]) == 0x7ffu); /* L max Y */
    assert(unpack_stick12_x(&last_noti_buf[19 + 3]) == 0x800u); /* L ctr X */
    assert(unpack_stick12_y(&last_noti_buf[19 + 3]) == 0x800u); /* L ctr Y */
    assert(unpack_stick12_x(&last_noti_buf[19 + 6]) == 0x7ffu); /* L min X */
    assert(unpack_stick12_y(&last_noti_buf[19 + 6]) == 0x7ffu); /* L min Y */
    assert(unpack_stick12_x(&last_noti_buf[19 + 9]) == 0x800u); /* R ctr X */
    assert(unpack_stick12_x(&last_noti_buf[19 + 12]) == 0x7ffu);/* R min X */
    assert(unpack_stick12_x(&last_noti_buf[19 + 15]) == 0x7ffu);/* R max X */

    /* Read Stick params @ 0x6086 & verify inner deadzone = 0x001 */
    const uint8_t spi_prm[5] = { 0x86, 0x60, 0x00, 0x00, 18 };
    send_out01(3u, NULL, 0x10u, spi_prm, sizeof(spi_prm));
    assert(last_noti_buf[18] == 18u);
    assert(unpack_stick12_x(&last_noti_buf[19 + 3]) == 0x001u);

    /* User stick cal @ 0x8010 returns 0xFF (uncalibrated -> use factory) */
    const uint8_t spi_usr[5] = { 0x10, 0x80, 0x00, 0x00, 22 };
    send_out01(3u, NULL, 0x10u, spi_usr, sizeof(spi_usr));
    assert(last_noti_buf[19] == 0xffu && last_noti_buf[40] == 0xffu);
    printf("PASS: 0x02 device info & 0x10 SPI flash reads verified!\n");

    /* 4. Pending 0x21 FIFO retry under busy TX */
    noti_fail_count = 2;
    const uint8_t mode_30 = 0x30u;
    const uint8_t leds = 0x05u;
    send_out01(4u, NULL, 0x03u, &mode_30, 1u);
    assert(last_noti_len == -1 && g_s2_test_state.input_mode == 0x30u && g_s2_test_state.q_count == 1u);
    send_out01(5u, NULL, 0x30u, &leds, 1u);
    assert(last_noti_len == -1 && g_s2_test_state.player_lights == 0x05u && g_s2_test_state.q_count == 2u);
    sw1_flush_pending_replies();
    assert(g_s2_test_state.q_count == 0u);
    assert(last_noti_handle == SW1_H_IN21 && last_noti_buf[13] == 0x30u);
    send_out01(6u, NULL, 0x31u, NULL, 0u);
    assert(last_noti_buf[12] == 0xb0u && last_noti_buf[13] == 0x31u && last_noti_buf[14] == 0x05u);
    printf("PASS: Pending 0x21 FIFO retry & player lights verified!\n");

    /* 5. Input 0x3F Simple HID report & change detection */
    const uint8_t mode_3f = 0x3fu;
    send_out01(7u, NULL, 0x03u, &mode_3f, 1u);
    mock_btn = (1u << 13) | (1u << 9) | (1u << 6); /* South (bit13=Xbox A -> 0x01 B) + D-pad Up+Right (hat=1) */
    mock_ls = (0x00u << 8) | (0x00u << 24); /* Full Right, Full Up (Y=0) */
    mock_rs = (0x80u << 8) | (0x80u << 24);
    mock_trig = 0u;
    noti_calls = 0;
    sw1_send_input_report();
    assert(noti_calls == 1 && last_noti_handle == SW1_H_IN3F && last_noti_len == 11);
    assert(last_noti_buf[0] == 0x01u && last_noti_buf[2] == 1u);
    assert(last_noti_buf[3] == 0xffu && last_noti_buf[4] == 0xffu); /* LX = +max */
    assert(last_noti_buf[5] == 0x00u && last_noti_buf[6] == 0x00u); /* LY = Up (0) */
    /* Unchanged state suppresses redundant 0x3F packets until idle keepalive */
    sw1_send_input_report();
    assert(noti_calls == 1);
    /* Verify East(bit12->0x02 A), West(bit4->0x04 Y), North(bit5->0x08 X) in 0x3F */
    mock_btn = (1u << 12) | (1u << 4);
    sw1_send_input_report();
    assert(last_noti_buf[0] == (0x02u | 0x04u));
    mock_btn = (1u << 5);
    sw1_send_input_report();
    assert(last_noti_buf[0] == 0x08u);
    printf("PASS: 0x3F simple HID mode verified!\n");

    /* 6. Input 0x30 Full State + IMU frames + 12-bit sticks + buttons */
    send_out01(8u, NULL, 0x03u, &mode_30, 1u);
    const uint8_t imu_on = 1u;
    send_out01(9u, NULL, 0x40u, &imu_on, 1u);
    assert(g_s2_test_state.imu_enabled == 1u);

    mock_bat = 90u;
    mock_btn = (1u << 13) | (1u << 9) | (1u << 6) | (1u << 16); /* South (bit13=Xbox A -> 0x04 B), Up, Right, Capture */
    mock_ls = (0x00u << 8) | (0xffu << 24); /* X inv->0xff (0xFFF), Y inv->0x00 (0x000) */
    mock_rs = (0x80u << 8) | (0x80u << 24);
    mock_trig = (0x80u << 8) | 0x00u; /* ZR > 0x4d only */
    mock_imu[0] = -526; mock_imu[1] = -100; mock_imu[2] = 4120;
    mock_imu[3] = 200;  mock_imu[4] = -300; mock_imu[5] = 400;
    sw1_send_input_report();
    assert(last_noti_handle == SW1_H_IN30 && last_noti_len == 48);
    assert(last_noti_buf[1] == 0x80u); /* Full battery */
    assert(last_noti_buf[2] == (0x04u | 0x80u)); /* B (South) + ZR */
    assert(last_noti_buf[3] == 0x20u); /* Capture */
    assert(last_noti_buf[4] == (0x02u | 0x04u)); /* Up + Right */
    assert(last_noti_buf[5] == 0xffu && last_noti_buf[6] == 0x0fu && last_noti_buf[7] == 0x00u);
    for (unsigned f = 0; f < 3u; f++) {
        const uint8_t *im = &last_noti_buf[12u + f * 12u];
        assert(get_i16(&im[0]) == -526 && get_i16(&im[2]) == -100 && get_i16(&im[4]) == 4120);
        assert(get_i16(&im[6]) == 200  && get_i16(&im[8]) == -300 && get_i16(&im[10]) == 400);
    }
    /* Verify East(bit12->0x08 A), West(bit4->0x01 Y), North(bit5->0x02 X) in 0x30 */
    mock_trig = 0u;
    mock_btn = (1u << 12) | (1u << 4) | (1u << 5);
    sw1_send_input_report();
    assert(last_noti_buf[2] == (0x08u | 0x01u | 0x02u));
    /* Turbo raw button bit 27 -> Capture */
    mock_btn = 0u;
    mock_raw_btn = 1u << 27;
    sw1_send_input_report();
    assert(last_noti_buf[3] == 0x20u);
    mock_raw_btn = 0u;
    printf("PASS: 0x30 full input report & 3x IMU frames verified!\n");

    /* 7. Battery notification */
    g_s2_test_state.cccd_bat[0] = 0x01u;
    mock_bat = 65u;
    sw1_notify_battery();
    assert(last_noti_handle == SW1_H_BAT_LVL && last_noti_len == 1 && last_noti_buf[0] == 65u);
    printf("PASS: Battery service notifications verified!\n");

    /* 8. Switch 1 HD rumble decoding + enable/disable + L2 length check + disconnect cleanup */
    mock_rumble_calls = 0u;
    /* Left: HF max (0xc8), Right: LF mid (~0x59 -> step=50 -> 100) */
    const uint8_t rumb_active[8] = {
        0x74, 0xc8, 0x3d, 0x40,
        0x74, 0x01, 0x3d, 0x59,
    };
    send_out10(0u, rumb_active);
    assert(mock_rumble_calls == 1u && mock_rumble[0] == 255u && mock_rumble[1] == 100u);
    /* Neutral rumble via 0x01 must NOT stop active rumble */
    send_out01(10u, NULL, 0x04u, NULL, 0u);
    assert(mock_rumble_calls == 1u);
    /* Neutral rumble via 0x10 DOES stop active rumble */
    const uint8_t rumb_neutral[8] = { 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
    send_out10(1u, rumb_neutral);
    assert(mock_rumble_calls == 2u && mock_rumble[0] == 0u && mock_rumble[1] == 0u);
    /* Disable vibration via subcmd 0x48 0x00 */
    const uint8_t vib_off = 0u;
    send_out01(11u, NULL, 0x48u, &vib_off, 1u);
    send_out10(2u, rumb_active);
    assert(mock_rumble_calls == 2u);
    /* Oversized ATT write guard */
    uint8_t bad_att[16] = { 0, 0, 200, 0 };
    last_noti_len = -1;
    sw1_out01_write_cb(bad_att);
    assert(last_noti_len == -1);
    /* Disconnect resets CCCDs & stops rumble */
    const uint8_t vib_on = 1u;
    send_out01(12u, NULL, 0x48u, &vib_on, 1u);
    send_out10(3u, rumb_active);
    assert(mock_rumble_calls == 3u);
    sw1_on_ble_disconnect();
    assert(mock_rumble_calls == 4u && mock_rumble[0] == 0u && mock_rumble[1] == 0u);
    assert(g_s2_test_state.cccd_in30[0] == 0u && g_s2_test_state.cccd_in21[0] == 0u);
    assert(g_s2_test_state.input_mode == 0x3fu && g_s2_test_state.imu_enabled == 0u);
    printf("PASS: Switch 1 HD rumble, L2 length guard, and disconnect reset verified!\n");

    /* 9. Flash config, 4-slot resolution, MAC derivation & v4 record migration */
    memset(mock_bond_flash, 0xff, sizeof(mock_bond_flash));
    s2_bt_boot_t b = { 0 };
    s2_cfg_load_bt(&b);
    assert(b.slot_modes == S2_DEFAULT_SLOT_MODES);
    assert(s2_resolve_bt_boot(&b, 3u, 0xffu) == 0u && b.mode == S2_SLOT_MODE_SW1);
    assert(s2_cfg_save_bt(&b, 1u) == 1 && b.mac_gen == 1u);
    s2_bt_boot_t b2 = { 0 };
    s2_cfg_load_bt(&b2);
    assert(b2.slot == 3u && b2.mode == S2_SLOT_MODE_SW1 && b2.mac_gen == 1u);
    /* Assign Down (1) + Y (SW1=0) -> mode_changed = 1 */
    assert(s2_resolve_bt_boot(&b2, 1u, S2_SLOT_MODE_SW1) == 1u && b2.slot == 1u && b2.mode == S2_SLOT_MODE_SW1);
    /* Corrupt CRC test */
    mock_bond_flash[0][28] ^= 0xffu;
    mock_bond_flash[1][28] ^= 0xffu;
    assert(s2_cfg_load() == 0u);
    printf("PASS: Flash persistence & 4-slot Switch 1 mode resolution verified!\n");

    return 0;
}
