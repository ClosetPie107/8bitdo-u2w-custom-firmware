/* U2W Elite Series 2 BLE report format, as documented by the U2W Toolkit
 * prototype. Apple uses 045e:0b05 and a 39-byte report including its ID.
 */
#include <stdint.h>
#include "s2_state.h"

#define ELITE_REPORT_SIZE 38u

void elite_pack_report(uint8_t out[38], const uint8_t stock[33]) {
    for (unsigned i = 0; i < 4u; ++i) {
        out[i * 2u] = out[i * 2u + 1u] = stock[i + 1u];
    }
    for (unsigned i = 0; i < 2u; ++i) {
        uint8_t trigger = stock[6u - i];
        out[8u + i * 2u] = (uint8_t)((trigger << 2) | (trigger >> 6));
        out[9u + i * 2u] = trigger >> 6;
    }
    out[12] = stock[0] < 8u ? stock[0] + 1u : 0u;
    out[13] = stock[7] & 0xdbu;
    out[14] = stock[8] & 0x7cu;
    for (unsigned i = 15u; i < ELITE_REPORT_SIZE; ++i) out[i] = 0u;
    /* P1 = R4, P2 = PR, P3 = L4, P4 = PL. Profile byte stays zero. */
    out[16] = ((stock[9] & 2u) >> 1) | ((stock[7] & 4u) >> 1)
            | ((stock[9] & 1u) << 2) | ((stock[7] & 32u) >> 2);
    for (unsigned i = 0; i < 12u; ++i) out[22u + i] = stock[14u + i];
}

static uint8_t elite_motor_scale(uint8_t strength, uint8_t loop_count) {
    if (strength > 100u) strength = 100u;
    if (loop_count) {
        unsigned amplitude = ((uint32_t)strength * 164u) >> 6;
        return amplitude > 255u ? 255u : (uint8_t)amplitude;
    }

    /* Apple sends 100 * intensity^2. Recover the nearest 8-bit amplitude. */
    uint32_t scaled_square = (uint32_t)strength * 2601u;
    unsigned root = 0u;
    for (unsigned bit = 128u; bit; bit >>= 1) {
        unsigned candidate = root | bit;
        if (candidate * candidate <= scaled_square / 4u) root = candidate;
    }
    unsigned midpoint = 2u * root + 1u;
    return (uint8_t)(root + (scaled_square >= midpoint * midpoint));
}

int elite_decode_rumble(const uint8_t *p, uint8_t motors[2]) {
    /* The stock ATT callback receives a packet with its value at byte 9. */
    if (p[2] != 11u || p[3] != 0u) return 0;
    uint8_t left = p[10] > p[12] ? p[10] : p[12];
    uint8_t right = p[11] > p[13] ? p[11] : p[13];
    motors[0] = elite_motor_scale(left, p[16]);
    motors[1] = elite_motor_scale(right, p[16]);
    return 1;
}

#ifndef HOST_TEST
extern int stock_gatt_notify(uint16_t conn, uint16_t handle, const uint8_t *value, int len);
extern int stock_ble_rumble(void *p);
extern void stock_set_rumble(uint8_t left, uint8_t right, uint8_t duration);
extern const uint8_t stock_hid_map[113], stock_hid_pnp[7];
extern const uint8_t stock_hid_name[26];
extern uint8_t elite_map_ram[113], elite_pnp_ram[7];
extern uint8_t elite_name_ram[26];

/* Keep the stock appearance and advertised services. The scan response and
 * the GAP Device Name return the same full name. */
const uint8_t elite_adv_data[30] = {
    16, 0x08, '8','B','i','t','D','o',' ','U','2','W',' ','X','b','o','x',
    2, 0x01, 0x06,               /* General Discoverable, LE only */
    3, 0x19, 0xc4, 0x03,         /* Gamepad appearance */
    5, 0x02, 0x12, 0x18, 0x0f, 0x18, /* HID and Battery services */
};
/* The scan response sends only the AD field. Its zero padding fills the
 * 26-byte GAP characteristic shared with stock mode. */
const uint8_t elite_scan_rsp[28] = "\x17\x09" "8BitDo Ultimate 2 Xbox";

/* Keep the stock ATT map length. A 16-bit Usage Page and seven balanced
 * Push/Pop pairs bring the Elite map to 113 bytes without changing fields.
 */
static const uint8_t elite_hid_map[113] = {
    0x06,0x01,0x00, 0x09,0x05, 0xa1,0x01, 0x85,0x01,
    0x09,0x30, 0x09,0x31, 0x09,0x33, 0x09,0x34, 0x26,0xff,0xff,
    0x95,0x04, 0x75,0x10, 0x81,0x02,
    0x05,0x02, 0x09,0xc4, 0x09,0xc5, 0x26,0xff,0xff, 0x95,0x02, 0x75,0x10, 0x81,0x02,
    0x05,0x01, 0x09,0x39, 0x25,0x08, 0x95,0x01, 0x75,0x04, 0x81,0x42,
    0x95,0x01, 0x75,0x04, 0x81,0x03,
    0x05,0x09, 0x19,0x01, 0x29,0x10, 0x25,0x01, 0x95,0x10, 0x75,0x01, 0x81,0x02,
    0x06,0x00,0xff, 0x09,0x01, 0x75,0x08, 0x95,0x17, 0x81,0x02,
    0x05,0x0f, 0x09,0x21, 0x85,0x03, 0x09,0x70, 0x75,0x08, 0x95,0x08, 0x91,0x02,
    0xa4,0xb4, 0xa4,0xb4, 0xa4,0xb4, 0xa4,0xb4, 0xa4,0xb4, 0xa4,0xb4, 0xa4,0xb4,
    0xc0,
};
static const uint8_t elite_hid_pnp[] = {2u, 0x5eu, 0x04u, 0x05u, 0x0bu, 1u, 0u};

void elite_gatt_init(void) {
    uint8_t elite = S2_STOCK_BT >= S2_BT_ELITE_LATCH;
    const uint8_t *map = elite ? elite_hid_map : stock_hid_map;
    const uint8_t *pnp = elite ? elite_hid_pnp : stock_hid_pnp;
    const uint8_t *name = elite ? elite_scan_rsp + 2 : stock_hid_name;
    for (unsigned i = 0; i < sizeof(elite_map_ram); ++i) elite_map_ram[i] = map[i];
    for (unsigned i = 0; i < sizeof(elite_pnp_ram); ++i) elite_pnp_ram[i] = pnp[i];
    for (unsigned i = 0; i < sizeof(elite_name_ram); ++i) elite_name_ram[i] = name[i];
    /* First byte of the initialized stock HID Output Report Reference. */
    *(volatile uint8_t *)0x00845a14u = elite ? 3u : 5u;
}

int elite_notify_hook(uint16_t conn, uint16_t handle, const uint8_t *value, int len) {
    if (S2_STOCK_BT < S2_BT_ELITE_LATCH) return stock_gatt_notify(conn, handle, value, len);
    uint8_t report[ELITE_REPORT_SIZE];
    elite_pack_report(report, value);
    return stock_gatt_notify(conn, handle, report, sizeof(report));
}

int elite_rumble_hook(void *p) {
    if (S2_STOCK_BT < S2_BT_ELITE_LATCH) return stock_ble_rumble(p);
    uint8_t motors[2];
    if (elite_decode_rumble(p, motors)) stock_set_rumble(motors[0], motors[1], 0u);
    return 0;
}
#endif
