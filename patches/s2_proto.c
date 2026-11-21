/*
 * Nintendo Switch 2 Pro Controller BLE protocol implementation for U2W v1.10:
 *   - Compact AES-128-ECB for 0x15/02 LTK confirmation (B2)
 *   - 0x15 pairing state machine (01 addr, 04 keys, 02 confirm, 03 finalise)
 *   - Telink LL_ENC_REQ LTK callback feeding LTK = A1 ^ B1 to blc_ll_ltkReqReply
 *   - Simulated Pro Controller 2 SPI flash reads (0x02) + init commands
 *   - 63-byte Pro Controller 2 input report (0x09 on handle 0x000e)
 */

#include <stdint.h>
#include "s2_state.h"

#ifdef HOST_TEST
extern int host_gatt_notify(uint16_t conn_h, uint16_t att_h, const uint8_t *p, int len);
extern int host_ll_ltk_reply(uint16_t conn_h, const uint8_t *ltk);
extern int host_ll_ltk_neg_reply(uint16_t conn_h);
extern int  host_set_adv_data(const uint8_t *p, uint8_t len);
extern void host_get_local_bdaddr(uint8_t out[6]);
extern uint32_t host_get_buttons(void);
extern uint32_t host_get_lstick_raw(void);
extern uint32_t host_get_rstick_raw(void);
extern uint16_t host_get_triggers_raw(void);
extern uint8_t  host_get_battery_pct(void);
extern void     host_get_imu(int16_t *ax, int16_t *ay, int16_t *az,
                             int16_t *gx, int16_t *gy, int16_t *gz);
extern uint32_t host_get_system_tick(void);
extern void     host_flash_read(uint32_t addr, uint32_t len, uint8_t *buf);
extern void     host_flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);
extern void     host_flash_erase(uint32_t addr);
extern void     host_set_rumble(uint8_t left, uint8_t right, uint8_t duration);
#define CALL_GATT_NOTIFY(c, h, p, l)  host_gatt_notify((c), (h), (p), (l))
#define CALL_LTK_REPLY(c, k)          host_ll_ltk_reply((c), (k))
#define CALL_LTK_NEG_REPLY(c)         host_ll_ltk_neg_reply((c))
#define CALL_SET_ADV_DATA(p, l)       host_set_adv_data((p), (l))
#define CALL_GET_BAT_PCT()            host_get_battery_pct()
#define CALL_GET_IMU(ax,ay,az,gx,gy,gz) host_get_imu((ax),(ay),(az),(gx),(gy),(gz))
#define CALL_GET_TICK()               host_get_system_tick()
#else
extern int  stock_gatt_notify(uint16_t conn_h, uint16_t att_h, const uint8_t *p, int len);
extern int  stock_ll_ltk_reply(uint16_t conn_h, const uint8_t *ltk);
extern int  stock_ll_ltk_neg_reply(uint16_t conn_h);
extern int  stock_ll_set_adv_data(const uint8_t *p, uint8_t len);
extern uint8_t stock_get_battery_pct(void);
extern void stock_get_imu(int16_t *ax, int16_t *ay, int16_t *az,
                          int16_t *gx, int16_t *gy, int16_t *gz);
extern void stock_flash_erase_sector(uint32_t addr);
extern void stock_flash_enable_writes(void);
extern void stock_flash_protect(void);
extern int  stock_l2cap_rx(uint16_t conn_h, uint8_t *raw);
extern void stock_set_rumble(uint8_t left, uint8_t right, uint8_t duration);
#define CALL_GATT_NOTIFY(c, h, p, l)  stock_gatt_notify((c), (h), (p), (l))
#define CALL_LTK_REPLY(c, k)          stock_ll_ltk_reply((c), (k))
#define CALL_LTK_NEG_REPLY(c)         stock_ll_ltk_neg_reply((c))
#define CALL_SET_ADV_DATA(p, l)       stock_ll_set_adv_data((p), (l))
#define CALL_GET_BAT_PCT()            stock_get_battery_pct()
#define CALL_GET_IMU(ax,ay,az,gx,gy,gz) stock_get_imu((ax),(ay),(az),(gx),(gy),(gz))
#define CALL_GET_TICK()               (*(const volatile uint32_t *)0x00800740u)
#endif

/* =========================================================================
 * Compact AES-128-ECB (encrypt only) for Switch 2 pairing confirmation B2
 * ========================================================================= */

static const uint8_t aes_sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
};

static uint8_t xtime(uint8_t x) {
    return (uint8_t)((x << 1) ^ ((x & 0x80u) ? 0x1bu : 0x00u));
}

static void aes128_ecb_encrypt(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
    uint8_t rk[16];
    uint8_t s[16];
    uint8_t rcon = 1u;

    for (int i = 0; i < 16; i++) {
        rk[i] = key[i];
        s[i] = in[i] ^ rk[i];
    }

    for (int round = 1; round <= 10; round++) {
        /* SubBytes */
        for (int i = 0; i < 16; i++) {
            s[i] = aes_sbox[s[i]];
        }
        /* ShiftRows */
        uint8_t t;
        t = s[1];  s[1]  = s[5];  s[5]  = s[9];  s[9]  = s[13]; s[13] = t;
        t = s[2];  s[2]  = s[10]; s[10] = t;
        t = s[6];  s[6]  = s[14]; s[14] = t;
        t = s[15]; s[15] = s[11]; s[11] = s[7];  s[7]  = s[3];  s[3]  = t;

        /* MixColumns (rounds 1..9) */
        if (round < 10) {
            for (int c = 0; c < 16; c += 4) {
                uint8_t a0 = s[c], a1 = s[c+1], a2 = s[c+2], a3 = s[c+3];
                uint8_t u = a0 ^ a1 ^ a2 ^ a3;
                s[c]   = a0 ^ u ^ xtime(a0 ^ a1);
                s[c+1] = a1 ^ u ^ xtime(a1 ^ a2);
                s[c+2] = a2 ^ u ^ xtime(a2 ^ a3);
                s[c+3] = a3 ^ u ^ xtime(a3 ^ a0);
            }
        }

        /* On-the-fly key schedule */
        uint8_t k0 = aes_sbox[rk[13]] ^ rcon;
        uint8_t k1 = aes_sbox[rk[14]];
        uint8_t k2 = aes_sbox[rk[15]];
        uint8_t k3 = aes_sbox[rk[12]];
        rcon = xtime(rcon);
        rk[0] ^= k0; rk[1] ^= k1; rk[2] ^= k2; rk[3] ^= k3;
        for (int i = 4; i < 16; i++) {
            rk[i] ^= rk[i - 4];
        }

        /* AddRoundKey */
        for (int i = 0; i < 16; i++) {
            s[i] ^= rk[i];
        }
    }

    for (int i = 0; i < 16; i++) {
        out[i] = s[i];
    }
}

/* =========================================================================
 * Simulated Pro Controller 2 SPI flash (command 0x02)
 *
 * Layout and nominal stick/IMU calibration blocks derived from public
 * Switch 2 Pro Controller documentation (ndeadly/switch2_controller_research,
 * novakpetya/linux-switch2), with zeroed gyro bias for the U2W IMU.
 * ========================================================================= */

static const uint8_t kFlash_013000[64] = {
    0x01, 0x00, 0x48, 0x45, 0x4a, 0x37, 0x31, 0x30, 0x30, 0x31, 0x31, 0x32, 0x31, 0x32, 0x34, 0x37,
    0x00, 0x00, 0x7e, 0x05, 0x69, 0x20, 0x01, 0x06, 0x01, 0x23, 0x23, 0x23, 0xa0, 0xa0, 0xa0, 0xe6,
    0xe6, 0xe6, 0x32, 0x32, 0x32, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
/* Bytes 4..15 are 3 LE float32 gyro zero-bias values; zero them for U2W IMU. */
static const uint8_t kFlash_013040[16] = {
    0x3b, 0xe0, 0xd3, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t kFlash_013080[64] = {
    0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a, 0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e,
    0xe2, 0x20, 0x0e, 0x9a, 0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5, 0x50, 0x2f,
    0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff, 0xb3, 0x67, 0x83, 0x2e, 0x66, 0x5e, 0x3a, 0x06,
    0x5f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
static const uint8_t kFlash_0130C0[64] = {
    0x01, 0xad, 0xd9, 0x9a, 0x55, 0x56, 0x65, 0xa0, 0x00, 0x0a, 0xa0, 0x00, 0x0a, 0xe2, 0x20, 0x0e,
    0xe2, 0x20, 0x0e, 0x9a, 0xad, 0xd9, 0x9a, 0xad, 0xd9, 0x0a, 0xa5, 0x50, 0x0a, 0xa5, 0x50, 0x2f,
    0xf6, 0x62, 0x2f, 0xf6, 0x62, 0x0a, 0xff, 0xff, 0x2c, 0x08, 0x84, 0xd1, 0x65, 0x63, 0x2a, 0x26,
    0x62, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
static const uint8_t kFlash_013100[24] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xa6, 0xf2, 0x62, 0xbd, 0xa8, 0x00, 0x08, 0x3d, 0x2f, 0xed, 0x20, 0x41,
};

static uint8_t sim_flash_byte(uint32_t a) {
    if (a >= 0x013000u && a < 0x013040u) return kFlash_013000[a - 0x013000u];
    if (a >= 0x013040u && a < 0x013050u) return kFlash_013040[a - 0x013040u];
    if (a >= 0x013080u && a < 0x0130c0u) return kFlash_013080[a - 0x013080u];
    if (a >= 0x0130c0u && a < 0x013100u) return kFlash_0130C0[a - 0x0130c0u];
    if (a >= 0x013100u && a < 0x013118u) return kFlash_013100[a - 0x013100u];
    return 0xffu;
}

/* =========================================================================
 * Pairing constants & response sender
 * ========================================================================= */

static const uint8_t kControllerKeyB1[16] = {
    0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1,
    0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10,
};

static int s2_send_response_now(
    uint8_t via_vib_cmd,
    uint8_t cmd,
    uint8_t transport,
    uint8_t sub,
    const uint8_t *payload,
    uint8_t payload_len
) {
    /* Max response: 14 (prefix) + 8 (hdr) + 72 (8 + 64B flash read) = 94 bytes */
    uint8_t out[104];
    uint8_t pos = 0;

    if (payload_len > 80u) {
        payload_len = 80u;
    }
    if (via_vib_cmd) {
        for (int i = 0; i < 14; i++) {
            out[pos++] = 0x00u;
        }
    }
    out[pos++] = cmd;
    out[pos++] = 0x01u; /* DIR_DEVICE_TO_HOST */
    out[pos++] = transport;
    out[pos++] = sub;
    out[pos++] = 0x10u; /* RSP_BYTE4_BT */
    out[pos++] = 0x78u; /* RSP_BYTE5_BT */
    out[pos++] = 0x00u;
    out[pos++] = 0x00u;
    for (uint8_t i = 0; i < payload_len; i++) {
        out[pos++] = payload[i];
    }

    uint16_t handle = via_vib_cmd ? S2_H_CMD_RESP2 : S2_H_CMD_RESP1;
    int rc = CALL_GATT_NOTIFY(BLS_CONN_HANDLE, handle, out, (int)pos);
    if (rc == 0 && cmd == 0x15u && sub == 0x03u && (S2_STATE->have_ltk & S2_BOND_DIRTY)) {
        S2_STATE->have_ltk |= S2_BOND_READY;
    }
    return rc;
}

enum {
    S2_CONST_EMPTY   = 0u,
    S2_CONST_OK1     = 1u,
    S2_CONST_Z1      = 2u,
    S2_CONST_Z4      = 3u,
    S2_CONST_Z24     = 4u,
    S2_CONST_11_01   = 5u,
    S2_CONST_11_03   = 6u,
    S2_CONST_18_01   = 7u,
    S2_CONST_FW_INFO = 8u,
    S2_CONST_NFC     = 9u,
};

static const uint8_t *s2_const_payload(uint8_t id, uint8_t *len_out) {
    static const uint8_t kOk1[1]   = { 0x01u };
    static const uint8_t kZ24[24]  = { 0 };
    static const uint8_t k11_01[4] = { 0x01, 0x00, 0x00, 0x00 };
    static const uint8_t k11_03[29] = {
        0x01,0x20,0x03,0x00,0x00,0x0a,0xe8,0x1c,0x3b,0x79,0x7d,0x8b,0x3a,0x0a,0xe8,
        0x9c,0x42,0x58,0xa0,0x0b,0x42,0x0a,0xe8,0x9c,0x41,0x58,0xa0,0x0b,0x41,
    };
    static const uint8_t k18_01[8] = { 0x00,0x00,0x40,0xf0,0x00,0x00,0x60,0x00 };
    static const uint8_t kFw[12]   = {
        0x02, 0x01, 0x04, 0x02, 0x0c, 0x00,
        0x00, 0x00, 0x00, 0x02, 0x03, 0x00,
    };
    static const uint8_t kNfc[4]   = { 0x61, 0x12, 0x50, 0x0d };
    static const struct {
        const uint8_t *ptr;
        uint8_t len;
    } kTable[10] = {
        { 0,      0u  }, /* S2_CONST_EMPTY   */
        { kOk1,   1u  }, /* S2_CONST_OK1     */
        { kZ24,   1u  }, /* S2_CONST_Z1      */
        { kZ24,   4u  }, /* S2_CONST_Z4      */
        { kZ24,   24u }, /* S2_CONST_Z24     */
        { k11_01, 4u  }, /* S2_CONST_11_01   */
        { k11_03, 29u }, /* S2_CONST_11_03   */
        { k18_01, 8u  }, /* S2_CONST_18_01   */
        { kFw,    12u }, /* S2_CONST_FW_INFO */
        { kNfc,   4u  }, /* S2_CONST_NFC     */
    };
    if (id >= 10u) id = 0u;
    *len_out = kTable[id].len;
    return kTable[id].ptr;
}

static void s2_flush_pending_response(void);

static void s2_send_response(
    uint8_t via_vib_cmd,
    uint8_t cmd,
    uint8_t transport,
    uint8_t sub,
    const uint8_t *payload,
    uint8_t payload_len
) {
    s2_state_t *s = S2_STATE;
    if (s->pending_rsp_valid != 0u) {
        s2_flush_pending_response();
        if (s->pending_rsp_valid != 0u) {
            return;
        }
    }
    int rc = s2_send_response_now(via_vib_cmd, cmd, transport, sub, payload, payload_len);
    if (rc != 0 && payload_len <= sizeof(s->pending_data)) {
        s->pending_rsp_valid = 1u;
        if (via_vib_cmd) s->flags |= S2_FLAG_PENDING_VIB;
        else s->flags &= (uint8_t)~S2_FLAG_PENDING_VIB;
        s->pending_cmd = cmd;
        s->pending_transport = transport;
        s->pending_sub = sub;
        s->pending_len = payload_len;
        for (uint8_t i = 0; i < payload_len; i++) {
            s->pending_data[i] = payload[i];
        }
    }
}

static void s2_queue_secondary(
    uint8_t kind,
    uint8_t via_vib_cmd,
    uint8_t cmd,
    uint8_t transport,
    uint8_t sub,
    const uint8_t data[5]
) {
    s2_state_t *s = S2_STATE;
    if (s->pending_q.valid != 0u) {
        /* Both FIFO slots full: preserve earlier pending replies in order. */
        return;
    }
    s->pending_q.valid = kind;
    s->pending_q.cmd = cmd;
    s->pending_q.transport = transport;
    s->pending_q.sub = sub;
    s->pending_q.via_vib = via_vib_cmd;
    for (int i = 0; i < 5; i++) {
        s->pending_q.data[i] = data[i];
    }
}

static void s2_send_const_response(
    uint8_t via_vib_cmd,
    uint8_t cmd,
    uint8_t transport,
    uint8_t sub,
    uint8_t const_id
) {
    s2_state_t *s = S2_STATE;
    uint8_t qdata[5] = { const_id, 0u, 0u, 0u, 0u };
    if (s->pending_rsp_valid != 0u) {
        s2_flush_pending_response();
        if (s->pending_rsp_valid != 0u) {
            s2_queue_secondary(3u, via_vib_cmd, cmd, transport, sub, qdata);
            return;
        }
    }
    uint8_t plen = 0u;
    const uint8_t *payload = s2_const_payload(const_id, &plen);
    if (s2_send_response_now(via_vib_cmd, cmd, transport, sub, payload, plen) != 0) {
        s->pending_rsp_valid = 3u;
        if (via_vib_cmd) s->flags |= S2_FLAG_PENDING_VIB;
        else s->flags &= (uint8_t)~S2_FLAG_PENDING_VIB;
        s->pending_cmd = cmd;
        s->pending_transport = transport;
        s->pending_sub = sub;
        s->pending_len = 1u;
        s->pending_data[0] = const_id;
    }
}

static int s2_send_flash_read_now(
    uint8_t via_vib_cmd,
    uint8_t transport,
    uint8_t sub,
    uint8_t rlen,
    uint32_t addr
) {
    if (rlen > 64u) {
        rlen = 64u;
    }
    uint8_t reply[72];
    reply[0] = rlen; reply[1] = 0; reply[2] = 0; reply[3] = 0;
    reply[4] = (uint8_t)(addr & 0xffu);
    reply[5] = (uint8_t)((addr >> 8) & 0xffu);
    reply[6] = (uint8_t)((addr >> 16) & 0xffu);
    reply[7] = (uint8_t)((addr >> 24) & 0xffu);
    for (uint8_t i = 0; i < rlen; i++) {
        reply[8u + i] = sim_flash_byte(addr + i);
    }
    return s2_send_response_now(via_vib_cmd, 0x02u, transport, sub, reply, (uint8_t)(8u + rlen));
}

static void s2_flush_pending_response(void) {
    s2_state_t *s = S2_STATE;
    if (s->pending_rsp_valid == 0u) {
        return;
    }
    uint8_t via_vib = (s->flags & S2_FLAG_PENDING_VIB) != 0u;
    int rc;
    if (s->pending_rsp_valid == 2u) {
        uint8_t rlen = s->pending_data[0];
        uint32_t addr = (uint32_t)s->pending_data[1]
                      | ((uint32_t)s->pending_data[2] << 8)
                      | ((uint32_t)s->pending_data[3] << 16)
                      | ((uint32_t)s->pending_data[4] << 24);
        rc = s2_send_flash_read_now(
            via_vib, s->pending_transport, s->pending_sub, rlen, addr
        );
    } else if (s->pending_rsp_valid == 3u) {
        uint8_t plen = 0u;
        const uint8_t *payload = s2_const_payload(s->pending_data[0], &plen);
        rc = s2_send_response_now(
            via_vib, s->pending_cmd, s->pending_transport,
            s->pending_sub, payload, plen
        );
    } else {
        rc = s2_send_response_now(
            via_vib, s->pending_cmd, s->pending_transport,
            s->pending_sub, s->pending_data, s->pending_len
        );
    }
    if (rc == 0) {
        if (s->pending_q.valid != 0u) {
            /* Promote second FIFO entry into slot 0. */
            s->pending_rsp_valid = s->pending_q.valid;
            if (s->pending_q.via_vib) s->flags |= S2_FLAG_PENDING_VIB;
            else s->flags &= (uint8_t)~S2_FLAG_PENDING_VIB;
            s->pending_cmd = s->pending_q.cmd;
            s->pending_transport = s->pending_q.transport;
            s->pending_sub = s->pending_q.sub;
            s->pending_len = (s->pending_q.valid == 2u) ? 5u : 1u;
            for (int i = 0; i < 5; i++) {
                s->pending_data[i] = s->pending_q.data[i];
            }
            s->pending_q.valid = 0u;
        } else {
            s->pending_rsp_valid = 0u;
        }
    }
}

static void s2_get_local_bdaddr(uint8_t out[6]) {
#ifdef HOST_TEST
    host_get_local_bdaddr(out);
#else
    /* L8: Telink LL public BD_ADDR at 0x008449e4 */
    const volatile uint8_t *mac = (const volatile uint8_t *)0x008449e4u;
    for (int i = 0; i < 6; i++) {
        out[i] = mac[i];
    }
#endif
}

/* =========================================================================
 * Pairing (0x15) & general command dispatcher
 * ========================================================================= */

static void s2_handle_pairing(
    uint8_t via_vib_cmd,
    uint8_t transport,
    uint8_t sub,
    const uint8_t *payload,
    uint16_t len
) {
    s2_state_t *s = S2_STATE;
    /* Replacing an existing console bond requires an active Pair-button window. */
    if ((s->have_ltk & S2_BOND_VALID) && (s->flags & S2_FLAG_PAIRING_OPEN) == 0u) {
        s->pairing_stage = 0u;
        return;
    }
#ifdef S2_BOND_DIAGNOSTICS
    s->bond_diagnostics |= 0x01u;  /* Received a pairing command. */
#endif

    if (sub == 0x01u) { /* EXCHANGE_ADDRESSES */
        if (len < 8u) {
            s->pairing_stage = 0u;
            return;
        }
        for (int i = 0; i < 6; i++) {
            s->scratch.pair.pending_host_addr[i] = payload[2 + i];
        }
        s->pairing_stage = 1u;

        uint8_t reply[9];
        reply[0] = 0x01u;
        reply[1] = 0x04u;
        reply[2] = 0x01u;
        s2_get_local_bdaddr(&reply[3]);
        s2_send_response(via_vib_cmd, 0x15u, transport, 0x01u, reply, 9u);
    } else if (sub == 0x04u) { /* EXCHANGE_KEYS */
        if (s->pairing_stage < 1u || len < 17u) {
            s->pairing_stage = 0u;
            return;
        }
        for (int i = 0; i < 16; i++) {
            s->scratch.pair.pending_ltk[i] = (uint8_t)(payload[1 + i] ^ kControllerKeyB1[i]);
        }
        s->pairing_stage = 2u;

        uint8_t reply[17];
        reply[0] = 0x01u;
        for (int i = 0; i < 16; i++) {
            reply[1 + i] = kControllerKeyB1[i];
        }
        s2_send_response(via_vib_cmd, 0x15u, transport, 0x04u, reply, 17u);
    } else if (sub == 0x02u) { /* CONFIRM_LTK */
        if (s->pairing_stage < 2u || len < 17u) {
            s->pairing_stage = 0u;
            return;
        }
        uint8_t key_rev[16];
        uint8_t blk_rev[16];
        uint8_t reply[17];
        for (int i = 0; i < 16; i++) {
            key_rev[i] = s->scratch.pair.pending_ltk[15 - i];
            blk_rev[i] = payload[1 + (15 - i)];
        }
        reply[0] = 0x01u;
        aes128_ecb_encrypt(key_rev, blk_rev, &reply[1]);
        s->pairing_stage = 3u;
        s2_send_response(via_vib_cmd, 0x15u, transport, 0x02u, reply, 17u);
    } else if (sub == 0x03u) { /* FINALISE */
        if (s->pairing_stage < 3u) {
            return;
        }
        for (int i = 0; i < 16; i++) {
            s->ltk[i] = s->scratch.pair.pending_ltk[i];
        }
        for (int i = 0; i < 6; i++) {
            s->host_addr[i] = s->scratch.pair.pending_host_addr[i];
        }
        s->have_ltk = S2_BOND_VALID | S2_BOND_DIRTY;
#ifdef S2_BOND_DIAGNOSTICS
        s->bond_diagnostics |= 0x02u;  /* Pairing reached finalization. */
#endif
        s->flags &= (uint8_t)~(S2_FLAG_BOND_ATTEMPTED | S2_FLAG_PAIRING_OPEN);
        s->pairing_stage = 0u;
        s->scratch.imu.valid = 0u;

        s2_send_const_response(via_vib_cmd, 0x15u, transport, 0x03u, S2_CONST_OK1);
    }
}

#define S2_FEATURE_MOTION      0x04u
#define S2_FEATURE_RUMBLE      0x20u
#define S2_RUMBLE_REPORT_SIZE  33u
#define S2_RUMBLE_HOLD_TICKS   5u /* 50 ms at the stock 10 ms motor tick */

#ifdef HOST_TEST
#define CALL_SET_RUMBLE host_set_rumble
#else
#define CALL_SET_RUMBLE stock_set_rumble
#endif

static void s2_stop_rumble(void) {
    if (S2_STATE->flags & S2_FLAG_RUMBLE_ACTIVE) {
        CALL_SET_RUMBLE(0u, 0u, 1u);
        S2_STATE->flags &= (uint8_t)~S2_FLAG_RUMBLE_ACTIVE;
    }
}

/* Combine the two bands and take the peak of the grip's 5 ms samples.
 * The U2W motor driver takes one scalar amplitude per grip. */
static uint8_t s2_rumble_level(const uint8_t *block) {
    uint16_t peak = 0u;
    unsigned count = (block[0] >> 4) & 3u;
    for (unsigned i = 0; i < count; i++) {
        const uint8_t *sample = block + 1u + i * 5u;
        uint16_t low = (sample[1] >> 2) | ((uint16_t)(sample[2] & 0x0fu) << 6);
        uint16_t high = (sample[3] >> 6) | ((uint16_t)sample[4] << 2);
        uint16_t combined = low + high;
        if (combined > peak) peak = combined;
    }
    if (peak == 0u) return 0u;
    uint16_t scaled = (uint16_t)((peak + 1u) >> 1);
    return scaled > 255u ? 255u : (uint8_t)scaled;
}

static void s2_apply_rumble(const uint8_t *report, uint16_t len) {
    if (len < S2_RUMBLE_REPORT_SIZE || (report[0] != 0u && report[0] != 2u) ||
        (S2_STATE->enabled_features & S2_FEATURE_RUMBLE) == 0u) return;

    const uint8_t *left = report + 1u;
    const uint8_t *right = report + 17u;
    /* Zero headers also appear in command-only writes. */
    if (left[0] == 0u && right[0] == 0u) return;
    if ((left[0] && (left[0] & 0xc0u) != 0x40u) ||
        (right[0] && (right[0] & 0xc0u) != 0x40u)) return;

    uint8_t left_level = s2_rumble_level(left);
    uint8_t right_level = s2_rumble_level(right);
    if (left_level || right_level) {
        /* Hold each pulse for 50 ms so 5 ms BLE bursts reach the 10 ms stock
         * motor tick and complete the ERM spin-up; silence lets it expire. */
        CALL_SET_RUMBLE(left_level, right_level, S2_RUMBLE_HOLD_TICKS);
        S2_STATE->flags |= S2_FLAG_RUMBLE_ACTIVE;
    }
}

static void s2_handle_vibration_cmd(uint8_t sub, const uint8_t *payload, uint16_t plen) {
    if (sub != 0x02u || plen < 1u) return;
    uint8_t sample = payload[0];
    if (sample == 0u) {
        s2_stop_rumble();
        return;
    }
    uint8_t level = 150u;
    uint8_t duration = 12u; /* 120 ms at the stock 10 ms motor tick */
    if (sample == 1u) {
        level = 160u;
        duration = 20u; /* 200 ms buzz (e.g. vibration toggle) */
    } else if (sample == 2u) {
        level = 255u;
        duration = 80u; /* 800 ms Search for Controllers */
    } else if (sample == 3u) {
        level = 110u;
        duration = 8u;  /* 80 ms crisp connect confirmation */
    } else if (sample == 5u) {
        level = 180u;
        duration = 15u;
    }
    CALL_SET_RUMBLE(level, level, duration);
    S2_STATE->flags |= S2_FLAG_RUMBLE_ACTIVE;
}

void s2_handle_command_bytes(uint8_t via_vib_cmd, const uint8_t *data, uint16_t len) {
    s2_state_t *s = S2_STATE;

    if (via_vib_cmd) {
        if (len < S2_RUMBLE_REPORT_SIZE + 8u) {
            return;
        }
        data += S2_RUMBLE_REPORT_SIZE;
        len -= S2_RUMBLE_REPORT_SIZE;
    }
    if (len < 8u) {
        return;
    }

    uint8_t cmd = data[0];
    uint8_t transport = data[2];
    uint8_t sub = data[3];
    const uint8_t *payload = data + 8u;
    uint16_t plen = (uint16_t)(len - 8u);

    if (cmd == 0x15u) {
        s2_handle_pairing(via_vib_cmd, transport, sub, payload, plen);
        return;
    }

    if (cmd == 0x02u) { /* FLASH_READ */
        if (plen < 8u) {
            s2_send_const_response(via_vib_cmd, cmd, transport, sub, S2_CONST_EMPTY);
            return;
        }
        uint8_t rlen = payload[0];
        if (rlen > 64u) {
            rlen = 64u;
        }
        uint32_t addr = (uint32_t)payload[4]
                      | ((uint32_t)payload[5] << 8)
                      | ((uint32_t)payload[6] << 16)
                      | ((uint32_t)payload[7] << 24);
        uint8_t qdata[5] = { rlen, payload[4], payload[5], payload[6], payload[7] };
        if (s->pending_rsp_valid != 0u) {
            s2_flush_pending_response();
            if (s->pending_rsp_valid != 0u) {
                s2_queue_secondary(2u, via_vib_cmd, 0x02u, transport, sub, qdata);
                return;
            }
        }
        if (s2_send_flash_read_now(via_vib_cmd, transport, sub, rlen, addr) != 0) {
            /* Queue compact flash-read descriptor for retry. */
            s->pending_rsp_valid = 2u;
            if (via_vib_cmd) s->flags |= S2_FLAG_PENDING_VIB;
            else s->flags &= (uint8_t)~S2_FLAG_PENDING_VIB;
            s->pending_cmd = 0x02u;
            s->pending_transport = transport;
            s->pending_sub = sub;
            s->pending_len = 5u;
            for (int i = 0; i < 5; i++) s->pending_data[i] = qdata[i];
        }
    } else if (cmd == 0x07u) {
        s2_send_const_response(via_vib_cmd, cmd, transport, sub, S2_CONST_Z1);
    } else if (cmd == 0x16u) {
        s2_send_const_response(via_vib_cmd, cmd, transport, sub, S2_CONST_Z24);
    } else if (cmd == 0x11u) {
        s2_send_const_response(
            via_vib_cmd, cmd, transport, sub,
            sub == 0x01u ? S2_CONST_11_01 : S2_CONST_11_03
        );
    } else if (cmd == 0x18u) {
        s2_send_const_response(
            via_vib_cmd, cmd, transport, sub,
            sub == 0x01u ? S2_CONST_18_01 : S2_CONST_EMPTY
        );
    } else if (cmd == 0x0cu) { /* FEATURE_SELECT */
        if (plen >= 1u) {
            if (sub == 0x02u) {
                s->feature_mask = payload[0];
                s->enabled_features = payload[0];
            } else if (sub == 0x04u) {
                s->enabled_features |= s->feature_mask ? (uint8_t)(payload[0] & s->feature_mask) : payload[0];
            } else if (sub == 0x05u) {
                s->enabled_features &= (uint8_t)(~payload[0]);
            }
        }
        if ((s->enabled_features & S2_FEATURE_RUMBLE) == 0u) s2_stop_rumble();
        s2_send_const_response(via_vib_cmd, cmd, transport, sub, S2_CONST_Z4);
    } else if (cmd == 0x10u) { /* FIRMWARE_INFO (FW 2.1.4 | Pro2 0x02 | BT 12.0.0 | DSP 2.3.0) */
        s2_send_const_response(via_vib_cmd, cmd, transport, sub, S2_CONST_FW_INFO);
    } else if (cmd == 0x01u && sub == 0x0cu) { /* NFC init probe */
        s2_send_const_response(via_vib_cmd, cmd, transport, sub, S2_CONST_NFC);
    } else {
        if (cmd == 0x0au) s2_handle_vibration_cmd(sub, payload, plen);
        s2_send_const_response(via_vib_cmd, cmd, transport, sub, S2_CONST_EMPTY);
    }
}

/* Telink GATT write callbacks: p[2..3] = l2capLen (= 3 + val_len), p+9 = value */
static int s2_att_write_common(void *p, uint8_t via_vib_cmd) {
    const uint8_t *pkt = (const uint8_t *)p;
    uint16_t l2cap_len = (uint16_t)pkt[2] | ((uint16_t)pkt[3] << 8);
    if (l2cap_len <= 3u || l2cap_len > 162u) { /* L2: bound against stale overflow length */
        return 0;
    }
    uint16_t len = (uint16_t)(l2cap_len - 3u);
    if (via_vib_cmd) s2_apply_rumble(pkt + 9u, len);
    s2_handle_command_bytes(via_vib_cmd, pkt + 9u, len);
    return 0;
}

int s2_cmd_write_cb(void *p) {
    return s2_att_write_common(p, 0u);
}

int s2_vib_cmd_write_cb(void *p) {
    return s2_att_write_common(p, 1u);
}

int s2_vib_write_cb(void *p) {
    const uint8_t *pkt = (const uint8_t *)p;
    uint16_t l2cap_len = (uint16_t)pkt[2] | ((uint16_t)pkt[3] << 8);
    if (l2cap_len >= 3u + S2_RUMBLE_REPORT_SIZE && l2cap_len <= 162u) {
        s2_apply_rumble(pkt + 9u, (uint16_t)(l2cap_len - 3u));
    }
    return 0;
}

/* =========================================================================
 * Telink LL_ENC_REQ LTK callback (registered via 0x16240)
 * ========================================================================= */

int s2_ltk_req_cb(uint16_t conn_handle, uint8_t *rand, uint16_t ediv) {
    (void)rand;
    (void)ediv;
    s2_state_t *s = S2_STATE;
#ifdef S2_BOND_DIAGNOSTICS
    s->bond_diagnostics |= 0x04u;  /* Link layer requested an LTK. */
#endif
    if (s->have_ltk & S2_BOND_VALID) {
        CALL_LTK_REPLY(conn_handle, s->ltk);
    } else {
        CALL_LTK_NEG_REPLY(conn_handle);
    }
    return 1;
}

int s2_l2cap_rx_tap(uint16_t conn_h, uint8_t *raw) {
#ifdef HOST_TEST
    (void)conn_h;
    (void)raw;
    return 0;
#else
    int r = stock_l2cap_rx(conn_h, raw);
    /* Keep ATT_EXCHANGE_MTU_RSP at 512 for Switch 2, but clamp effective_MTU to 158
     * so Telink's ATT TX buffer never overflows during GATT discovery responses. */
    if (*(volatile uint16_t *)0x00844d8eu > 158u) {
        *(volatile uint16_t *)0x00844d8eu = 158u;
    }
    return r;
#endif
}

void s2_on_ble_connect(void) {
    s2_state_t *s = S2_STATE;
#ifndef HOST_TEST
    s2_phy_reset_1m();
    /* Suppress stock 0xd4a2 200 ms BLE connect buzz; Switch 2 sends 0x0a/02 sample 3. */
    CALL_SET_RUMBLE(0u, 0u, 1u);
#endif
    s->pairing_stage = 0u;
    s->scratch.imu.valid = 0u;
    s->pending_rsp_valid = 0u;
    s->pending_q.valid = 0u;
    s->flags &= (uint8_t)~S2_FLAG_BOND_ATTEMPTED;
}

/* Two sectors outside the application image and stock BLE bond area. */
#define S2_BOND_SECTOR 0x0002b000u
#define S2_BOND_BACKUP_SECTOR 0x0002c000u
#define S2_BOND_RECORD_SIZE 32u

static uint8_t s2_bond_checksum(const uint8_t rec[32]) {
    uint8_t x = 0xa5u;
    for (int i = 0; i < 27; i++) {
        x ^= rec[i];
    }
    return x;
}

static int s2_flash_read_bytes(uint32_t addr, uint32_t len, uint8_t *buf) {
#ifdef HOST_TEST
    host_flash_read(addr, len, buf);
#else
    typedef void (*telink_flash_read_fn)(uint32_t, uint32_t, uint8_t *);
    telink_flash_read_fn f_read = *(telink_flash_read_fn * volatile)0x00844904u;
    if (!f_read) return 0;
    f_read(addr, len, buf);
#endif
    return 1;
}

static int s2_flash_program(uint32_t addr, uint32_t len, const uint8_t *buf,
                            uint8_t erase_sector) {
#ifdef HOST_TEST
    if (erase_sector) host_flash_erase(addr);
    host_flash_write(addr, len, buf);
#else
    typedef void (*telink_flash_write_fn)(uint32_t, uint32_t, const uint8_t *);
    telink_flash_write_fn f_write = *(telink_flash_write_fn * volatile)0x0084490cu;
    if (!f_write) return 0;

    /* Stock startup protects flash. Borrow its write path for this operation,
     * and restore protection if it was enabled on entry. */
    uint8_t was_protected = *(volatile uint8_t *)0x00845facu;
    if (was_protected) stock_flash_enable_writes();
    if (erase_sector) stock_flash_erase_sector(addr);
    f_write(addr, len, buf);
    if (was_protected) stock_flash_protect();
#endif
    return 1;
}

#ifdef S2_BOND_DIAGNOSTICS
void s2_save_bond_diagnostics(void) {
    s2_state_t *s = S2_STATE;
    if (s->report_seq < 192u || (s->bond_diagnostics & 0x80u)) return;

    /* Append one checkpoint per boot outside the 32-byte bond record. */
    uint32_t addr = S2_BOND_SECTOR + 0x100u;
    uint8_t log[32];
    uint8_t flags = s->bond_diagnostics;
    s->bond_diagnostics |= 0x80u;
    if (!s2_flash_read_bytes(addr, sizeof(log), log)) return;
    for (unsigned i = 0; i < sizeof(log); i++) {
        if (log[i] == 0xffu) {
            s2_flash_program(addr + i, 1u, &flags, 0u);
            return;
        }
    }
}
#endif

static uint16_t s2_bond_crc(const uint8_t *rec) {
    uint16_t crc = 0xffffu;
    for (unsigned i = 0; i < 28u; i++) {
        crc ^= (uint16_t)rec[i] << 8;
        for (unsigned bit = 0; bit < 8u; bit++) {
            crc = (crc & 0x8000u) ? ((uint32_t)crc << 1) ^ 0x1021u : (uint32_t)crc << 1;
        }
    }
    if (rec[26] == 3u) {
        crc ^= (uint16_t)rec[30] << 8;
        for (unsigned bit = 0; bit < 8u; bit++) {
            crc = (crc & 0x8000u) ? ((uint32_t)crc << 1) ^ 0x1021u : (uint32_t)crc << 1;
        }
    }
    return crc;
}

static int s2_bond_valid(const uint8_t *rec) {
    if (rec[0] != 'S' || rec[1] != '2' || rec[2] != 'B' || rec[3] != 'D') {
        return 0;
    }
    if (rec[26] == 1u) {
        return rec[27] == s2_bond_checksum(rec);
    }
    uint16_t crc = (uint16_t)rec[28] | ((uint16_t)rec[29] << 8);
    return (rec[26] == 2u || rec[26] == 3u) && rec[31] == 0u && crc == s2_bond_crc(rec);
}

static uint8_t s2_bond_generation(const uint8_t *rec) {
    return rec[26] >= 2u ? rec[27] : 0u;
}

static uint8_t s2_rec_has_bond(const uint8_t *rec) {
    return rec[26] < 3u || (rec[30] & S2_CFG_HAS_BOND) != 0u;
}

static uint8_t s2_rec_mode_flags(const uint8_t *rec) {
    return rec[26] == 3u ? (uint8_t)(rec[30] & S2_CFG_MODE_MASK) : 0u;
}

static uint32_t s2_bond_latest(uint8_t *rec) {
    uint8_t backup[S2_BOND_RECORD_SIZE];
    int primary_ok = s2_flash_read_bytes(S2_BOND_SECTOR, S2_BOND_RECORD_SIZE, rec)
                  && s2_bond_valid(rec);
    int backup_ok = s2_flash_read_bytes(S2_BOND_BACKUP_SECTOR, sizeof(backup), backup)
                 && s2_bond_valid(backup);
    if (backup_ok && (!primary_ok ||
        (uint8_t)(s2_bond_generation(backup) - s2_bond_generation(rec)) == 1u)) {
        for (unsigned i = 0; i < sizeof(backup); i++) rec[i] = backup[i];
        return S2_BOND_BACKUP_SECTOR;
    }
    return primary_ok ? S2_BOND_SECTOR : 0u;
}

uint8_t s2_cfg_load(void) {
    uint8_t rec[S2_BOND_RECORD_SIZE];
    if (!s2_bond_latest(rec)) return 0u;
    return s2_rec_mode_flags(rec);
}

void s2_bond_load(void) {
    s2_state_t *s = S2_STATE;
    uint8_t rec[S2_BOND_RECORD_SIZE];
    if (s2_bond_latest(rec) && s2_rec_has_bond(rec)) {
        for (unsigned i = 0; i < sizeof(s->ltk); i++) s->ltk[i] = rec[4 + i];
        for (unsigned i = 0; i < sizeof(s->host_addr); i++) s->host_addr[i] = rec[20 + i];
        s->have_ltk = S2_BOND_VALID;
    }
}

static int s2_bond_same(const uint8_t *rec, const s2_state_t *s) {
    if (!s2_rec_has_bond(rec)) return 0;
    for (unsigned i = 0; i < sizeof(s->ltk); i++) {
        if (rec[4 + i] != s->ltk[i]) return 0;
    }
    for (unsigned i = 0; i < sizeof(s->host_addr); i++) {
        if (rec[20 + i] != s->host_addr[i]) return 0;
    }
    return 1;
}

static int s2_flash_commit_record(uint32_t latest, uint8_t rec[S2_BOND_RECORD_SIZE]) {
    uint8_t cur[S2_BOND_RECORD_SIZE];
    /* Keep the previous valid record in the other sector until this one verifies. */
    uint32_t dest = latest == S2_BOND_SECTOR ? S2_BOND_BACKUP_SECTOR : S2_BOND_SECTOR;
    if (!s2_flash_read_bytes(dest, sizeof(cur), cur)) return 0;
    uint8_t erase_sector = 0u;
    for (unsigned i = 0; i < sizeof(cur); i++) {
        if (cur[i] != 0xffu) {
            erase_sector = 1u;
            break;
        }
    }
    if (!s2_flash_program(dest, S2_BOND_RECORD_SIZE, rec, erase_sector) ||
        !s2_flash_read_bytes(dest, sizeof(cur), cur)) return 0;
    for (unsigned i = 0; i < sizeof(cur); i++) {
        if (cur[i] != rec[i]) return 0;
    }
    return 1;
}

void s2_cfg_save_flags(uint8_t flags) {
    flags &= S2_CFG_MODE_MASK;
    uint8_t cur[S2_BOND_RECORD_SIZE];
    uint32_t latest = s2_bond_latest(cur);
    if (latest && s2_rec_mode_flags(cur) == flags) {
        return;
    }

    uint8_t rec[S2_BOND_RECORD_SIZE];
    for (unsigned i = 0; i < sizeof(rec); i++) rec[i] = 0u;
    rec[0] = 'S'; rec[1] = '2'; rec[2] = 'B'; rec[3] = 'D';
    if (latest && s2_rec_has_bond(cur)) {
        for (unsigned i = 4; i < 26u; i++) rec[i] = cur[i];
        flags |= S2_CFG_HAS_BOND;
    }
    rec[26] = 3u;
    rec[27] = latest ? (uint8_t)(s2_bond_generation(cur) + 1u) : 0u;
    rec[30] = flags;
    uint16_t crc = s2_bond_crc(rec);
    rec[28] = (uint8_t)crc;
    rec[29] = (uint8_t)(crc >> 8);
    s2_flash_commit_record(latest, rec);
}

static void s2_bond_save(void) {
    s2_state_t *s = S2_STATE;
    uint8_t cur[S2_BOND_RECORD_SIZE];
    uint32_t latest = s2_bond_latest(cur);
    if (latest && s2_bond_same(cur, s)) {
        s->have_ltk = S2_BOND_VALID;
        return;
    }

    uint8_t rec[S2_BOND_RECORD_SIZE];
    for (unsigned i = 0; i < sizeof(rec); i++) rec[i] = 0u;
    rec[0] = 'S'; rec[1] = '2'; rec[2] = 'B'; rec[3] = 'D';
    for (unsigned i = 0; i < sizeof(s->ltk); i++) rec[4 + i] = s->ltk[i];
    for (unsigned i = 0; i < sizeof(s->host_addr); i++) rec[20 + i] = s->host_addr[i];
    rec[26] = 3u;
    rec[27] = latest ? (uint8_t)(s2_bond_generation(cur) + 1u) : 0u;
    rec[30] = (uint8_t)((latest ? s2_rec_mode_flags(cur) : 0u) | S2_CFG_HAS_BOND);
    uint16_t crc = s2_bond_crc(rec);
    rec[28] = (uint8_t)crc;
    rec[29] = (uint8_t)(crc >> 8);

#ifdef S2_BOND_DIAGNOSTICS
    s->bond_diagnostics |= 0x20u;  /* Reached the flash programming call. */
#endif
    if (!s2_flash_commit_record(latest, rec)) return;
    s->have_ltk = S2_BOND_VALID;
#ifdef S2_BOND_DIAGNOSTICS
    s->bond_diagnostics |= 0x40u;  /* Readback matched. */
#endif
}

/* 16 MHz system tick shifted by 16 (~244.14 Hz). */
#define S2_WAKE_WINDOW_TICKS ((8000000u * 16u + 65535u) / 65536u)
#define S2_PAIR_WINDOW_TICKS ((60000000u * 16u + 65535u) / 65536u)

static void s2_set_advertising(uint8_t pairing) {
    s2_state_t *s = S2_STATE;
    uint8_t adv[31];
    uint8_t flag = (s->flags & S2_FLAG_WAKE_ACTIVE) ? S2_ADV_WAKE_FLAG : 0u;
    s2_build_adv_data(adv, pairing ? S2_ADV_FORCE_DISCOVERY : flag);
    if (CALL_SET_ADV_DATA(adv, sizeof(adv)) != 0) {
        s->flags |= S2_FLAG_ADV_PENDING;
    } else {
        s->flags &= (uint8_t)~S2_FLAG_ADV_PENDING;
    }
}

void s2_startup_advertising(uint16_t tick, uint8_t pairing) {
    s2_state_t *s = S2_STATE;
    s->window_start_tick = tick;
    s->flags &= (uint8_t)~(S2_FLAG_WAKE_ACTIVE | S2_FLAG_PAIRING_OPEN);
    if (pairing) {
        s->flags |= S2_FLAG_PAIRING_OPEN;
    } else if (s->have_ltk & S2_BOND_VALID) {
        s->flags |= S2_FLAG_WAKE_ACTIVE;
    }
    s2_set_advertising(pairing);
}

void s2_pairing_advertising(uint16_t tick) {
    s2_state_t *s = S2_STATE;
    s->window_start_tick = tick;
    s->flags &= (uint8_t)~S2_FLAG_WAKE_ACTIVE;
    s->flags |= S2_FLAG_PAIRING_OPEN;
    s2_set_advertising(1u);
}

void s2_service_ble(uint16_t tick, uint8_t connected, uint8_t encrypted, uint8_t pairing) {
    s2_state_t *s = S2_STATE;
#ifdef S2_BOND_DIAGNOSTICS
    if (encrypted) s->bond_diagnostics |= 0x08u;
#endif
    uint16_t elapsed = (uint16_t)(tick - s->window_start_tick);
    if ((s->flags & S2_FLAG_WAKE_ACTIVE) && (encrypted || elapsed >= S2_WAKE_WINDOW_TICKS)) {
        s->flags &= (uint8_t)~S2_FLAG_WAKE_ACTIVE;
        if (!connected) s->flags |= S2_FLAG_ADV_PENDING;
    }
    if ((s->flags & S2_FLAG_PAIRING_OPEN) && (encrypted || elapsed >= S2_PAIR_WINDOW_TICKS)) {
        s->flags &= (uint8_t)~S2_FLAG_PAIRING_OPEN;
        s->pairing_stage = 0u;
    }
    if ((s->flags & S2_FLAG_ADV_PENDING) && !connected) s2_set_advertising(pairing);

    /* The console finalizes pairing after checking our LTK challenge response.
     * Commit after the finalize reply enters the BLE TX queue. The console
     * can start link-layer encryption later. */
    if (connected && (s->have_ltk & S2_BOND_READY) && (s->flags & S2_FLAG_BOND_ATTEMPTED) == 0u) {
#ifdef S2_BOND_DIAGNOSTICS
        s->bond_diagnostics |= 0x10u;  /* Save started. */
#endif
        s->flags |= S2_FLAG_BOND_ATTEMPTED;
        s2_bond_save();
    }
}

void s2_on_ble_disconnect(uint16_t tick, uint8_t pairing) {
    s2_state_t *s = S2_STATE;
#ifndef HOST_TEST
    s2_phy_reset_1m();
#endif
    s2_stop_rumble();
    s->enabled_features = 0u;
    s->feature_mask = 0u;
    s->cccd_common[0] = 0u; s->cccd_common[1] = 0u;
    s->cccd_pro2[0]   = 0u; s->cccd_pro2[1]   = 0u;
    s->cccd_resp1[0]  = 0u; s->cccd_resp1[1]  = 0u;
    s->cccd_resp2[0]  = 0u; s->cccd_resp2[1]  = 0u;
    s->cccd_unk1[0]   = 0u; s->cccd_unk1[1]   = 0u;
    s->cccd_unk2[0]   = 0u; s->cccd_unk2[1]   = 0u;
    s->cccd_aud[0]    = 0u; s->cccd_aud[1]    = 0u;
    s->cccd_svc[0]    = 0u; s->cccd_svc[1]    = 0u;
    s->pairing_stage = 0u;
    s->scratch.imu.valid = 0u;
    s->pending_rsp_valid = 0u;
    s->pending_q.valid = 0u;
    if (pairing) {
        s->window_start_tick = tick;
        s->flags &= (uint8_t)~S2_FLAG_WAKE_ACTIVE;
        s->flags |= S2_FLAG_PAIRING_OPEN;
    } else {
        s->flags &= (uint8_t)~S2_FLAG_PAIRING_OPEN;
    }
    s2_set_advertising(pairing);
}

/* =========================================================================
 * Switch 2 native 40-byte (0x28, hdr=0xf0) motion encoder + Q20 quaternion
 * ========================================================================= */

static void s2_put_bits(uint8_t *buf, unsigned start, unsigned width, uint32_t val) {
    uint32_t mask = (1u << width) - 1u;
    val &= mask;
    unsigned byte_idx = start >> 3;
    unsigned shift = start & 7u;
    uint32_t bits = val << shift;
    uint32_t total = shift + width;
    buf[byte_idx] |= (uint8_t)(bits & 0xffu);
    if (total > 8u)  buf[byte_idx + 1u] |= (uint8_t)((bits >> 8) & 0xffu);
    if (total > 16u) buf[byte_idx + 2u] |= (uint8_t)((bits >> 16) & 0xffu);
    if (total > 24u) buf[byte_idx + 3u] |= (uint8_t)((bits >> 24) & 0xffu);
}

static int32_t s2_clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void s2_pack_motion(uint8_t rep[63]) {
    s2_state_t *s = S2_STATE;
    int16_t u_ax, u_ay, u_az, u_gx, u_gy, u_gz;
    CALL_GET_IMU(&u_ax, &u_ay, &u_az, &u_gx, &u_gy, &u_gz);

    /* Rotate U2W IMU frame (+X=USB, +Y=Left, +Z=Up, ±8g, ±2000 dps) into
     * Switch 2 Pro 2 native frame (+X=Right, +Y=USB, +Z=Up, ±8g, ±500 dps). */
    int32_t ax = s2_clamp_i32(-(int32_t)u_ay, -8192, 8191);
    int32_t ay = s2_clamp_i32((int32_t)u_ax, -8192, 8191);
    int32_t az = s2_clamp_i32((int32_t)u_az, -8192, 8191);
    int32_t gx = s2_clamp_i32(-(int32_t)u_gy * 2, -16384, 16383);
    int32_t gy = s2_clamp_i32((int32_t)u_gx * 2, -16384, 16383);
    int32_t gz = s2_clamp_i32((int32_t)u_gz * 2, -16384, 16383);

    uint32_t tick = CALL_GET_TICK();
    uint16_t tick16 = (uint16_t)(tick >> 10); /* 15625 Hz (64 us/tick), wraps cleanly at 2^32 */

    uint16_t dt_ticks = 78u; /* default 5 ms */
    uint16_t t12;
    uint8_t dt8 = 5u;
    if (!s->scratch.imu.valid) {
        s->scratch.imu.q[0] = 1 << 20;
        s->scratch.imu.q[1] = 0;
        s->scratch.imu.q[2] = 0;
        s->scratch.imu.q[3] = 0;
        s->scratch.imu.last_tick16 = tick16;
        t12 = (uint16_t)((tick16 & 0xfffu) | 1u);
        s->scratch.imu.sample_time12 = t12;
        s->scratch.imu.sample_frac = 0u;
        s->scratch.imu.valid = 1u;
    } else {
        dt_ticks = (uint16_t)(tick16 - s->scratch.imu.last_tick16);
        if (dt_ticks == 0u) dt_ticks = 1u;
        if (dt_ticks > 1562u) dt_ticks = 1562u; /* clamp at 100 ms */
        s->scratch.imu.last_tick16 = tick16;

        /* Accumulate ~960 Hz timestamp from wrap-safe 15625 Hz elapsed ticks:
         * 960 / 15625 * 65536 = 4026.53 -> 4027 (0.01% error). */
        uint32_t acc = (uint32_t)s->scratch.imu.sample_frac + (uint32_t)dt_ticks * 4027u;
        uint32_t step = acc >> 16;
        s->scratch.imu.sample_frac = (uint16_t)(acc & 0xffffu);
        s->scratch.imu.sample_time12 = (uint16_t)((s->scratch.imu.sample_time12 + step) & 0xfffu);
        t12 = s->scratch.imu.sample_time12;
        dt8 = step > 255u ? 255u : (uint8_t)step;

        /* Integrate Q20 unit quaternion q = [w, x, y, z] using 32-bit math only.
         * 0.5 * (rad(500)/16384) * (1/15625) * 2^20 = 0.017872 ≈ 146 / 8192. */
        int32_t hx = s2_clamp_i32((((gx * 146) >> 3) * (int32_t)dt_ticks) >> 10, -32767, 32767);
        int32_t hy = s2_clamp_i32((((gy * 146) >> 3) * (int32_t)dt_ticks) >> 10, -32767, 32767);
        int32_t hz = s2_clamp_i32((((gz * 146) >> 3) * (int32_t)dt_ticks) >> 10, -32767, 32767);

        int32_t w = s->scratch.imu.q[0];
        int32_t x = s->scratch.imu.q[1];
        int32_t y = s->scratch.imu.q[2];
        int32_t z = s->scratch.imu.q[3];
        int32_t w15 = w >> 5, x15 = x >> 5, y15 = y >> 5, z15 = z >> 5;

        w += (-x15 * hx - y15 * hy - z15 * hz) >> 15;
        x += ( w15 * hx + y15 * hz - z15 * hy) >> 15;
        y += ( w15 * hy - x15 * hz + z15 * hx) >> 15;
        z += ( w15 * hz + x15 * hy - y15 * hx) >> 15;

        /* One Newton-Raphson renormalization step in Q14: scale = 1.5 - 0.5*norm2 */
        int32_t w14 = w >> 6, x14 = x >> 6, y14 = y >> 6, z14 = z >> 6;
        int32_t norm2_q14 = (w14 * w14 + x14 * x14 + y14 * y14 + z14 * z14) >> 14;
        int32_t scale_q14 = (3 << 13) - (norm2_q14 >> 1);
        s->scratch.imu.q[0] = ((w >> 4) * scale_q14) >> 10;
        s->scratch.imu.q[1] = ((x >> 4) * scale_q14) >> 10;
        s->scratch.imu.q[2] = ((y >> 4) * scale_q14) >> 10;
        s->scratch.imu.q[3] = ((z >> 4) * scale_q14) >> 10;
    }

    /* Smallest-three quaternion encoding: [q0, q1, q2, q3] = [x, y, z, w]. */
    int32_t qc[4] = {
        s->scratch.imu.q[1],
        s->scratch.imu.q[2],
        s->scratch.imu.q[3],
        s->scratch.imu.q[0],
    };
    unsigned omit = 3u;
    int32_t best = qc[3] < 0 ? -qc[3] : qc[3];
    for (unsigned i = 0; i < 3u; i++) {
        int32_t a = qc[i] < 0 ? -qc[i] : qc[i];
        if (a > best) {
            best = a;
            omit = i;
        }
    }
    if (qc[omit] < 0) {
        for (unsigned i = 0; i < 4u; i++) qc[i] = -qc[i];
    }
    /* Selector 0 omits q3 (w), 1 omits q0 (x), 2 omits q1 (y), 3 omits q2 (z). */
    unsigned sel = (omit + 1u) & 3u;

    rep[0x0e] = 0x28u;
    s2_put_bits(rep, 120u, 12u, t12);
    s2_put_bits(rep, 132u, 8u, dt8);
    s2_put_bits(rep, 140u, 14u, 0x30f0u); /* hdr=0xf0, fixed=0x10, bit153=1 */
    s2_put_bits(rep, 154u, 2u, sel);

    for (unsigned k = 0; k < 3u; k++) {
        int32_t c = qc[(sel + k) & 3u];
        /* Scale by sqrt(2) (23170 / 16384) and bias +2^20 into [0, 2^21-1]. */
        int32_t c_s2 = ((c >> 6) * 23170) >> 8;
        int32_t raw = s2_clamp_i32(c_s2 + (1 << 20), 0, (1 << 21) - 1);
        s2_put_bits(rep, 157u + k * 21u, 21u, (uint32_t)raw);
    }

    /* A0 (s14), G2 (s15), A1 (s13 x 2), G3 (s15), A2 (s14) */
    s2_put_bits(rep, 220u, 14u, (uint32_t)ax);
    s2_put_bits(rep, 234u, 14u, (uint32_t)ay);
    s2_put_bits(rep, 248u, 14u, (uint32_t)az);

    s2_put_bits(rep, 263u, 15u, (uint32_t)gx);
    s2_put_bits(rep, 279u, 15u, (uint32_t)gy);
    s2_put_bits(rep, 295u, 15u, (uint32_t)gz);

    s2_put_bits(rep, 310u, 13u, (uint32_t)(ax >> 1));
    s2_put_bits(rep, 323u, 13u, (uint32_t)(ay >> 1));
    s2_put_bits(rep, 336u, 13u, (uint32_t)(az >> 1));

    s2_put_bits(rep, 350u, 15u, (uint32_t)gx);
    s2_put_bits(rep, 366u, 15u, (uint32_t)gy);
    s2_put_bits(rep, 382u, 15u, (uint32_t)gz);

    s2_put_bits(rep, 397u, 14u, (uint32_t)ax);
    s2_put_bits(rep, 411u, 14u, (uint32_t)ay);
    s2_put_bits(rep, 425u, 14u, (uint32_t)az);
}

/* =========================================================================
 * 63-byte Pro Controller 2 input report (0x09 on handle 0x000e)
 * ========================================================================= */

static void pack_u12_stick(uint8_t *dst, uint16_t xv, uint16_t yv) {
    dst[0] = (uint8_t)(xv & 0xffu);
    dst[1] = (uint8_t)(((yv & 0x0fu) << 4) | ((xv >> 8) & 0x0fu));
    dst[2] = (uint8_t)((yv >> 4) & 0xffu);
}

int s2_send_input_report(void) {
    s2_state_t *s = S2_STATE;
    /* H5: retry any queued command response before streaming input reports */
    if (s->pending_rsp_valid != 0u) {
        s2_flush_pending_response();
    }
    if ((s->cccd_pro2[0] & 0x01u) == 0u) {
        return 0;
    }

#ifdef HOST_TEST
    uint32_t btn  = host_get_buttons();
    uint32_t ls   = host_get_lstick_raw();
    uint32_t rs   = host_get_rstick_raw();
    uint16_t trig = host_get_triggers_raw();
#else
    const volatile uint32_t *in = (const volatile uint32_t *)0x00846d44u;
    uint32_t btn  = in[0];
    uint32_t ls   = in[1];
    uint32_t rs   = in[2];
    uint16_t trig = *(const volatile uint16_t *)0x00846d50u;
#endif

    uint8_t rep[63];
    for (int i = 0; i < 63; i++) {
        rep[i] = 0u;
    }

    rep[0] = s->report_seq++;

    /* Power byte: level 0..9 in bits [5:2] (exact /100 via *5243 >> 19 for 0..950) */
    uint8_t pct = CALL_GET_BAT_PCT();
    if (pct > 100u) pct = 100u;
    uint32_t num = (uint32_t)pct * 9u + 50u;
    uint8_t lvl = (uint8_t)((num * 5243u) >> 19);
    if (lvl > 9u) lvl = 9u;
    rep[1] = (uint8_t)(lvl << 2);

    /*
     * Map U2W internal 32-bit button mask (0x846d44, decoded from 0xe26a..0xe394)
     * into Switch 2 Pro Controller report offsets 2, 3, 4:
     *   rep[2]: 0x80 RStick 0x40 Plus 0x20 ZR 0x10 R 0x08 X 0x04 Y 0x02 A 0x01 B
     *   rep[3]: 0x80 LStick 0x40 Minus 0x20 ZL 0x10 L 0x08 Up 0x04 Left 0x02 Right 0x01 Down
     *   rep[4]: 0x10 C 0x08 GL 0x04 GR 0x02 Capture 0x01 Home
     */
    uint8_t b2 = 0u, b3 = 0u, b4 = 0u;

    /* Face buttons (M2 positional: East=A, South=B, North=X, West=Y) */
    if (btn & (1u << 12)) b2 |= 0x02u; /* A (East / Xbox B) */
    if (btn & (1u << 13)) b2 |= 0x01u; /* B (South / Xbox A) */
    if (btn & (1u << 5))  b2 |= 0x08u; /* X (North / Xbox Y) */
    if (btn & (1u << 4))  b2 |= 0x04u; /* Y (West / Xbox X) */

    /* Shoulders, triggers (H4: high byte=ZR/Accel, low byte=ZL/Brake), Plus/Minus, Stick clicks */
    if (btn & (1u << 11)) b2 |= 0x10u; /* R */
    if ((btn & (1u << 15)) || ((trig >> 8) > 0x4du))   b2 |= 0x20u; /* ZR */
    if (btn & (1u << 0))  b2 |= 0x40u; /* Plus */
    if (btn & (1u << 2))  b2 |= 0x80u; /* RStick */

    if (btn & (1u << 10)) b3 |= 0x10u; /* L */
    if ((btn & (1u << 14)) || ((trig & 0xffu) > 0x4du)) b3 |= 0x20u; /* ZL */
    if (btn & (1u << 3))  b3 |= 0x40u; /* Minus */
    if (btn & (1u << 1))  b3 |= 0x80u; /* LStick */

    /* D-pad (H2 verified from 0xe344..0xe49c: 9=Up, 6=Right, 8=Down, 7=Left) */
    if (btn & (1u << 9))  b3 |= 0x08u; /* Up */
    if (btn & (1u << 8))  b3 |= 0x01u; /* Down */
    if (btn & (1u << 7))  b3 |= 0x04u; /* Left */
    if (btn & (1u << 6))  b3 |= 0x02u; /* Right */

    /* System and grip buttons. P2 is the left paddle on the Ultimate 2 Wireless. */
    if (btn & (1u << 17)) b4 |= 0x01u; /* Home */
    if (btn & (1u << 16)) b4 |= 0x02u; /* Capture (Share) */
    if (btn & (1u << 26)) b4 |= 0x08u; /* GL (P2) */
    if (btn & (1u << 25)) b4 |= 0x04u; /* GR (P1) */

    rep[2] = b2;
    rep[3] = b3;
    rep[4] = b4;

    /*
     * Sticks (H3 verified from 0xe396..0xe3c4):
     *   X = byte 1 ((w >> 8) & 0xff), inverted (255 - x)
     *   Y = byte 3 ((w >> 24) & 0xff), inverted for Switch up=+Y (255 - y)
     */
    uint8_t lx8 = (uint8_t)(255u - ((ls >> 8) & 0xffu));
    uint8_t ly8 = (uint8_t)(255u - ((ls >> 24) & 0xffu));
    uint8_t rx8 = (uint8_t)(255u - ((rs >> 8) & 0xffu));
    uint8_t ry8 = (uint8_t)(255u - ((rs >> 24) & 0xffu));

    uint16_t lx12 = (uint16_t)(((uint16_t)lx8 << 4) | (lx8 >> 4));
    uint16_t ly12 = (uint16_t)(((uint16_t)ly8 << 4) | (ly8 >> 4));
    uint16_t rx12 = (uint16_t)(((uint16_t)rx8 << 4) | (rx8 >> 4));
    uint16_t ry12 = (uint16_t)(((uint16_t)ry8 << 4) | (ry8 >> 4));

    pack_u12_stick(&rep[5], lx12, ly12);
    pack_u12_stick(&rep[8], rx12, ry12);

    /* Feature status byte at 0x0b: 0x30 (or 0x38 if rumble bit 0x20 enabled) */
    rep[0x0b] = (s->enabled_features & S2_FEATURE_RUMBLE) ? 0x38u : 0x30u;

    if ((s->enabled_features & S2_FEATURE_MOTION) && s->pairing_stage == 0u) {
        s2_pack_motion(rep);
    }

    return CALL_GATT_NOTIFY(BLS_CONN_HANDLE, S2_H_PRO2_INPUT, rep, 63);
}
