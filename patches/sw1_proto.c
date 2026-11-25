/*
 * Nintendo Switch 1 Pro Controller (057e:2009) BLE HOGP protocol for U2W v1.10:
 *   - Output 0x01 (rumble + subcommand) & Output 0x10 (rumble only) write handlers
 *   - Simulated Pro Controller SPI flash (IMU cal, stick cal, serial, colors)
 *   - Input 0x21 (48-byte subcommand reply with back-to-back retry FIFO)
 *   - Input 0x3F (11-byte simple HID) & Input 0x30 (48-byte full state + 3 IMU frames)
 *   - Switch 1 HD rumble decoding to U2W ERM motors
 *   - Dual-sector flash persistence for universal 4-slot BLE + 2.4G config
 */

#include <stdint.h>
#include "s2_state.h"

#ifdef HOST_TEST
extern int      host_gatt_notify(uint16_t conn_h, uint16_t att_h, const uint8_t *p, int len);
extern void     host_get_local_bdaddr(uint8_t out[6]);
extern uint32_t host_get_buttons(void);
extern uint32_t host_get_raw_buttons(void);
extern uint32_t host_get_lstick_raw(void);
extern uint32_t host_get_rstick_raw(void);
extern uint16_t host_get_triggers_raw(void);
extern uint8_t  host_get_battery_pct(void);
extern void     host_get_imu(int16_t *ax, int16_t *ay, int16_t *az,
                             int16_t *gx, int16_t *gy, int16_t *gz);
extern void     host_flash_read(uint32_t addr, uint32_t len, uint8_t *buf);
extern void     host_flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);
extern void     host_flash_erase(uint32_t addr);
extern void     host_set_rumble(uint8_t left, uint8_t right, uint8_t duration);
#define CALL_GATT_NOTIFY(c, h, p, l)    host_gatt_notify((c), (h), (p), (l))
#define CALL_GET_BAT_PCT()              host_get_battery_pct()
#define CALL_GET_IMU(ax,ay,az,gx,gy,gz) host_get_imu((ax),(ay),(az),(gx),(gy),(gz))
#define CALL_SET_RUMBLE                 host_set_rumble
#else
extern int     stock_gatt_notify(uint16_t conn_h, uint16_t att_h, const uint8_t *p, int len);
extern uint8_t stock_get_battery_pct(void);
extern void    stock_get_imu(int16_t *ax, int16_t *ay, int16_t *az,
                             int16_t *gx, int16_t *gy, int16_t *gz);
extern void    stock_flash_erase_sector(uint32_t addr);
extern void    stock_flash_enable_writes(void);
extern void    stock_flash_protect(void);
extern int     stock_l2cap_rx(uint16_t conn_h, uint8_t *raw);
extern void    stock_set_rumble(uint8_t left, uint8_t right, uint8_t duration);
#define CALL_GATT_NOTIFY(c, h, p, l)    stock_gatt_notify((c), (h), (p), (l))
#define CALL_GET_BAT_PCT()              stock_get_battery_pct()
#define CALL_GET_IMU(ax,ay,az,gx,gy,gz) stock_get_imu((ax),(ay),(az),(gx),(gy),(gz))
#define CALL_SET_RUMBLE                 stock_set_rumble
#endif

/* =========================================================================
 * Hardware input reading & stick/button/battery helpers
 * ========================================================================= */

static void sw1_read_hw_inputs(uint32_t *btn, uint32_t *raw_btn,
                               uint32_t *ls, uint32_t *rs, uint16_t *trig) {
#ifdef HOST_TEST
    *btn     = host_get_buttons();
    *raw_btn = host_get_raw_buttons();
    *ls      = host_get_lstick_raw();
    *rs      = host_get_rstick_raw();
    *trig    = host_get_triggers_raw();
#else
    const volatile uint32_t *in = (const volatile uint32_t *)0x00846d44u;
    *btn     = in[0];
    *raw_btn = *(const volatile uint32_t *)0x00845fe8u;
    *ls      = in[1];
    *rs      = in[2];
    *trig    = *(const volatile uint16_t *)0x00846d50u;
#endif
}

static void sw1_get_local_bdaddr(uint8_t out[6]) {
#ifdef HOST_TEST
    host_get_local_bdaddr(out);
#else
    /* Telink LL public BD_ADDR at 0x008449e4 */
    const volatile uint8_t *mac = (const volatile uint8_t *)0x008449e4u;
    for (int i = 0; i < 6; i++) {
        out[i] = mac[i];
    }
#endif
}

static uint8_t sw1_read_bat_pct(void) {
    uint8_t pct = CALL_GET_BAT_PCT();
    return pct > 100u ? 100u : pct;
}

static uint8_t sw1_bat_conn_byte(uint8_t pct) {
    /* High nibble: 8=full, 6=medium, 4=low, 2=critical, 0=empty; low nibble: 0 (Pro BT). */
    uint8_t step = pct >= 85u ? 8u
                 : (pct >= 60u ? 6u
                 : (pct >= 35u ? 4u
                 : (pct >= 10u ? 2u : 0u)));
    return (uint8_t)(step << 4);
}

static void sw1_pack_stick12(uint8_t dst[3], uint32_t raw) {
    uint8_t x8 = (uint8_t)(255u - ((raw >> 8) & 0xffu));
    uint8_t y8 = (uint8_t)(255u - ((raw >> 24) & 0xffu));
    uint16_t xv = (uint16_t)(((uint16_t)x8 << 4) | (x8 >> 4));
    uint16_t yv = (uint16_t)(((uint16_t)y8 << 4) | (y8 >> 4));
    dst[0] = (uint8_t)(xv & 0xffu);
    dst[1] = (uint8_t)(((yv & 0x0fu) << 4) | ((xv >> 8) & 0x0fu));
    dst[2] = (uint8_t)((yv >> 4) & 0xffu);
}

/* Pack common 12-byte Switch 1 state block (without Report ID):
 *   out[0]:     timer
 *   out[1]:     battery + connection info
 *   out[2..4]:  Right, Shared, Left buttons (physical South=B, East=A, West=Y, North=X)
 *   out[5..7]:  12-bit Left stick
 *   out[8..10]: 12-bit Right stick
 *   out[11]:    vibrator report byte
 */
static void sw1_fill_common_state(uint8_t out[SW1_BODY_LEN]) {
    s2_state_t *s = S2_STATE;
    uint32_t btn, raw_btn, ls, rs;
    uint16_t trig;
    sw1_read_hw_inputs(&btn, &raw_btn, &ls, &rs, &trig);

    uint8_t pct = sw1_read_bat_pct();
    s->bat_level = pct;

    out[0] = s->report_timer++;
    out[1] = sw1_bat_conn_byte(pct);
    out[2] = 0u;
    out[3] = 0u;
    out[4] = 0u;

    /*
     * U2W internal button bits (0x846d44) -> Switch 1 Standard Input Report bytes 3, 4, 5:
     *   out[2] (Right):  0x01 Y(West/X) 0x02 X(North/Y) 0x04 B(South/A) 0x08 A(East/B) 0x40 R 0x80 ZR
     *   out[3] (Shared): 0x01 Minus 0x02 Plus 0x04 RStick 0x08 LStick 0x10 Home 0x20 Capture
     *   out[4] (Left):   0x01 Down 0x02 Up 0x04 Right 0x08 Left 0x40 L 0x80 ZL
     */
    static const struct { uint8_t bit; uint8_t idx; uint8_t mask; } kBtnMap[] = {
        { 13u, 2u, 0x04u }, { 12u, 2u, 0x08u }, {  4u, 2u, 0x01u }, {  5u, 2u, 0x02u }, /* Bottom(A)->B, Right(B)->A, Left(X)->Y, Top(Y)->X */
        { 11u, 2u, 0x40u }, { 15u, 2u, 0x80u }, {  0u, 3u, 0x02u }, {  2u, 3u, 0x04u }, /* R, ZR, Plus, R3 */
        { 10u, 4u, 0x40u }, { 14u, 4u, 0x80u }, {  3u, 3u, 0x01u }, {  1u, 3u, 0x08u }, /* L, ZL, Minus, L3 */
        {  9u, 4u, 0x02u }, {  8u, 4u, 0x01u }, {  7u, 4u, 0x08u }, {  6u, 4u, 0x04u }, /* U D L R */
        { 17u, 3u, 0x10u }, { 16u, 3u, 0x20u },                                         /* Home, Capture */
    };
    for (unsigned i = 0; i < sizeof(kBtnMap) / sizeof(kBtnMap[0]); i++) {
        if (btn & (1u << kBtnMap[i].bit)) out[kBtnMap[i].idx] |= kBtnMap[i].mask;
    }
    if ((trig >> 8) > 0x4du)   out[2] |= 0x80u; /* Analog ZR */
    if ((trig & 0xffu) > 0x4du) out[4] |= 0x80u; /* Analog ZL */
    if (raw_btn & (1u << 27))  out[3] |= 0x20u; /* Physical Square (Turbo) -> Capture */

    sw1_pack_stick12(&out[5], ls);
    sw1_pack_stick12(&out[8], rs);
    out[11] = s->vib_enabled ? (uint8_t)(0x80u | ((s->vib_cnt++ & 3u) << 4)) : 0x00u;
}

/* =========================================================================
 * Simulated Switch 1 Pro Controller SPI Flash (Subcommand 0x10)
 * ========================================================================= */

/* Serial "8BD0U2WSW1000001" */
static const uint8_t kSw1Serial[16] = {
    '8','B','D','0','U','2','W','S','W','1','0','0','0','0','0','1'
};

/*
 * Factory IMU cal @ 0x6020..0x6037 (24B):
 *   Accel origin (0,0,0), Accel coeff (0x4000,0x4000,0x4000 -> +/-8g @ 4096 LSB/g)
 *   Gyro origin  (0,0,0), Gyro coeff  (0x343B,0x343B,0x343B -> matches 2000 dps)
 */
static const uint8_t kSw1ImuCal[24] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3b, 0x34, 0x3b, 0x34, 0x3b, 0x34,
};

/*
 * Factory Stick cal @ 0x603D..0x604E (18B):
 *   center = 0x800, max = 0x7FF, min = 0x7FF. On macOS, JoyConHIDServicePlugin
 *   overrides Pro Controller (0x03) stick range to 0x04D4 (1236) unless
 *   (max - min) <= 1, where its quirk path restores min(maxX, maxY) = 0x7FF
 *   (2047) so full-scale 0..4095 sticks aren't 1.66x over-amplified to 20%.
 */
static const uint8_t kSw1StickCal[18] = {
    /* Left:  Xmax=0x7FF, Ymax=0x7FF, Xctr=0x800, Yctr=0x800, Xmin=0x7FF, Ymin=0x7FF */
    0xff, 0xf7, 0x7f, 0x00, 0x08, 0x80, 0xff, 0xf7, 0x7f,
    /* Right: Xctr=0x800, Yctr=0x800, Xmin=0x7FF, Ymin=0x7FF, Xmax=0x7FF, Ymax=0x7FF */
    0x00, 0x08, 0x80, 0xff, 0xf7, 0x7f, 0xff, 0xf7, 0x7f,
};

/* Colors @ 0x6050..0x605B: dark charcoal body/grips, light grey buttons */
static const uint8_t kSw1Colors[12] = {
    0x32, 0x32, 0x32,
    0xe6, 0xe6, 0xe6,
    0x46, 0x46, 0x46,
    0x46, 0x46, 0x46,
};

static const uint8_t kSw1FactoryHdr6080[6] = {
    0x50, 0xfd, 0x00, 0x00, 0xc6, 0x0f,
};

/* Switch Pro stick parameters (inner deadzone 0x001 for 1:1 raw throw on macOS) */
static const uint8_t kSw1StickParams[18] = {
    0x0f, 0x30, 0x61, 0x01, 0x30, 0xf3, 0xd4, 0x14, 0x54,
    0x41, 0x15, 0x54, 0xc7, 0x79, 0x9c, 0x33, 0x36, 0x63,
};

static uint8_t sw1_sim_spi_byte(uint32_t addr) {
    if (addr >= 0x6000u && addr < 0x6010u) return kSw1Serial[addr - 0x6000u];
    if (addr == 0x6012u) return 0x03u; /* Pro Controller */
    if (addr == 0x601bu) return 0x01u; /* SPI colors valid */
    if (addr >= 0x6020u && addr < 0x6038u) return kSw1ImuCal[addr - 0x6020u];
    if (addr >= 0x603du && addr < 0x604fu) return kSw1StickCal[addr - 0x603du];
    if (addr >= 0x6050u && addr < 0x605cu) return kSw1Colors[addr - 0x6050u];
    if (addr >= 0x6080u && addr < 0x6086u) return kSw1FactoryHdr6080[addr - 0x6080u];
    if (addr >= 0x6086u && addr < 0x6098u) return kSw1StickParams[addr - 0x6086u];
    if (addr >= 0x6098u && addr < 0x60aau) return kSw1StickParams[addr - 0x6098u];
    return 0xffu;
}

/* =========================================================================
 * Switch 1 HD Rumble decoder -> U2W ERM motors
 * ========================================================================= */

#define SW1_RUMBLE_HOLD_TICKS 25u /* 250 ms hold at stock 10 ms motor tick */

static void sw1_stop_rumble(void) {
    if (S2_STATE->flags & SW1_FLAG_RUMBLE_ACTIVE) {
        CALL_SET_RUMBLE(0u, 0u, 1u);
        S2_STATE->flags &= (uint8_t)~SW1_FLAG_RUMBLE_ACTIVE;
    }
}

static uint8_t sw1_step_to_motor(uint8_t step) {
    if (step == 0u) return 0u;
    if (step > 100u) step = 100u;
    uint16_t level = step <= 64u
        ? (uint16_t)(step * 2u)
        : (uint16_t)(128u + (((uint32_t)(step - 64u) * 910u) >> 8));
    return level > 255u ? 255u : (uint8_t)level;
}

static uint8_t sw1_decode_motor_side(const uint8_t b[4]) {
    uint8_t hf_step = (b[1] & 0xfeu) >> 1;
    uint8_t lf_step = 0u;
    if (b[3] >= 0x40u) {
        uint16_t s = (uint16_t)((b[3] - 0x40u) * 2u) + ((b[2] >> 7) & 1u);
        lf_step = s > 100u ? 100u : (uint8_t)s;
    }
    uint16_t sum = (uint16_t)sw1_step_to_motor(hf_step) + (uint16_t)sw1_step_to_motor(lf_step);
    return sum > 255u ? 255u : (uint8_t)sum;
}

static void sw1_apply_rumble(const uint8_t rumble[8], uint8_t allow_stop) {
    if (!S2_STATE->vib_enabled) {
        sw1_stop_rumble();
        return;
    }
    uint8_t left  = sw1_decode_motor_side(&rumble[0]);
    uint8_t right = sw1_decode_motor_side(&rumble[4]);
    if (left || right) {
        CALL_SET_RUMBLE(left, right, SW1_RUMBLE_HOLD_TICKS);
        S2_STATE->flags |= SW1_FLAG_RUMBLE_ACTIVE;
    } else if (allow_stop) {
        sw1_stop_rumble();
    }
}

/* =========================================================================
 * Subcommand replies (Input 0x21) & retry FIFO
 * ========================================================================= */

static int sw1_send_reply_now(const sw1_pending_rsp_t *r) {
    s2_state_t *s = S2_STATE;
    if ((s->cccd_in21[0] & 0x01u) == 0u) {
        return -1;
    }

    uint8_t out[SW1_BODY_LEN];
    for (unsigned i = 0; i < SW1_BODY_LEN; i++) {
        out[i] = 0u;
    }
    sw1_fill_common_state(out);

    uint8_t ack = 0x80u;
    out[13] = r->subcmd;

    if (r->subcmd == 0x02u) { /* Request Device Info */
        ack = 0x82u;
        out[14] = 0x03u; out[15] = 0x48u; /* FW 3.72 */
        out[16] = 0x03u;                 /* Pro Controller */
        out[17] = 0x02u;
        uint8_t mac[6];
        sw1_get_local_bdaddr(mac);
        for (unsigned i = 0; i < 6u; i++) {
            out[18u + i] = mac[5u - i];  /* Big-endian MAC */
        }
        out[24] = 0x01u;
        out[25] = 0x01u;                 /* Colors stored in SPI flash */
    } else if (r->subcmd == 0x04u) {     /* Trigger buttons elapsed time */
        ack = 0x83u;
    } else if (r->subcmd == 0x10u) {     /* SPI flash read */
        ack = 0x90u;
        uint8_t rlen = r->rlen > 29u ? 29u : r->rlen;
        out[14] = (uint8_t)(r->addr & 0xffu);
        out[15] = (uint8_t)((r->addr >> 8) & 0xffu);
        out[16] = (uint8_t)((r->addr >> 16) & 0xffu);
        out[17] = (uint8_t)((r->addr >> 24) & 0xffu);
        out[18] = rlen;
        for (uint8_t i = 0; i < rlen; i++) {
            out[19u + i] = sw1_sim_spi_byte(r->addr + i);
        }
    } else if (r->subcmd == 0x31u) {     /* Get player lights */
        ack = 0xb0u;
        out[14] = r->arg0;
    } else if (r->subcmd == 0x50u) {     /* Get regulated voltage */
        ack = 0xd0u;
        out[14] = 0x18u; out[15] = 0x06u; /* 1560 -> ~3.9 V */
    } else if (r->subcmd == 0x52u) {     /* Get GPIO pin value */
        ack = 0xd1u;
        out[14] = 0x04u;
    }

    out[12] = ack;
    return CALL_GATT_NOTIFY(BLS_CONN_HANDLE, SW1_H_IN21, out, SW1_BODY_LEN);
}

void sw1_flush_pending_replies(void) {
    s2_state_t *s = S2_STATE;
    while (s->q_count > 0u) {
        sw1_pending_rsp_t *r = &s->pending_q[s->q_head & 3u];
        if (sw1_send_reply_now(r) != 0) {
            break;
        }
        r->used = 0u;
        s->q_head = (uint8_t)((s->q_head + 1u) & 3u);
        s->q_count--;
    }
}

static void sw1_dispatch_reply(const sw1_pending_rsp_t *r) {
    s2_state_t *s = S2_STATE;
    if (s->q_count > 0u) {
        sw1_flush_pending_replies();
    }
    if (s->q_count == 0u && sw1_send_reply_now(r) == 0) {
        return;
    }
    if (s->q_count >= 4u) {
        return; /* Ring full: preserve earlier pending replies */
    }
    s->pending_q[s->q_tail & 3u] = *r;
    s->pending_q[s->q_tail & 3u].used = 1u;
    s->q_tail = (uint8_t)((s->q_tail + 1u) & 3u);
    s->q_count++;
}

static void sw1_handle_subcmd(uint8_t subcmd, const uint8_t *args, uint16_t arg_len) {
    s2_state_t *s = S2_STATE;
    sw1_pending_rsp_t rsp = { 0u, subcmd, 0u, 0u, 1u };

    if (subcmd == 0x03u && arg_len >= 1u) { /* Set input report mode */
        s->input_mode = args[0];
        s->has_last_simple = 0u;
    } else if (subcmd == 0x10u && arg_len >= 5u) { /* SPI flash read */
        rsp.addr = (uint32_t)args[0]
                 | ((uint32_t)args[1] << 8)
                 | ((uint32_t)args[2] << 16)
                 | ((uint32_t)args[3] << 24);
        rsp.rlen = args[4] > 29u ? 29u : args[4];
    } else if (subcmd == 0x30u && arg_len >= 1u) { /* Set player lights */
        s->player_lights = args[0];
    } else if (subcmd == 0x31u) { /* Get player lights */
        rsp.arg0 = s->player_lights;
    } else if (subcmd == 0x40u && arg_len >= 1u) { /* Enable IMU */
        s->imu_enabled = args[0] ? 1u : 0u;
    } else if (subcmd == 0x48u && arg_len >= 1u) { /* Enable vibration */
        s->vib_enabled = args[0] ? 1u : 0u;
        if (!s->vib_enabled) sw1_stop_rumble();
    }

    sw1_dispatch_reply(&rsp);
}

/* Telink GATT write callbacks: pkt[2..3] = l2cap_len (3 + val_len), pkt + 9 = value */
static int sw1_read_att_write(void *p, const uint8_t **val_out, uint16_t *len_out) {
    const uint8_t *pkt = (const uint8_t *)p;
    uint16_t l2cap_len = (uint16_t)pkt[2] | ((uint16_t)pkt[3] << 8);
    if (l2cap_len <= 3u || l2cap_len > 162u) {
        return 0;
    }
    *val_out = pkt + 9u;
    *len_out = (uint16_t)(l2cap_len - 3u);
    return 1;
}

int sw1_out01_write_cb(void *p) {
    const uint8_t *val;
    uint16_t len;
    if (!sw1_read_att_write(p, &val, &len)) {
        return 0;
    }
    if (len >= 9u) {
        sw1_apply_rumble(&val[1], 0u);
    }
    if (len >= 10u) {
        sw1_handle_subcmd(val[9], &val[10], (uint16_t)(len - 10u));
    }
    return 0;
}

int sw1_out10_write_cb(void *p) {
    const uint8_t *val;
    uint16_t len;
    if (!sw1_read_att_write(p, &val, &len)) {
        return 0;
    }
    if (len >= 9u) {
        sw1_apply_rumble(&val[1], 1u);
    }
    return 0;
}

/* =========================================================================
 * Input reports: 0x3F (simple HID), 0x30 (full state + IMU), Battery (0x2A19)
 * ========================================================================= */

static void sw1_put_i16(uint8_t *dst, int16_t v) {
    uint16_t u = (uint16_t)v;
    dst[0] = (uint8_t)(u & 0xffu);
    dst[1] = (uint8_t)(u >> 8);
}

static uint8_t sw1_dpad_hat(uint32_t btn) {
    uint8_t u = (btn & (1u << 9)) != 0u;
    uint8_t d = (btn & (1u << 8)) != 0u;
    uint8_t l = (btn & (1u << 7)) != 0u;
    uint8_t r = (btn & (1u << 6)) != 0u;
    if (u && !d) return r ? 1u : (l ? 7u : 0u);
    if (d && !u) return r ? 3u : (l ? 5u : 4u);
    if (r && !l) return 2u;
    if (l && !r) return 6u;
    return 8u; /* Centered */
}

static int sw1_send_simple_report(void) {
    s2_state_t *s = S2_STATE;
    if ((s->cccd_in3f[0] & 0x01u) == 0u) {
        return 0;
    }

    uint32_t btn, raw_btn, ls, rs;
    uint16_t trig;
    sw1_read_hw_inputs(&btn, &raw_btn, &ls, &rs, &trig);

    uint8_t out[SW1_SIMPLE_BODY_LEN];
    out[0] = 0u;
    out[1] = 0u;
    if (btn & (1u << 13)) out[0] |= 0x01u; /* B (South / physical bottom A) */
    if (btn & (1u << 12)) out[0] |= 0x02u; /* A (East / physical right B) */
    if (btn & (1u <<  4)) out[0] |= 0x04u; /* Y (West / physical left X) */
    if (btn & (1u <<  5)) out[0] |= 0x08u; /* X (North / physical top Y) */
    if (btn & (1u << 10)) out[0] |= 0x10u; /* L */
    if (btn & (1u << 11)) out[0] |= 0x20u; /* R */
    if ((btn & (1u << 14)) || (trig & 0xffu) > 0x4du) out[0] |= 0x40u; /* ZL */
    if ((btn & (1u << 15)) || (trig >> 8) > 0x4du)    out[0] |= 0x80u; /* ZR */

    if (btn & (1u <<  3)) out[1] |= 0x01u; /* Minus */
    if (btn & (1u <<  0)) out[1] |= 0x02u; /* Plus */
    if (btn & (1u <<  1)) out[1] |= 0x04u; /* LStick */
    if (btn & (1u <<  2)) out[1] |= 0x08u; /* RStick */
    if (btn & (1u << 17)) out[1] |= 0x10u; /* Home */
    if ((btn & (1u << 16)) || (raw_btn & (1u << 27))) out[1] |= 0x20u; /* Capture */

    out[2] = sw1_dpad_hat(btn);

    /* Simple mode 16-bit unsigned sticks: 0..65535 (X: right=+, Y: down=+ for HID) */
    uint8_t lx8 = (uint8_t)(255u - ((ls >> 8) & 0xffu));
    uint8_t ly8 = (uint8_t)((ls >> 24) & 0xffu);
    uint8_t rx8 = (uint8_t)(255u - ((rs >> 8) & 0xffu));
    uint8_t ry8 = (uint8_t)((rs >> 24) & 0xffu);
    out[3]  = lx8; out[4]  = lx8;
    out[5]  = ly8; out[6]  = ly8;
    out[7]  = rx8; out[8]  = rx8;
    out[9]  = ry8; out[10] = ry8;

    uint8_t changed = !s->has_last_simple;
    for (unsigned i = 0; i < SW1_SIMPLE_BODY_LEN; i++) {
        if (s->last_simple[i] != out[i]) {
            changed = 1u;
            break;
        }
    }
    if (!changed && (++s->simple_idle_div & 0x1fu) != 0u) {
        return 0;
    }

    for (unsigned i = 0; i < SW1_SIMPLE_BODY_LEN; i++) {
        s->last_simple[i] = out[i];
    }
    s->has_last_simple = 1u;
    return CALL_GATT_NOTIFY(BLS_CONN_HANDLE, SW1_H_IN3F, out, SW1_SIMPLE_BODY_LEN);
}

void sw1_notify_battery(void) {
    s2_state_t *s = S2_STATE;
    uint8_t pct = sw1_read_bat_pct();
    s->bat_level = pct;
    if ((s->cccd_bat[0] & 0x01u) && pct != s->last_bat_pct) {
        if (CALL_GATT_NOTIFY(BLS_CONN_HANDLE, SW1_H_BAT_LVL, &s->bat_level, 1) == 0) {
            s->last_bat_pct = pct;
        }
    }
}

int sw1_send_input_report(void) {
    s2_state_t *s = S2_STATE;
    if (s->q_count > 0u) {
        sw1_flush_pending_replies();
    }

    if (s->input_mode == 0x3fu) {
        return sw1_send_simple_report();
    }

    if ((s->cccd_in30[0] & 0x01u) == 0u) {
        return 0;
    }

    uint8_t rep[SW1_BODY_LEN];
    sw1_fill_common_state(rep);

    if (s->imu_enabled) {
        int16_t ax, ay, az, gx, gy, gz;
        CALL_GET_IMU(&ax, &ay, &az, &gx, &gy, &gz);
        for (unsigned frame = 0; frame < 3u; frame++) {
            uint8_t *dst = &rep[12u + frame * 12u];
            sw1_put_i16(&dst[0],  ax);
            sw1_put_i16(&dst[2],  ay);
            sw1_put_i16(&dst[4],  az);
            sw1_put_i16(&dst[6],  gx);
            sw1_put_i16(&dst[8],  gy);
            sw1_put_i16(&dst[10], gz);
        }
    } else {
        for (unsigned i = 12u; i < SW1_BODY_LEN; i++) {
            rep[i] = 0u;
        }
    }

    return CALL_GATT_NOTIFY(BLS_CONN_HANDLE, SW1_H_IN30, rep, SW1_BODY_LEN);
}

/* =========================================================================
 * BLE lifecycle hooks & L2CAP MTU clamp
 * ========================================================================= */

int s2_l2cap_rx_tap(uint16_t conn_h, uint8_t *raw) {
#ifdef HOST_TEST
    (void)conn_h;
    (void)raw;
    return 0;
#else
    int r = stock_l2cap_rx(conn_h, raw);
    if (*(volatile uint16_t *)0x00844d8eu > 158u) {
        *(volatile uint16_t *)0x00844d8eu = 158u;
    }
    return r;
#endif
}

static void sw1_reset_session_state(s2_state_t *s) {
#ifndef HOST_TEST
    s2_phy_reset_1m();
#endif
    s->input_mode = 0x3fu;
    s->imu_enabled = 0u;
    s->q_head = 0u;
    s->q_tail = 0u;
    s->q_count = 0u;
    s->has_last_simple = 0u;
    for (unsigned i = 0; i < 4u; i++) {
        s->pending_q[i].used = 0u;
    }
}

void sw1_on_ble_connect(void) {
    s2_state_t *s = S2_STATE;
    sw1_reset_session_state(s);
    s->vib_enabled = 1u;
    s->proto_mode = 0x01u;
    s->simple_idle_div = 0u;
    s->bat_level = sw1_read_bat_pct();
    s->last_bat_pct = 0xffu;
}

void sw1_on_ble_disconnect(void) {
    s2_state_t *s = S2_STATE;
    sw1_stop_rumble();
    for (unsigned i = 0; i < 10u; i++) {
        (&s->cccd_in30[0])[i] = 0u;
    }
    sw1_reset_session_state(s);
}

/* =========================================================================
 * Dual-sector flash persistence (S2BD v6 layout, preserves existing slots)
 * ========================================================================= */

#define S2_BOND_SECTOR        0x0002b000u
#define S2_BOND_BACKUP_SECTOR 0x0002c000u
#define S2_BOND_RECORD_SIZE   108u
#define S2_MAC_GEN_OFF        32u
#define S2_MAC_GEN_COUNT      6u
#define S2_SLOT_MODES_OFF     38u
#define S2_BOND_MASK_OFF      39u
#define S2_LAST_SLOTS_OFF     40u

static unsigned s2_rec_mac_count(uint8_t version) {
    return version == 4u ? 5u : (version >= 5u ? S2_MAC_GEN_COUNT : 0u);
}

static unsigned s2_rec_crc_extra_len(uint8_t version) {
    return version >= 6u ? (S2_BOND_RECORD_SIZE - S2_MAC_GEN_OFF) : s2_rec_mac_count(version);
}

static unsigned s2_slot_bond_off(uint8_t slot) {
    slot &= 3u;
    return slot == 3u ? 4u : (42u + (unsigned)slot * 22u);
}

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

    uint8_t was_protected = *(volatile uint8_t *)0x00845facu;
    if (was_protected) stock_flash_enable_writes();
    for (uint8_t i = 0; i < erase_sector; i++) {
        stock_flash_erase_sector(addr + ((uint32_t)i << 12));
    }
    if (len) f_write(addr, len, buf);
    if (was_protected) stock_flash_protect();
#endif
    return 1;
}

static uint16_t s2_crc16_byte(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (unsigned bit = 0; bit < 8u; bit++) {
        crc = (crc & 0x8000u) ? (uint16_t)(((uint32_t)crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t s2_bond_crc(const uint8_t *rec) {
    uint16_t crc = 0xffffu;
    for (unsigned i = 0; i < 28u; i++) {
        crc = s2_crc16_byte(crc, rec[i]);
    }
    if (rec[26] >= 3u) {
        crc = s2_crc16_byte(crc, rec[30]);
    }
    for (unsigned i = 0; i < s2_rec_crc_extra_len(rec[26]); i++) {
        crc = s2_crc16_byte(crc, rec[S2_MAC_GEN_OFF + i]);
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
    return (rec[26] >= 2u && rec[26] <= 6u) && rec[31] == 0u && crc == s2_bond_crc(rec);
}

static uint8_t s2_bond_generation(const uint8_t *rec) {
    return rec[26] >= 2u ? rec[27] : 0u;
}

static uint8_t s2_rec_has_bond(const uint8_t *rec) {
    return rec[26] < 3u || (rec[30] & S2_CFG_HAS_BOND) != 0u;
}

static uint8_t s2_rec_mode_flags(const uint8_t *rec) {
    if (rec[26] < 3u) return 0u;
    return rec[30] & (rec[26] < 5u ? (S2_CFG_MODE_MASK & ~S2_CFG_BT_ELITE) : S2_CFG_MODE_MASK);
}

static uint8_t s2_rec_mac_gen(const uint8_t *rec, uint8_t slot) {
    if (slot < s2_rec_mac_count(rec[26])) {
        return rec[S2_MAC_GEN_OFF + slot];
    }
    return 0u;
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

static void s2_rec_decode(const uint8_t *rec, int has_rec, s2_bt_boot_t *b) {
    uint8_t flags = has_rec ? s2_rec_mode_flags(rec) : 0u;
    uint8_t slot_modes = S2_DEFAULT_SLOT_MODES;
    uint8_t bond_mask = 0u;
    uint8_t last_slots = S2_DEFAULT_LAST_MODE_SLOTS;

    if (has_rec) {
        if (rec[26] >= 6u) {
            slot_modes = rec[S2_SLOT_MODES_OFF];
            bond_mask = rec[S2_BOND_MASK_OFF] & 0x0fu;
            last_slots = rec[S2_LAST_SLOTS_OFF];
        } else {
            if (s2_rec_has_bond(rec)) bond_mask = (1u << 3);
            uint8_t bt_mode = flags & (S2_CFG_BT_STOCK | S2_CFG_BT_ELITE);
            uint8_t slot = (uint8_t)((flags & S2_CFG_BT_SLOT_MASK) >> S2_CFG_BT_SLOT_SHIFT);
            if (bt_mode == 0u) {
                slot = 3u;
            } else if (bt_mode == S2_CFG_BT_ELITE) {
                slot = 2u;
            } else {
                slot_modes = s2_slot_mode_set(slot_modes, slot, S2_SLOT_MODE_STOCK);
                last_slots = s2_slot_mode_set(last_slots, S2_SLOT_MODE_STOCK, slot);
            }
            flags = (uint8_t)((flags & (uint8_t)~S2_CFG_BT_SLOT_MASK)
                              | (uint8_t)((slot << S2_CFG_BT_SLOT_SHIFT) & S2_CFG_BT_SLOT_MASK));
        }
    }

    b->cfg = flags;
    b->slot_modes = slot_modes;
    b->s2_bond_mask = bond_mask;
    b->last_mode_slots = last_slots;
    b->slot = (uint8_t)((flags & S2_CFG_BT_SLOT_MASK) >> S2_CFG_BT_SLOT_SHIFT);
    b->mode = s2_slot_mode_get(slot_modes, b->slot);
    b->mac_gen = has_rec ? s2_rec_mac_gen(rec, b->slot) : 0u;
}

void s2_cfg_load_bt(s2_bt_boot_t *b) {
    uint8_t rec[S2_BOND_RECORD_SIZE];
    s2_rec_decode(rec, s2_bond_latest(rec) != 0u, b);
}

uint8_t s2_cfg_load(void) {
    s2_bt_boot_t b;
    s2_cfg_load_bt(&b);
    return b.cfg;
}

static int s2_flash_commit_record(uint32_t latest, uint8_t rec[S2_BOND_RECORD_SIZE]) {
    uint8_t cur[S2_BOND_RECORD_SIZE];
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

static void s2_copy_bonds_from_cur(uint8_t rec[S2_BOND_RECORD_SIZE], uint32_t latest,
                                   const uint8_t *cur, uint8_t bond_mask) {
    if (!latest) return;
    for (uint8_t s = 0u; s < 4u; s++) {
        if ((bond_mask & (uint8_t)(1u << s)) && (s == 3u || cur[26] >= 6u)) {
            unsigned off = s2_slot_bond_off(s);
            for (unsigned i = 0u; i < 22u; i++) rec[off + i] = cur[off + i];
        }
    }
}

static int s2_record_commit(uint32_t latest, const uint8_t *cur, uint8_t rec[S2_BOND_RECORD_SIZE],
                            const s2_bt_boot_t *b, uint8_t bump_mac) {
    rec[0] = 'S'; rec[1] = '2'; rec[2] = 'B'; rec[3] = 'D';
    for (unsigned i = 0; i < S2_MAC_GEN_COUNT; i++) {
        rec[S2_MAC_GEN_OFF + i] = latest ? s2_rec_mac_gen(cur, (uint8_t)i) : 0u;
    }
    if (bump_mac && b->slot < S2_MAC_GEN_COUNT) {
        rec[S2_MAC_GEN_OFF + b->slot] = b->mac_gen;
    }
    rec[S2_SLOT_MODES_OFF] = b->slot_modes;
    rec[S2_BOND_MASK_OFF] = b->s2_bond_mask & 0x0fu;
    rec[S2_LAST_SLOTS_OFF] = b->last_mode_slots;
    rec[26] = 6u;
    rec[27] = latest ? (uint8_t)(s2_bond_generation(cur) + 1u) : 0u;
    rec[30] = (uint8_t)((b->cfg & S2_CFG_MODE_MASK) | ((b->s2_bond_mask & (1u << 3)) ? S2_CFG_HAS_BOND : 0u));
    uint16_t crc = s2_bond_crc(rec);
    rec[28] = (uint8_t)crc;
    rec[29] = (uint8_t)(crc >> 8);
    return s2_flash_commit_record(latest, rec);
}

int s2_cfg_save_bt(s2_bt_boot_t *b, uint8_t bump_mac) {
    uint8_t cur[S2_BOND_RECORD_SIZE];
    uint32_t latest = s2_bond_latest(cur);
    uint8_t gen = latest ? s2_rec_mac_gen(cur, b->slot) : 0u;
    if (bump_mac && b->slot < S2_MAC_GEN_COUNT) {
        gen = (uint8_t)(gen + 1u);
    }
    b->mac_gen = gen;
    uint8_t flags = b->cfg & S2_CFG_MODE_MASK;
    uint8_t mask = b->s2_bond_mask & 0x0fu;
    if (!bump_mac && latest && cur[26] >= 6u &&
        s2_rec_mode_flags(cur) == flags && cur[S2_SLOT_MODES_OFF] == b->slot_modes &&
        (cur[S2_BOND_MASK_OFF] & 0x0fu) == mask && cur[S2_LAST_SLOTS_OFF] == b->last_mode_slots) {
        return 1;
    }

    uint8_t rec[S2_BOND_RECORD_SIZE];
    for (unsigned i = 0; i < sizeof(rec); i++) rec[i] = 0u;
    s2_copy_bonds_from_cur(rec, latest, cur, mask);
    return s2_record_commit(latest, cur, rec, b, bump_mac);
}

void s2_cfg_save_flags(uint8_t flags) {
    s2_bt_boot_t b;
    s2_cfg_load_bt(&b);
    b.cfg = flags & S2_CFG_MODE_MASK;
    b.slot = (uint8_t)((b.cfg & S2_CFG_BT_SLOT_MASK) >> S2_CFG_BT_SLOT_SHIFT);
    (void)s2_cfg_save_bt(&b, 0u);
}

uint8_t s2_resolve_bt_boot(s2_bt_boot_t *b, uint8_t dpad, uint8_t face) {
    uint8_t slot = (uint8_t)((b->cfg & S2_CFG_BT_SLOT_MASK) >> S2_CFG_BT_SLOT_SHIFT);
    uint8_t slot_modes = b->slot_modes;
    uint8_t mode = s2_slot_mode_get(slot_modes, slot);
    uint8_t mode_changed = 0u;

    if (dpad < 4u) {
        slot = dpad;
        mode = s2_slot_mode_get(slot_modes, slot);
        if (face <= S2_SLOT_MODE_ELITE && face != mode) {
            mode = face;
            slot_modes = s2_slot_mode_set(slot_modes, slot, mode);
            mode_changed = 1u;
        }
    } else if (face <= S2_SLOT_MODE_ELITE && mode != face) {
        uint8_t pref = (uint8_t)((b->last_mode_slots >> (face * 2u)) & 3u);
        if (s2_slot_mode_get(slot_modes, pref) == face) {
            slot = pref;
        } else {
            uint8_t i = 0u;
            for (; i < 4u; i++) {
                if (s2_slot_mode_get(slot_modes, i) == face) {
                    slot = i;
                    break;
                }
            }
            if (i == 4u) {
                slot_modes = s2_slot_mode_set(slot_modes, slot, face);
                mode_changed = 1u;
            }
        }
        mode = face;
    }

    uint8_t bt_bits = mode == S2_SLOT_MODE_STOCK ? S2_CFG_BT_STOCK
                    : (mode == S2_SLOT_MODE_ELITE ? S2_CFG_BT_ELITE : 0u);
    b->cfg = (uint8_t)((b->cfg & (uint8_t)~(S2_CFG_BT_STOCK | S2_CFG_BT_ELITE | S2_CFG_BT_SLOT_MASK))
                       | bt_bits
                       | (uint8_t)((slot << S2_CFG_BT_SLOT_SHIFT) & S2_CFG_BT_SLOT_MASK));
    b->slot_modes = slot_modes;
    b->last_mode_slots = s2_slot_mode_set(b->last_mode_slots, mode, slot);
    b->slot = slot;
    b->mode = mode;
    return mode_changed;
}

void s2_erase_smp_slot(uint32_t addr) {
#ifdef HOST_TEST
    (void)addr;
#else
    (void)s2_flash_program(addr, 0u, 0, 2u);
#endif
}

void s2_derive_slot_mac(uint8_t mac[6], uint8_t bt_slot, uint8_t mac_gen) {
    mac[0] = (uint8_t)(mac[0] + mac_gen);
    if (bt_slot > 0u && bt_slot < 4u) {
        mac[1] ^= (uint8_t)(bt_slot << 4);
    }
}
