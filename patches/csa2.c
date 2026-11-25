/*
 * U2W v1.10 Switch 1 BLE HOGP hooks & CSA #2 channel selection.
 *
 * Placed in the 12 KiB zero cave at flash 0x0a040..0x0d000 (inside the
 * bootloader-covered Telink code region [0x5000, 0x1ca60)).
 */

#include <stdint.h>
#include "s2_state.h"

/*
 * Resolve an LL_PHY_UPDATE_IND for this single-PHY radio path.
 * Returns the new symmetric PHY (1 = 1M, 2 = 2M) when both directions
 * resolve to the same supported PHY and differ from cur_phy; otherwise 0.
 */
uint8_t s2_resolve_phy_update(uint8_t cur_phy, uint8_t central_to_pad, uint8_t pad_to_central) {
    uint8_t next_rx = central_to_pad ? central_to_pad : cur_phy;
    uint8_t next_tx = pad_to_central ? pad_to_central : cur_phy;
    if (next_rx != next_tx) return 0u;
    if (next_rx != 1u && next_rx != 2u) return 0u;
    if (next_rx == cur_phy) return 0u;
    return next_rx;
}

static inline uint16_t csa2_perm(uint16_t v) {
    v = (uint16_t)(((v & 0xAAAAu) >> 1) | ((v & 0x5555u) << 1));
    v = (uint16_t)(((v & 0xCCCCu) >> 2) | ((v & 0x3333u) << 2));
    v = (uint16_t)(((v & 0xF0F0u) >> 4) | ((v & 0x0F0Fu) << 4));
    return v;
}

static inline uint16_t csa2_mam(uint16_t a, uint16_t b) {
    return (uint16_t)((17u * (uint32_t)a + (uint32_t)b) & 0xFFFFu);
}

static inline uint8_t mod37_u16(uint16_t x) {
    uint32_t q = ((uint32_t)x * 28340u) >> 20;
    return (uint8_t)(x - q * 37u);
}

#ifdef HOST_TEST
extern const uint8_t *host_csa2_remap_table(void);
#define CSA2_REMAP_TABLE  (host_csa2_remap_table())
#else
#define CSA2_REMAP_TABLE  ((const volatile uint8_t *)0x00844b50u)
#endif

/*
 * Called from RAM 0x32b4 on every CSA #2 connection event:
 *   r0 = conn->chn_map (5-byte bitmap)
 *   r1 = conn->event_counter (uint16_t)
 *   r2 = conn->channel_identifier (uint16_t)
 */
#ifndef HOST_TEST
__attribute__((section(".cave.csa2")))
#endif
uint8_t csa2_select_channel(const uint8_t *chn_map, uint16_t counter, uint16_t chan_id) {
    uint16_t x = (uint16_t)(counter ^ chan_id);
    x = csa2_mam(csa2_perm(x), chan_id);
    x = csa2_mam(csa2_perm(x), chan_id);
    x = csa2_mam(csa2_perm(x), chan_id);
    uint16_t prn_e = (uint16_t)(x ^ chan_id);

    uint8_t unmapped = mod37_u16(prn_e);
    if (chn_map[unmapped >> 3] & (uint8_t)(1u << (unmapped & 7u))) {
        return unmapped;
    }

    uint8_t n = CSA2_REMAP_TABLE[37];
    if (n == 0u) {
        return 0u;
    }
    uint8_t idx = (uint8_t)(((uint32_t)n * (uint32_t)prn_e) >> 16);
    return CSA2_REMAP_TABLE[idx];
}

#ifndef HOST_TEST

/* Stock U2W v1.10 symbols resolved by link.ld */
extern void stock_ll_init_slave(void);
extern void stock_att_set_table(const void *tbl);
extern int  stock_smp_init(void);
extern int  stock_ll_set_adv_data(const uint8_t *p, uint8_t len);
extern void stock_ll_set_scan_rsp(const uint8_t *p, uint8_t len);
extern int  stock_disconn_check(void);
extern void stock_gatt_init(void);
extern int  stock_bond_count(void);
extern int  stock_gatt_exchange_mtu(uint16_t conn_h, uint16_t mtu);
extern int  stock_conn_param_update(uint16_t min, uint16_t max, uint16_t latency, uint16_t timeout);
extern void stock_batt_notify(void);
extern void stock_start_pairing(void);
extern int  stock_input_report(void);
extern void stock_ble_app_loop(void);
extern void stock_24g_init(void);

extern const uint8_t elite_adv_data[30];
extern const uint8_t elite_scan_rsp[28];

#define REG_CSA2_EN        (*(volatile uint8_t  *)0x00844969u)
#define REG_LL_FEAT_LO     (*(volatile uint32_t *)0x00844a04u)
#define BLTTCON            ((volatile uint8_t   *)0x00844af0u)
#define REG_CSA2_CB        (*(volatile uint32_t *)0x00844b78u)
#define BLT_CONN_PHY       ((volatile uint8_t   *)0x00844b7cu)
#define REG_PHY_HW         (*(volatile uint8_t  *)0x00844b86u)
#define REG_PHY_UPDATE_CB  (*(volatile uint32_t *)0x00844b88u)
#define REG_PHY_SWITCH_CB  (*(volatile uint32_t *)0x00844b8cu)
#define REG_ATT_RX_MTU     (*(volatile uint16_t *)0x00844d8cu)
#define REG_ATT_DISC_HOLD  (*(volatile uint8_t  *)0x00844d94u)
#define REG_SMP_FLASH_ADDR (*(volatile uint32_t *)0x00844f68u)
#define REG_PAIR_MODE      (*(volatile uint8_t  *)0x00845fa0u)
#define REG_24G_MODE       (*(volatile uint8_t  *)0x0084615du)

#define REG8(a)            (*(volatile uint8_t  *)(a))
#define REG16(a)           (*(volatile uint16_t *)(a))
#define REG32(a)           (*(volatile uint32_t *)(a))

#define LL_FEAT_CSA2_BIT   (1u << 14)
#define LL_FEAT_2M_PHY     (1u << 8)
#define LL_FEAT_CONN_PRM   (1u << 1)

/* Exact TLSR8258 BLE 1M / 2M PHY baseband & modem register switch (from Telink liblt_8258) */
static void rf_ble_switch_phy_1m_2m(uint8_t phy) {
    static const struct { uint16_t off; uint8_t v1m; uint8_t v2m; } kRegs[] = {
        { 0x1220u, 0x16u, 0x04u }, { 0x1221u, 0x0au, 0x2au },
        { 0x1222u, 0x20u, 0x43u }, { 0x1223u, 0x23u, 0x06u },
        { 0x124au, 0x0eu, 0x89u }, { 0x124bu, 0x09u, 0x06u },
        { 0x1254u, 0x89u, 0x0eu }, { 0x1255u, 0x06u, 0x09u },
        { 0x1256u, 0x8cu, 0x0cu }, { 0x1257u, 0x07u, 0x08u },
        { 0x1273u, 0x01u, 0x01u },
        { 0x1236u, 0xb7u, 0xb7u }, { 0x1237u, 0x8eu, 0x8eu },
        { 0x1238u, 0xc4u, 0xc4u }, { 0x1239u, 0x71u, 0x71u },
        { 0x0402u, 0x46u, 0x4cu }, { 0x0404u, 0xf5u, 0xe5u },
        { 0x0405u, 0x04u, 0x04u }, { 0x0420u, 0x1eu, 0x1eu },
    };
    uint8_t is2m = (phy == 2u);
    for (unsigned i = 0; i < sizeof(kRegs) / sizeof(kRegs[0]); i++) {
        REG8(0x00800000u + kRegs[i].off) = is2m ? kRegs[i].v2m : kRegs[i].v1m;
    }
    REG32(0x00800460u) = is2m ? 0x61514636u : 0x5f4f4434u;
    REG16(0x00800464u) = is2m ? 0x786du : 0x766bu;
}

void s2_phy_reset_1m(void) {
    BLT_CONN_PHY[1] = 1u; /* conn_cur_phy = 1M */
    BLT_CONN_PHY[2] = 1u; /* conn_next_phy = 1M */
    BLT_CONN_PHY[7] = 0u; /* phy_update_pending = 0 */
    if (REG_PHY_HW != 1u) {
        rf_ble_switch_phy_1m_2m(1u);
        REG_PHY_HW = 1u;
    }
}

static void s2_phy_schedule_update(uint8_t phy, uint16_t instant) {
    BLT_CONN_PHY[2] = phy;
    BLT_CONN_PHY[7] = 1u;
    /* Arm Telink's native connUpdate_mapUpdate_proc (0x169f0): blttcon[30..31]=instant, [1]=3 */
    BLTTCON[30] = (uint8_t)(instant & 0xffu);
    BLTTCON[31] = (uint8_t)((instant >> 8) & 0xffu);
    BLTTCON[1] = 3u;
}

/* Stock LL RX has already authenticated/decrypted the PDU and checked its length. */
void s2_handle_phy_update(const uint8_t *pdu) {
    uint8_t next = s2_resolve_phy_update(BLT_CONN_PHY[1], pdu[3], pdu[4]);
    if (next != 0u) {
        uint16_t instant = (uint16_t)pdu[5] | ((uint16_t)pdu[6] << 8);
        s2_phy_schedule_update(next, instant);
    }
}

/* Called by Telink's connUpdate_mapUpdate_proc (0x16aa4) at exact event_counter == instant */
int s2_phy_update_cb(void) {
    BLT_CONN_PHY[1] = BLT_CONN_PHY[2];
    BLT_CONN_PHY[7] = 0u;
    return 0;
}

/* Called from SRAM brx (0x3246) on every connection event right after rf_set_channel */
int s2_phy_switch_cb(void) {
    uint8_t target = BLT_CONN_PHY[1];
    if (target != REG_PHY_HW && (target == 1u || target == 2u)) {
        rf_ble_switch_phy_1m_2m(target);
        REG_PHY_HW = target;
    }
    return 0;
}

/* Hooked at 0xd58e (replaces `tjl 0x161bc`) */
extern void stock_matrix_set_all_columns(uint8_t high);
extern void stock_matrix_set_column(uint8_t column, uint8_t high);
extern uint8_t stock_matrix_read_rows(void);
extern void stock_delay_us(uint32_t us);

static uint8_t s2_sample_matrix(uint8_t col, uint8_t row_mask) {
    /* Row bit 0 (0x01) on cols 1..4 = D-pad Up, Left, Down, Right.
     * Row bit 1 (0x02) on cols 1..4 = Face X, Y, A, B. */
    stock_matrix_set_all_columns(1u);
    stock_matrix_set_column(col, 0u);
    stock_delay_us(6u);
    uint8_t pressed = (stock_matrix_read_rows() & row_mask) != 0u;
    stock_matrix_set_all_columns(0u);
    stock_delay_us(6u);
    return pressed;
}

/* 8 KiB ping-pong Telink SMP regions for universal D-pad slots 0..3 (Up/Down/Left/Right). */
static const uint32_t kSmpSlotAddr[4] = {
    0x00074000u,
    0x0006e000u,
    0x00070000u,
    0x00072000u,
};

static uint8_t s2_scan_cols(const uint8_t *cols, uint8_t count, uint8_t row_mask) {
    for (uint8_t i = 0; i < count; i++) {
        if (s2_sample_matrix(cols[i], row_mask)) return i;
    }
    return 0xffu;
}

/* Hooked at 0xd570 (replaces no-op `tjl 0xfce0(mac, 6)` before LL MAC init).
 * Resolves universal D-pad slot + BLE mode, persists selection, clears/bumps
 * pairing identity on mode change or Pair, and derives mac[6]. */
void s2_bt_mac_init_hook(uint8_t mac[6], uint8_t len) {
    (void)len;
    static const uint8_t kDpadCols[4] = { 1u, 3u, 2u, 4u }; /* Up, Down, Left, Right */
    static const uint8_t kFaceCols[3] = { 2u, 4u, 1u };     /* Y=Switch1(0), B=Stock(1), X=Elite(2) */
    uint8_t dpad = s2_scan_cols(kDpadCols, 4u, 0x01u);
    uint8_t face = s2_scan_cols(kFaceCols, 3u, 0x02u);

    s2_bt_boot_t prev, boot;
    s2_cfg_load_bt(&prev);
    boot = prev;
    uint8_t mode_changed = s2_resolve_bt_boot(&boot, dpad, face);
    if (mode_changed) {
        boot.s2_bond_mask &= (uint8_t)~(1u << boot.slot);
    }
    if (!s2_cfg_save_bt(&boot, (REG_PAIR_MODE == 1u) || mode_changed)) {
        boot = prev;
    } else if (mode_changed) {
        s2_erase_smp_slot(kSmpSlotAddr[boot.slot]);
    }
    s2_derive_slot_mac(mac, boot.slot, boot.mac_gen);
    S2_STOCK_BT = boot.mode == S2_SLOT_MODE_ELITE ? (uint8_t)(S2_BT_ELITE_LATCH + boot.slot)
                : (boot.mode == S2_SLOT_MODE_STOCK ? (uint8_t)(1u + boot.slot) : 0u);
}

__attribute__((section(".cave.init")))
void ble_init_hook(void) {
    stock_ll_init_slave();
    if (S2_STOCK_BT) return;

    uint8_t *p = (uint8_t *)S2_STATE;
    for (unsigned i = 0; i < S2_STATE_BYTES; i++) {
        p[i] = 0u;
    }
    S2_STATE->input_mode = 0x3fu;
    S2_STATE->vib_enabled = 1u;
    S2_STATE->proto_mode = 0x01u;
    S2_STATE->last_bat_pct = 0xffu;

    REG_CSA2_EN = 1u;
    REG_LL_FEAT_LO |= LL_FEAT_CSA2_BIT | LL_FEAT_2M_PHY | LL_FEAT_CONN_PRM;
    REG_CSA2_CB = (uint32_t)&csa2_select_channel;
    REG_PHY_HW = 1u;
    BLT_CONN_PHY[1] = 1u;
    BLT_CONN_PHY[2] = 1u;
    BLT_CONN_PHY[7] = 0u;
    REG_PHY_UPDATE_CB = (uint32_t)&s2_phy_update_cb;
    REG_PHY_SWITCH_CB = (uint32_t)&s2_phy_switch_cb;
}

/* Hooked at 0xd59a (replaces `tjl 0xd75c`) */
void s2_gatt_init_hook(void) {
    if (S2_STOCK_BT) {
        elite_gatt_init();
        stock_gatt_init();
        return;
    }
    stock_att_set_table(sw1_gatt_table);
    REG_ATT_RX_MTU = 158u;
    REG_ATT_DISC_HOLD = 0u;      /* Disable 300 ms post-discovery notify hold-off */
}

/* Hooked at 0xd5aa (replaces `tjl 0x19fc8`) */
int s2_smp_init_hook(void) {
    if (S2_STOCK_BT) {
        REG_SMP_FLASH_ADDR = kSmpSlotAddr[(S2_STOCK_BT - 1u) & 3u];
        return stock_smp_init();
    }
    s2_bt_boot_t boot;
    s2_cfg_load_bt(&boot);
    REG_SMP_FLASH_ADDR = kSmpSlotAddr[boot.slot & 3u];
    int r = stock_smp_init();
    /* Bound Telink's negotiated ATT MTU after processing Exchange MTU. */
    *(volatile uint32_t *)0x00844a30u = (uint32_t)&s2_l2cap_rx_tap;
    return r;
}

/* Hooked at 0xd5b2 (replaces `tjl 0x158b4`) */
void s2_adv_data_hook(const uint8_t *orig_p, uint8_t orig_len) {
    if (S2_STOCK_BT >= S2_BT_ELITE_LATCH) {
        stock_ll_set_adv_data(elite_adv_data, sizeof(elite_adv_data));
        return;
    }
    if (S2_STOCK_BT) {
        stock_ll_set_adv_data(orig_p, orig_len);
        return;
    }
    stock_ll_set_adv_data(sw1_adv_data, sizeof(sw1_adv_data));
}

/* Hooked at 0xd5ba (replaces `tjl 0x15938`) */
void s2_scan_rsp_hook(const uint8_t *orig_p, uint8_t orig_len) {
    if (S2_STOCK_BT >= S2_BT_ELITE_LATCH) {
        stock_ll_set_scan_rsp(elite_scan_rsp, (uint8_t)(elite_scan_rsp[0] + 1u));
        return;
    }
    if (S2_STOCK_BT) {
        stock_ll_set_scan_rsp(orig_p, orig_len);
        return;
    }
    stock_ll_set_scan_rsp(sw1_scan_rsp, sizeof(sw1_scan_rsp));
}

/* Hooked at 0xd5be (replaces `tjl 0x1a018`): standard SMP bond count */
int s2_bond_count_hook(void) {
    return stock_bond_count();
}

/* Hooked at 0xd48a (replaces `tjl 0x187f0` in connect cb) */
int s2_connect_mtu_hook(uint16_t conn_h, uint16_t mtu) {
    if (S2_STOCK_BT) return stock_gatt_exchange_mtu(conn_h, mtu);
    sw1_on_ble_connect();
    return stock_gatt_exchange_mtu(conn_h, 158u);
}

/* Hooked at 0xd4ea (replaces `tjl 0x10e94` in disconnect cb) */
int s2_disconnect_hook(void) {
    if (!S2_STOCK_BT) {
        sw1_on_ble_disconnect();
    }
    return stock_disconn_check();
}

/* Hooked at 0xd496 (connection parameter update). */
int s2_conn_param_hook(uint16_t min, uint16_t max, uint16_t latency, uint16_t timeout) {
    return stock_conn_param_update(min, max, latency, timeout);
}

void s2_batt_notify_hook(void) {
    if (S2_STOCK_BT) stock_batt_notify();
    else sw1_notify_battery();
}

int s2_input_report_hook(void) {
    return S2_STOCK_BT ? stock_input_report() : sw1_send_input_report();
}

void s2_start_pairing_hook(void) {
    stock_start_pairing();
}

/* Called after the Telink BLE stack has run for this iteration. */
void s2_ble_app_loop_hook(void) {
    stock_ble_app_loop();
    if (!S2_STOCK_BT && S2_STATE->q_count > 0u) {
        sw1_flush_pending_replies();
    }
}

/* Hooked at 0xd9ec (replaces `tjl 0xff94` for 2.4G / USB startup). */
void s2_24g_init_hook(void) {
    stock_24g_init();
    /* Home + B = DInput (1), Home + X = XInput (0), Home + Y = Switch (2), Home = last used. */
    static const uint8_t k24gCols[3] = { 4u, 1u, 2u }; /* B > X > Y precedence */
    uint8_t sel = s2_scan_cols(k24gCols, 3u, 0x02u);
    uint8_t cfg = s2_cfg_load();
    uint8_t mode = sel < 3u ? (uint8_t)(sel ^ (sel < 2u))
                            : (uint8_t)((cfg & S2_CFG_24G_MASK) >> S2_CFG_24G_SHIFT);
    if (mode > 2u) mode = 0u;
    uint8_t new_cfg = (uint8_t)((cfg & (uint8_t)~S2_CFG_24G_MASK)
                                | ((uint8_t)(mode << S2_CFG_24G_SHIFT) & S2_CFG_24G_MASK));
    if (new_cfg != cfg) {
        s2_cfg_save_flags(new_cfg);
    }
    REG_24G_MODE = mode;
}

#endif /* !HOST_TEST */
