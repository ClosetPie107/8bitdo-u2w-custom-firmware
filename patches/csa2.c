/*
 * U2W v1.10 Switch 2 BLE hooks & CSA #2 channel selection.
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
extern void stock_ll_reg_ltk_cb(void *cb);
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
#ifdef S2_BOND_DIAGNOSTICS
extern uint8_t stock_profile_led_slot(void);
extern uint8_t stock_is_non_ble(void);
#endif

extern const uint8_t s2_scan_rsp[16];

#define REG_CSA2_EN       (*(volatile uint8_t  *)0x00844969u)
#define REG_LL_FEAT_LO    (*(volatile uint32_t *)0x00844a04u)
#define BLTTCON           ((volatile uint8_t   *)0x00844af0u)
#define REG_CSA2_CB       (*(volatile uint32_t *)0x00844b78u)
#define BLT_CONN_PHY      ((volatile uint8_t   *)0x00844b7cu)
#define REG_PHY_HW        (*(volatile uint8_t  *)0x00844b86u)
#define REG_PHY_UPDATE_CB (*(volatile uint32_t *)0x00844b88u)
#define REG_PHY_SWITCH_CB (*(volatile uint32_t *)0x00844b8cu)
#define REG_ATT_RX_MTU    (*(volatile uint16_t *)0x00844d8cu)
#define REG_ATT_DISC_HOLD (*(volatile uint8_t  *)0x00844d94u)
#define REG_SMP_SEC_REQ0  (*(volatile uint8_t  *)0x00844f28u)
#define REG_SMP_SEC_REQ1  (*(volatile uint8_t  *)0x00844f29u)
#define REG_PAIR_MODE     (*(volatile uint8_t  *)0x00845fa0u)
#define REG_SYSTEM_TICK   (*(volatile uint32_t *)0x00800740u)
#define REG_LL_STATE     (*(volatile uint8_t  *)0x0084495fu)
#define REG_LL_ENC_STATE (*(volatile uint8_t  *)0x00844c3du)
#define REG_APP_CONNECTED (*(volatile uint32_t *)0x0084445cu)
#define REG_24G_MODE     (*(volatile uint8_t  *)0x0084615du)

#define REG8(a)           (*(volatile uint8_t  *)(a))
#define REG16(a)          (*(volatile uint16_t *)(a))
#define REG32(a)          (*(volatile uint32_t *)(a))

#define LL_FEAT_CSA2_BIT  (1u << 14)
#define LL_FEAT_2M_PHY    (1u << 8)
#define LL_FEAT_CONN_PRM  (1u << 1)

/* Exact TLSR8258 BLE 1M / 2M PHY baseband & modem register switch (from Telink liblt_8258) */
static void rf_ble_switch_phy_1m_2m(uint8_t phy) {
    if (phy == 2u) {
        REG8(0x00801220u) = 0x04u;
        REG8(0x00801221u) = 0x2au;
        REG8(0x00801222u) = 0x43u;
        REG8(0x00801223u) = 0x06u;
        REG8(0x0080124au) = 0x89u;
        REG8(0x0080124bu) = 0x06u;
        REG8(0x00801254u) = 0x0eu;
        REG8(0x00801255u) = 0x09u;
        REG8(0x00801256u) = 0x0cu;
        REG8(0x00801257u) = 0x08u;
        REG8(0x00801273u) = 0x01u;
        REG8(0x00801236u) = 0xb7u;
        REG8(0x00801237u) = 0x8eu;
        REG8(0x00801238u) = 0xc4u;
        REG8(0x00801239u) = 0x71u;
        REG8(0x00800402u) = 0x4cu;
        REG8(0x00800404u) = 0xe5u;
        REG8(0x00800405u) = 0x04u;
        REG8(0x00800420u) = 0x1eu;
        REG32(0x00800460u) = 0x61514636u;
        REG16(0x00800464u) = 0x786du;
    } else {
        REG8(0x00801220u) = 0x16u;
        REG8(0x00801221u) = 0x0au;
        REG8(0x00801222u) = 0x20u;
        REG8(0x00801223u) = 0x23u;
        REG8(0x0080124au) = 0x0eu;
        REG8(0x0080124bu) = 0x09u;
        REG8(0x00801254u) = 0x89u;
        REG8(0x00801255u) = 0x06u;
        REG8(0x00801256u) = 0x8cu;
        REG8(0x00801257u) = 0x07u;
        REG8(0x00801273u) = 0x01u;
        REG8(0x00801236u) = 0xb7u;
        REG8(0x00801237u) = 0x8eu;
        REG8(0x00801238u) = 0xc4u;
        REG8(0x00801239u) = 0x71u;
        REG8(0x00800402u) = 0x46u;
        REG8(0x00800404u) = 0xf5u;
        REG8(0x00800405u) = 0x04u;
        REG8(0x00800420u) = 0x1eu;
        REG32(0x00800460u) = 0x5f4f4434u;
        REG16(0x00800464u) = 0x766bu;
    }
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

static uint8_t s2_sample_face_button(uint8_t col) {
    /* Row bit 1 (0x02) on columns 1..4 maps to physical X, Y, A, B. */
    stock_matrix_set_all_columns(1u);
    stock_matrix_set_column(col, 0u);
    stock_delay_us(6u);
    uint8_t pressed = (stock_matrix_read_rows() & 0x02u) != 0u;
    stock_matrix_set_all_columns(0u);
    stock_delay_us(6u);
    return pressed;
}

__attribute__((section(".cave.init")))
void ble_init_hook(void) {
    /* Home + B = 8BitDo BLE (DInput), Home + Y = Switch 2 BLE, Home = last used. */
    uint8_t hold_b = s2_sample_face_button(4u);
    uint8_t hold_y = s2_sample_face_button(2u);
    uint8_t cfg = s2_cfg_load();
    uint8_t bt_stock = (cfg & S2_CFG_BT_STOCK) != 0u;
    if (hold_b) {
        bt_stock = 1u;
    } else if (hold_y) {
        bt_stock = 0u;
    }
    uint8_t new_cfg = (uint8_t)((cfg & (uint8_t)~S2_CFG_BT_STOCK) | (bt_stock ? S2_CFG_BT_STOCK : 0u));
    if (new_cfg != cfg) {
        s2_cfg_save_flags(new_cfg);
    }
    S2_STOCK_BT = bt_stock;
    stock_ll_init_slave();
    if (S2_STOCK_BT) return;

    uint8_t *p = (uint8_t *)S2_STATE;
    for (unsigned i = 0; i < sizeof(s2_state_t); i++) {
        p[i] = 0u;
    }
    s2_bond_load();
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
        stock_gatt_init();
        return;
    }
    stock_att_set_table(s2_gatt_table);
    REG_ATT_RX_MTU = 512u;       /* Switch 2 requires ATT_EXCHANGE_MTU_RSP == 512 (effective MTU clamped to 158 in l2cap_rx_tap) */
    REG_ATT_DISC_HOLD = 0u;      /* Disable 300 ms post-discovery notify hold-off */
}

/* Hooked at 0xd5aa (replaces `tjl 0x19fc8`) */
int s2_smp_init_hook(void) {
    int r = stock_smp_init();
    if (S2_STOCK_BT) return r;
    REG_SMP_SEC_REQ0 = 0u;       /* Suppress SMP Security Request (0x0b) on connect */
    REG_SMP_SEC_REQ1 = 0u;
    stock_ll_reg_ltk_cb((void *)&s2_ltk_req_cb);
    /* Bound Telink's negotiated ATT MTU after processing the request. */
    *(volatile uint32_t *)0x00844a30u = (uint32_t)&s2_l2cap_rx_tap;
    return r;
}

/* Hooked at 0xd5b2 (replaces `tjl 0x158b4`) */
void s2_adv_data_hook(const uint8_t *orig_p, uint8_t orig_len) {
    if (S2_STOCK_BT) {
        stock_ll_set_adv_data(orig_p, orig_len);
        return;
    }
    s2_startup_advertising((uint16_t)(REG_SYSTEM_TICK >> 16), REG_PAIR_MODE == 1u);
}

/* Hooked at 0xd5ba (replaces `tjl 0x15938`) */
void s2_scan_rsp_hook(const uint8_t *orig_p, uint8_t orig_len) {
    if (S2_STOCK_BT) {
        stock_ll_set_scan_rsp(orig_p, orig_len);
        return;
    }
    stock_ll_set_scan_rsp(s2_scan_rsp, 16u);
}

/* Hooked at 0xd5be (replaces `tjl 0x1a018`): return 0 -> always use ADV_IND */
int s2_bond_count_hook(void) {
    return S2_STOCK_BT ? stock_bond_count() : 0;
}

/* Hooked at 0xd48a (replaces `tjl 0x187f0` in connect cb) */
int s2_connect_mtu_hook(uint16_t conn_h, uint16_t mtu) {
    if (S2_STOCK_BT) return stock_gatt_exchange_mtu(conn_h, mtu);
    s2_on_ble_connect();
    /* Let Switch 2 initiate ATT_EXCHANGE_MTU_REQ */
    return 0;
}

/* Hooked at 0xd4ea (replaces `tjl 0x10e94` in disconnect cb) */
int s2_disconnect_hook(void) {
    if (!S2_STOCK_BT) {
        s2_on_ble_disconnect((uint16_t)(REG_SYSTEM_TICK >> 16), REG_PAIR_MODE == 1u);
    }
    return stock_disconn_check();
}

/* Hooked at 0xd496 (connection parameter update). */
int s2_conn_param_hook(uint16_t min, uint16_t max, uint16_t latency, uint16_t timeout) {
    return S2_STOCK_BT ? stock_conn_param_update(min, max, latency, timeout) : 0;
}

void s2_batt_notify_hook(void) {
    if (S2_STOCK_BT) stock_batt_notify();
}

int s2_input_report_hook(void) {
    return S2_STOCK_BT ? stock_input_report() : s2_send_input_report();
}

/* Stock pairing can start while already advertising, without redoing BLE init. */
void s2_start_pairing_hook(void) {
    if (!S2_STOCK_BT) s2_pairing_advertising((uint16_t)(REG_SYSTEM_TICK >> 16));
    stock_start_pairing();
}

/* Called after the Telink BLE stack has run for this iteration. */
void s2_ble_app_loop_hook(void) {
    stock_ble_app_loop();
    if (S2_STOCK_BT) return;
    uint8_t connected = (REG_LL_STATE & 2u) || REG_APP_CONNECTED;
#ifndef S2_BOND_DIAGNOSTICS
    if ((S2_STATE->flags & (S2_FLAG_WAKE_ACTIVE | S2_FLAG_ADV_PENDING | S2_FLAG_PAIRING_OPEN)) ||
        (S2_STATE->have_ltk & S2_BOND_READY))
#endif
    {
        /* The stock LL_START_ENC_RSP handler sets encryption state to 5. */
        s2_service_ble((uint16_t)(REG_SYSTEM_TICK >> 16), connected,
                       connected && REG_LL_ENC_STATE == 5u, REG_PAIR_MODE == 1u);
    }
#ifdef S2_BOND_DIAGNOSTICS
    if (connected) s2_save_bond_diagnostics();
#endif
}

/* Hooked at 0xd9ec (replaces `tjl 0xff94` for 2.4G / USB startup). */
void s2_24g_init_hook(void) {
    stock_24g_init();
    /* Home + B = DInput (1), Home + X = XInput (0), Home + Y = Switch (2), Home = last used. */
    uint8_t hold_b = s2_sample_face_button(4u);
    uint8_t hold_x = s2_sample_face_button(1u);
    uint8_t hold_y = s2_sample_face_button(2u);
    uint8_t cfg = s2_cfg_load();
    uint8_t mode = (uint8_t)((cfg & S2_CFG_24G_MASK) >> S2_CFG_24G_SHIFT);
    if (mode > 2u) mode = 0u;
    if (hold_b) {
        mode = 1u;
    } else if (hold_x) {
        mode = 0u;
    } else if (hold_y) {
        mode = 2u;
    }
    uint8_t new_cfg = (uint8_t)((cfg & (uint8_t)~S2_CFG_24G_MASK)
                                | ((uint8_t)(mode << S2_CFG_24G_SHIFT) & S2_CFG_24G_MASK));
    if (new_cfg != cfg) {
        s2_cfg_save_flags(new_cfg);
    }
    REG_24G_MODE = mode;
}

#ifdef S2_BOND_DIAGNOSTICS
/* This return value goes only to the peer's LED display field, not the profile logic. */
uint8_t s2_profile_led_slot_hook(void) {
    uint8_t slot = stock_profile_led_slot();
    if (S2_STOCK_BT || !REG_APP_CONNECTED || stock_is_non_ble()) return slot;

    uint8_t flags = S2_STATE->bond_diagnostics;
    if ((flags & 0x40u) && (REG_SYSTEM_TICK & (1u << 23))) return 6u;
    if (flags & 0x20u) return 2u;
    return (flags & 0x02u) ? 1u : 0u;
}
#endif

#endif /* !HOST_TEST */
