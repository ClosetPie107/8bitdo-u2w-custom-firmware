#ifndef S2_STATE_H
#define S2_STATE_H

#include <stdint.h>

/*
 * Switch 2 BLE runtime state placed in stock U2W GATT RAM [0x845f08, 0x845f78)
 * The final byte is reserved for the mode latch. The stock GATT values
 * occupy this area when stock Bluetooth is selected.
 */
typedef struct {
    uint8_t valid;      /* 0=empty, 2=flash read [rlen, addr(4)], 3=const payload id */
    uint8_t cmd;
    uint8_t transport;
    uint8_t sub;
    uint8_t via_vib;
    uint8_t data[5];    /* flash: [rlen, addr0..3]; const: [const_id] */
} s2_pending_q_t;

typedef struct {
    /* Pairing scratch is only live during 0x15 handshake; IMU state only when streaming. */
    union {
        struct {
            uint8_t pending_ltk[16];
            uint8_t pending_host_addr[6];
        } pair;
        struct {
            int32_t q[4];          /* Q20 unit quaternion [w, x, y, z] */
            uint16_t last_tick16;  /* (REG_SYSTEM_TICK >> 10), 15625 Hz */
            uint16_t sample_time12;/* 12-bit ~960 Hz accumulated motion timestamp */
            uint16_t sample_frac;  /* Q16 fractional accumulator for 960 Hz */
            uint8_t valid;
        } imu;
    } scratch;
    uint16_t window_start_tick;    /* wake or pairing window start (tick >> 16) */
    uint8_t ltk[16];               /* active bonded LTK = A1 ^ B1 */
    uint8_t host_addr[6];          /* bonded console identity addr (LE) */
    uint8_t cccd_common[2];        /* handle 0x000b */
    uint8_t cccd_pro2[2];          /* handle 0x000f */
    uint8_t cccd_resp1[2];         /* handle 0x001b */
    uint8_t cccd_resp2[2];         /* handle 0x001f */
    uint8_t cccd_unk1[2];          /* handle 0x0023 */
    uint8_t cccd_unk2[2];          /* handle 0x0027 */
    uint8_t cccd_aud[2];           /* handle 0x002f */
    uint8_t cccd_svc[2];           /* handle 0x003b */
    uint8_t have_ltk;              /* S2_BOND_* flags */
    uint8_t flags;                 /* S2_FLAG_* runtime state bits */
    uint8_t pairing_stage;         /* 0->1(addr)->2(key)->3(confirm) */
    uint8_t feature_mask;          /* from 0x0c/02 */
    uint8_t enabled_features;      /* from 0x0c/02,04,05 */
    uint8_t report_seq;            /* 0..255 input report counter */
    uint8_t dummy_val[4];          /* dummy read/write buffer */
    /* 2-entry pending response FIFO (slot 0 supports inline <=17B, flash, or const; slot 1 supports flash or const). */
    uint8_t pending_rsp_valid;     /* 1 = inline, 2 = flash read, 3 = const id */
    uint8_t pending_cmd;
    uint8_t pending_transport;
    uint8_t pending_sub;
    uint8_t pending_len;
    uint8_t pending_data[17];      /* inline (max 17B), flash [rlen, addr(4)], or [const_id] */
    s2_pending_q_t pending_q;      /* second FIFO entry for back-to-back init commands */
#ifdef S2_BOND_DIAGNOSTICS
    uint8_t bond_diagnostics;
#endif
} s2_state_t;

typedef char s2_state_fits_in_ram[sizeof(s2_state_t) <= 111u ? 1 : -1];

#ifdef HOST_TEST
extern s2_state_t g_s2_test_state;
#define S2_STATE (&g_s2_test_state)
#else
#define S2_STATE ((s2_state_t *)0x00845f08u)
#endif

/* Exact Switch 2 Pro Controller GATT handles */
#define S2_H_COMMON_INPUT   0x000au
#define S2_H_PRO2_INPUT     0x000eu
#define S2_H_COMMAND        0x0014u
#define S2_H_VIB_COMMAND    0x0016u
#define S2_H_CMD_RESP1      0x001au
#define S2_H_CMD_RESP2      0x001eu

#define BLS_CONN_HANDLE     0x0040u

/* Telink attribute_t (24 bytes, matches stock U2W v1.10 table at 0x1b5c8) */
typedef struct {
    uint16_t attNum;
    uint8_t  perm;
    uint8_t  uuidLen;
    uint32_t attrLen;
    const uint8_t *uuid;
    uint8_t *pAttrValue;
    int (*w)(void *p);
    int (*r)(void *p);
} telink_attr_t;

/* Telink ATT permissions */
#define ATT_PERM_READ       0x01u
#define ATT_PERM_WRITE      0x02u
#define ATT_PERM_RW         0x03u

/* BLE GATT characteristic properties */
#define PROP_READ           0x02u
#define PROP_WRITE_NR       0x04u
#define PROP_WRITE          0x08u
#define PROP_NOTIFY         0x10u

/* Latched at BLE startup. hooks.s reads the same byte for CONNECT_IND validation. */
#define S2_STOCK_BT (*(volatile uint8_t *)0x00845f77u)

#define S2_BOND_VALID          0x01u
#define S2_BOND_DIRTY          0x02u
#define S2_BOND_READY          0x04u

/* Runtime flags in s2_state_t.flags. */
#define S2_FLAG_WAKE_ACTIVE    0x01u
#define S2_FLAG_ADV_PENDING    0x02u
#define S2_FLAG_BOND_ATTEMPTED 0x04u
#define S2_FLAG_RUMBLE_ACTIVE  0x08u
#define S2_FLAG_PAIRING_OPEN   0x10u
#define S2_FLAG_PENDING_VIB    0x20u

/* Persisted mode flags in flash record byte 30 (version 3). */
#define S2_CFG_BT_STOCK        0x01u /* 0 = Switch 2 BLE, 1 = 8BitDo BLE */
#define S2_CFG_24G_SHIFT       1u
#define S2_CFG_24G_MASK        0x06u /* 0 = XInput, 1 = DInput, 2 = Switch */
#define S2_CFG_MODE_MASK       0x07u
#define S2_CFG_HAS_BOND        0x80u /* 1 = ltk + host_addr in record are valid */

#define S2_ADV_FORCE_DISCOVERY  0xffu
#define S2_ADV_WAKE_FLAG        0x81u

/* Shared functions across patch units */
extern const telink_attr_t s2_gatt_table[];
void s2_build_adv_data(uint8_t out[31], uint8_t wake_flag);
int  s2_cmd_write_cb(void *p);
int  s2_vib_cmd_write_cb(void *p);
int  s2_vib_write_cb(void *p);
int  s2_ltk_req_cb(uint16_t conn_handle, uint8_t *rand, uint16_t ediv);
int  s2_send_input_report(void);
void s2_on_ble_connect(void);
void s2_on_ble_disconnect(uint16_t tick, uint8_t pairing);
void s2_startup_advertising(uint16_t tick, uint8_t pairing);
void s2_pairing_advertising(uint16_t tick);
void s2_service_ble(uint16_t tick, uint8_t connected, uint8_t encrypted, uint8_t pairing);
void s2_bond_load(void);
uint8_t s2_cfg_load(void);
void s2_cfg_save_flags(uint8_t flags);
#ifdef S2_BOND_DIAGNOSTICS
void s2_save_bond_diagnostics(void);
#endif
int  s2_l2cap_rx_tap(uint16_t conn_h, uint8_t *raw);
void s2_phy_reset_1m(void);
uint8_t s2_resolve_phy_update(uint8_t cur_phy, uint8_t central_to_pad, uint8_t pad_to_central);
uint8_t csa2_select_channel(const uint8_t *chn_map, uint16_t counter, uint16_t chan_id);

#endif /* S2_STATE_H */
