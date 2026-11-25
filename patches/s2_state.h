#ifndef S2_STATE_H
#define S2_STATE_H

#include <stddef.h>
#include <stdint.h>

/*
 * Runtime BLE state placed in stock U2W GATT RAM [0x845f08, 0x845f78).
 * The final byte (0x845f77) is reserved for the S2_STOCK_BT mode latch.
 * Stock GATT values occupy this area when stock or Elite Bluetooth is active.
 */
typedef struct {
    uint32_t addr;
    uint8_t  subcmd;
    uint8_t  rlen;
    uint8_t  arg0;
    uint8_t  used;
} sw1_pending_rsp_t;

typedef struct {
    uint8_t cccd_in30[2];     /* Input 0x30 (Standard full + IMU) CCCD */
    uint8_t cccd_in21[2];     /* Input 0x21 (Subcommand reply) CCCD */
    uint8_t cccd_in3f[2];     /* Input 0x3F (Simple HID) CCCD */
    uint8_t cccd_bat[2];      /* Battery Level (0x2A19) CCCD */
    uint8_t cccd_svc[2];      /* GATT Service Changed (0x2A05) CCCD */
    uint8_t bat_level;        /* Battery Level attribute value (0..100%) */
    uint8_t proto_mode;       /* HID Protocol Mode attribute value (0x01 = Report) */
    uint8_t dummy_val[4];     /* Generic attribute value buffer */
    uint8_t input_mode;       /* Current Switch 1 report mode (0x3F or 0x30) */
    uint8_t imu_enabled;      /* 1 = include live IMU samples in 0x30 reports */
    uint8_t vib_enabled;      /* 1 = HD rumble active */
    uint8_t player_lights;    /* Stored player LED mask from subcmd 0x30 */
    uint8_t report_timer;     /* Wrapping 0..255 counter for 0x30 / 0x21 reports */
    uint8_t vib_cnt;          /* Vibrator report nibble state */
    uint8_t flags;            /* SW1_FLAG_* runtime state bits */
    uint8_t last_bat_pct;     /* Last battery % notified over Battery Service */
    uint8_t q_head;           /* Pending 0x21 response ring head */
    uint8_t q_tail;           /* Pending 0x21 response ring tail */
    uint8_t q_count;          /* Pending 0x21 response ring count */
    uint8_t has_last_simple;  /* 1 = last_simple holds previous 0x3F payload */
    uint8_t simple_idle_div;  /* Periodic keepalive counter for 0x3F mode */
    uint8_t last_simple[11];  /* Previous 0x3F report body for change detection */
    sw1_pending_rsp_t pending_q[4]; /* Back-to-back subcommand reply FIFO */
} s2_state_t;

#define S2_STATE_BYTES sizeof(s2_state_t)
typedef char s2_state_fits_in_ram[S2_STATE_BYTES <= 111u ? 1 : -1];

#ifdef HOST_TEST
extern s2_state_t g_s2_test_state;
#define S2_STATE (&g_s2_test_state)
#else
#define S2_STATE ((s2_state_t *)0x00845f08u)
#endif

/* Switch 1 BLE HOGP ATT value handles (indices into sw1_gatt_table) */
#define SW1_H_BAT_LVL       0x0011u
#define SW1_H_IN30          0x001du
#define SW1_H_IN21          0x0021u
#define SW1_H_IN3F          0x0025u
#define SW1_H_OUT01         0x0029u
#define SW1_H_OUT10         0x002cu

#define SW1_BODY_LEN        48u
#define SW1_SIMPLE_BODY_LEN 11u

#define BLS_CONN_HANDLE     0x0040u

/* Telink attribute_t (24 bytes, matches stock U2W v1.10 table convention) */
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
#define PROP_INDICATE       0x20u

/* Latched at BLE startup.
 * 0 = Switch 1 BLE HOGP; 1..4 = Stock 8BitDo BLE slots 0..3; 5..8 = Elite BLE slots 0..3. */
#define S2_STOCK_BT (*(volatile uint8_t *)0x00845f77u)
#define S2_BT_ELITE_LATCH     5u   /* >= 5: Stock BLE stack with Elite identity */

#define S2_SLOT_MODE_SW1      0u
#define S2_SLOT_MODE_STOCK    1u
#define S2_SLOT_MODE_ELITE    2u
/* Default slot modes: Up=Stock(1), Down=Stock(1), Left=Elite(2), Right=Switch 1(0). */
#define S2_DEFAULT_SLOT_MODES      0x25u
/* Default last-used slot per mode: Switch1=Right(3), Stock=Up(0), Elite=Left(2). */
#define S2_DEFAULT_LAST_MODE_SLOTS 0x23u

static inline uint8_t s2_slot_mode_get(uint8_t slot_modes, uint8_t slot) {
    uint8_t m = (uint8_t)((slot_modes >> ((slot & 3u) * 2u)) & 3u);
    return m > S2_SLOT_MODE_ELITE ? S2_SLOT_MODE_STOCK : m;
}

static inline uint8_t s2_slot_mode_set(uint8_t slot_modes, uint8_t slot, uint8_t mode) {
    uint8_t shift = (uint8_t)((slot & 3u) * 2u);
    return (uint8_t)((slot_modes & (uint8_t)~(3u << shift)) | ((mode & 3u) << shift));
}

/* Runtime flags in s2_state_t.flags. */
#define SW1_FLAG_RUMBLE_ACTIVE 0x01u

/* Persisted mode flags in flash record byte 30 (S2BD format). */
#define S2_CFG_BT_STOCK        0x01u /* 0 = Switch 1 BLE, 1 = 8BitDo BLE */
#define S2_CFG_24G_SHIFT       1u
#define S2_CFG_24G_MASK        0x06u /* 0 = XInput, 1 = DInput, 2 = Switch */
#define S2_CFG_BT_SLOT_SHIFT   3u
#define S2_CFG_BT_SLOT_MASK    0x18u /* 0..3 = BLE slots Up/Down/Left/Right */
#define S2_CFG_BT_ELITE        0x20u
#define S2_CFG_MODE_MASK       0x3fu
#define S2_CFG_HAS_BOND        0x80u /* Legacy Switch 2 bond flag */

typedef struct {
    uint8_t cfg;
    uint8_t slot_modes;
    uint8_t s2_bond_mask;
    uint8_t last_mode_slots;
    uint8_t slot;
    uint8_t mode;
    uint8_t mac_gen;
} s2_bt_boot_t;

/* Shared functions across patch units */
extern const telink_attr_t sw1_gatt_table[];
extern const uint8_t sw1_adv_data[29];
extern const uint8_t sw1_scan_rsp[16];

int  sw1_out01_write_cb(void *p);
int  sw1_out10_write_cb(void *p);
int  sw1_send_input_report(void);
void sw1_notify_battery(void);
void sw1_flush_pending_replies(void);
void sw1_on_ble_connect(void);
void sw1_on_ble_disconnect(void);

uint8_t s2_cfg_load(void);
void s2_cfg_load_bt(s2_bt_boot_t *b);
void s2_cfg_save_flags(uint8_t flags);
int  s2_cfg_save_bt(s2_bt_boot_t *b, uint8_t bump_mac);
uint8_t s2_resolve_bt_boot(s2_bt_boot_t *b, uint8_t dpad, uint8_t face);
void s2_erase_smp_slot(uint32_t addr);
void s2_derive_slot_mac(uint8_t mac[6], uint8_t bt_slot, uint8_t mac_gen);
void elite_gatt_init(void);
int  s2_l2cap_rx_tap(uint16_t conn_h, uint8_t *raw);
void s2_phy_reset_1m(void);
uint8_t s2_resolve_phy_update(uint8_t cur_phy, uint8_t central_to_pad, uint8_t pad_to_central);
uint8_t csa2_select_channel(const uint8_t *chn_map, uint16_t counter, uint16_t chan_id);

#endif /* S2_STATE_H */
