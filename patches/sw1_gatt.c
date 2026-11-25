/*
 * Nintendo Switch 1 Pro Controller (057e:2009) BLE HOGP GATT table & advertising data
 * for Telink TLSR8258 (U2W v1.10).
 *
 * Compatible with macOS BTLEServer + JoyConHIDServicePlugin + AppleGameControllerPersonality,
 * Linux BlueZ / hid-nintendo, and SDL3 HIDAPI Switch driver.
 */

#include <stdint.h>
#include "s2_state.h"

#define CHAR_DECL_16(props, h_lo, h_hi, u_lo, u_hi) \
    { (props), (h_lo), (h_hi), (u_lo), (u_hi) }

/* Standard 16-bit GATT / HID UUIDs (LE) */
static const uint8_t u16_primary_svc[2]  = { 0x00, 0x28 };
static const uint8_t u16_char_decl[2]    = { 0x03, 0x28 };
static const uint8_t u16_cccd[2]         = { 0x02, 0x29 };
static const uint8_t u16_report_ref[2]   = { 0x08, 0x29 };

static const uint8_t u16_gap_svc[2]      = { 0x00, 0x18 };
static const uint8_t u16_dev_name[2]     = { 0x00, 0x2a };
static const uint8_t u16_appearance[2]   = { 0x01, 0x2a };

static const uint8_t u16_gatt_svc[2]     = { 0x01, 0x18 };
static const uint8_t u16_svc_chg[2]      = { 0x05, 0x2a };

static const uint8_t u16_dis_svc[2]      = { 0x0a, 0x18 };
static const uint8_t u16_mfr_name[2]     = { 0x29, 0x2a };
static const uint8_t u16_pnp_id[2]       = { 0x50, 0x2a };

static const uint8_t u16_bat_svc[2]      = { 0x0f, 0x18 };
static const uint8_t u16_bat_lvl[2]      = { 0x19, 0x2a };

static const uint8_t u16_hid_svc[2]      = { 0x12, 0x18 };
static const uint8_t u16_hid_info[2]     = { 0x4a, 0x2a };
static const uint8_t u16_report_map[2]   = { 0x4b, 0x2a };
static const uint8_t u16_hid_ctrl[2]     = { 0x4c, 0x2a };
static const uint8_t u16_report[2]       = { 0x4d, 0x2a };
static const uint8_t u16_proto_mode[2]   = { 0x4e, 0x2a };

/* Characteristic declaration values (5 bytes each) */
static const uint8_t cd_dev_name[5]   = CHAR_DECL_16(PROP_READ,                         0x03, 0x00, 0x00, 0x2a);
static const uint8_t cd_appear[5]     = CHAR_DECL_16(PROP_READ,                         0x05, 0x00, 0x01, 0x2a);
static const uint8_t cd_svc_chg[5]    = CHAR_DECL_16(PROP_INDICATE,                     0x08, 0x00, 0x05, 0x2a);
static const uint8_t cd_mfr_name[5]   = CHAR_DECL_16(PROP_READ,                         0x0c, 0x00, 0x29, 0x2a);
static const uint8_t cd_pnp_id[5]     = CHAR_DECL_16(PROP_READ,                         0x0e, 0x00, 0x50, 0x2a);
static const uint8_t cd_bat_lvl[5]    = CHAR_DECL_16(PROP_READ | PROP_NOTIFY,           0x11, 0x00, 0x19, 0x2a);
static const uint8_t cd_hid_info[5]   = CHAR_DECL_16(PROP_READ,                         0x15, 0x00, 0x4a, 0x2a);
static const uint8_t cd_report_map[5] = CHAR_DECL_16(PROP_READ,                         0x17, 0x00, 0x4b, 0x2a);
static const uint8_t cd_hid_ctrl[5]   = CHAR_DECL_16(PROP_WRITE_NR,                     0x19, 0x00, 0x4c, 0x2a);
static const uint8_t cd_proto_mode[5] = CHAR_DECL_16(PROP_READ | PROP_WRITE_NR,         0x1b, 0x00, 0x4e, 0x2a);
static const uint8_t cd_rep_in30[5]   = CHAR_DECL_16(PROP_READ | PROP_NOTIFY,           0x1d, 0x00, 0x4d, 0x2a);
static const uint8_t cd_rep_in21[5]   = CHAR_DECL_16(PROP_READ | PROP_NOTIFY,           0x21, 0x00, 0x4d, 0x2a);
static const uint8_t cd_rep_in3f[5]   = CHAR_DECL_16(PROP_READ | PROP_NOTIFY,           0x25, 0x00, 0x4d, 0x2a);
static const uint8_t cd_rep_out01[5]  = CHAR_DECL_16(PROP_READ | PROP_WRITE | PROP_WRITE_NR, 0x29, 0x00, 0x4d, 0x2a);
static const uint8_t cd_rep_out10[5]  = CHAR_DECL_16(PROP_READ | PROP_WRITE | PROP_WRITE_NR, 0x2c, 0x00, 0x4d, 0x2a);

/* Static attribute values */
static const uint8_t val_dev_name[14] = {
    'P','r','o',' ','C','o','n','t','r','o','l','l','e','r'
};
static const uint8_t val_appear[2]    = { 0xc4, 0x03 }; /* Gamepad (0x03C4) */
static const uint8_t val_mfr_name[8]  = { 'N','i','n','t','e','n','d','o' };
/* PnP ID: Vendor Source 0x02 (USB), VID 0x057E (Nintendo), PID 0x2009 (Pro Controller), Ver 0x0210 */
static const uint8_t val_pnp_id[7]    = { 0x02, 0x7e, 0x05, 0x09, 0x20, 0x10, 0x02 };
/* HID Info: bcdHID=0x0111, bCountryCode=0x00, Flags=0x02 (NormallyConnectable=0, RemoteWake=1) */
static const uint8_t val_hid_info[4]  = { 0x11, 0x01, 0x00, 0x02 };

/* Report Reference descriptors: { report_id, report_type (1=Input, 2=Output) } */
static const uint8_t ref_in30[2]  = { 0x30, 0x01 };
static const uint8_t ref_in21[2]  = { 0x21, 0x01 };
static const uint8_t ref_in3f[2]  = { 0x3f, 0x01 };
static const uint8_t ref_out01[2] = { 0x01, 0x02 };
static const uint8_t ref_out10[2] = { 0x10, 0x02 };

/*
 * Switch 1 Pro Controller BLE HOGP HID Report Map:
 *   - Input 0x3F (11 bytes body): standard Gamepad (16 buttons, hat + 4b pad, 4x16-bit axes)
 *   - Input 0x30 (48 bytes body): full Switch 1 state + 3 IMU frames
 *   - Input 0x21 (48 bytes body): subcommand reply
 *   - Output 0x01 (48 bytes body): rumble + subcommand
 *   - Output 0x10 (48 bytes body): rumble only
 */
static const uint8_t sw1_report_map[] = {
    0x05, 0x01,        /* Usage Page (Generic Desktop) */
    0x09, 0x05,        /* Usage (Game Pad) */
    0xA1, 0x01,        /* Collection (Application) */

    /* Input 0x3F: Simple HID */
    0x85, 0x3F,        /*   Report ID (0x3F) */
    0x05, 0x09,        /*   Usage Page (Button) */
    0x19, 0x01,        /*   Usage Minimum (1) */
    0x29, 0x10,        /*   Usage Maximum (16) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x01,        /*   Logical Maximum (1) */
    0x75, 0x01,        /*   Report Size (1) */
    0x95, 0x10,        /*   Report Count (16) */
    0x81, 0x02,        /*   Input (Data,Var,Abs) */
    0x05, 0x01,        /*   Usage Page (Generic Desktop) */
    0x09, 0x39,        /*   Usage (Hat switch) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0x07,        /*   Logical Maximum (7) */
    0x35, 0x00,        /*   Physical Minimum (0) */
    0x46, 0x3B, 0x01,  /*   Physical Maximum (315) */
    0x65, 0x14,        /*   Unit (Eng Rot: Degree) */
    0x75, 0x04,        /*   Report Size (4) */
    0x95, 0x01,        /*   Report Count (1) */
    0x81, 0x42,        /*   Input (Data,Var,Abs,Null State) */
    0x65, 0x00,        /*   Unit (None) */
    0x95, 0x01,        /*   Report Count (1) */
    0x81, 0x03,        /*   Input (Const,Var,Abs) */
    0x35, 0x00,        /*   Physical Minimum (0) */
    0x45, 0x00,        /*   Physical Maximum (0) */
    0x09, 0x30,        /*   Usage (X) */
    0x09, 0x31,        /*   Usage (Y) */
    0x09, 0x33,        /*   Usage (Rx) */
    0x09, 0x34,        /*   Usage (Ry) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x27, 0xFF, 0xFF, 0x00, 0x00, /* Logical Maximum (65535) */
    0x75, 0x10,        /*   Report Size (16) */
    0x95, 0x04,        /*   Report Count (4) */
    0x81, 0x02,        /*   Input (Data,Var,Abs) */

    /* Input 0x30: Standard full controller state + IMU */
    0x06, 0x00, 0xFF,  /*   Usage Page (Vendor Defined 0xFF00) */
    0x85, 0x30,        /*   Report ID (0x30) */
    0x09, 0x01,        /*   Usage (1) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00,  /*   Logical Maximum (255) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 0x30,        /*   Report Count (48) */
    0x81, 0x02,        /*   Input (Data,Var,Abs) */

    /* Input 0x21: Subcommand reply */
    0x85, 0x21,        /*   Report ID (0x21) */
    0x09, 0x01,        /*   Usage (1) */
    0x95, 0x30,        /*   Report Count (48) */
    0x81, 0x02,        /*   Input (Data,Var,Abs) */

    /* Output 0x01: Rumble + subcommand */
    0x85, 0x01,        /*   Report ID (0x01) */
    0x09, 0x01,        /*   Usage (1) */
    0x95, 0x30,        /*   Report Count (48) */
    0x91, 0x02,        /*   Output (Data,Var,Abs) */

    /* Output 0x10: Rumble only */
    0x85, 0x10,        /*   Report ID (0x10) */
    0x09, 0x01,        /*   Usage (1) */
    0x95, 0x30,        /*   Report Count (48) */
    0x91, 0x02,        /*   Output (Data,Var,Abs) */

    0xC0               /* End Collection */
};

/* Fixed RAM addresses inside S2_STATE (compile-time constant pointers for flash table) */
#ifdef HOST_TEST
#define RAM_PTR(field) ((uint8_t *)&(g_s2_test_state.field))
#else
#define RAM_PTR(field) ((uint8_t *)(0x00845f08u + __builtin_offsetof(s2_state_t, field)))
#endif

const telink_attr_t sw1_gatt_table[0x2e] = {
    /* 0x00: last valid handle index (0x2d = 45) */
    { 0x2d, 0, 0, 0, 0, 0, 0, 0 },

    /* 0x01..0x05: Generic Access (GAP 0x1800, 5 attributes) */
    { 5, ATT_PERM_READ, 2, 2,  u16_primary_svc, (uint8_t *)u16_gap_svc,  0, 0 },
    { 0, ATT_PERM_READ, 2, 5,  u16_char_decl,   (uint8_t *)cd_dev_name,  0, 0 },
    { 0, ATT_PERM_READ, 2, 14, u16_dev_name,    (uint8_t *)val_dev_name, 0, 0 },
    { 0, ATT_PERM_READ, 2, 5,  u16_char_decl,   (uint8_t *)cd_appear,    0, 0 },
    { 0, ATT_PERM_READ, 2, 2,  u16_appearance,  (uint8_t *)val_appear,   0, 0 },

    /* 0x06..0x09: Generic Attribute (GATT 0x1801, 4 attributes) */
    { 4, ATT_PERM_READ, 2, 2,  u16_primary_svc, (uint8_t *)u16_gatt_svc, 0, 0 },
    { 0, ATT_PERM_READ, 2, 5,  u16_char_decl,   (uint8_t *)cd_svc_chg,   0, 0 },
    { 0, ATT_PERM_READ, 2, 4,  u16_svc_chg,     RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,   2, 2,  u16_cccd,        RAM_PTR(cccd_svc),       0, 0 },

    /* 0x0a..0x0e: Device Information (DIS 0x180A, 5 attributes) */
    { 5, ATT_PERM_READ, 2, 2,  u16_primary_svc, (uint8_t *)u16_dis_svc,  0, 0 },
    { 0, ATT_PERM_READ, 2, 5,  u16_char_decl,   (uint8_t *)cd_mfr_name,  0, 0 },
    { 0, ATT_PERM_READ, 2, 8,  u16_mfr_name,    (uint8_t *)val_mfr_name, 0, 0 },
    { 0, ATT_PERM_READ, 2, 5,  u16_char_decl,   (uint8_t *)cd_pnp_id,    0, 0 },
    { 0, ATT_PERM_READ, 2, 7,  u16_pnp_id,      (uint8_t *)val_pnp_id,   0, 0 },

    /* 0x0f..0x12: Battery Service (0x180F, 4 attributes) */
    { 4, ATT_PERM_READ, 2, 2,  u16_primary_svc, (uint8_t *)u16_bat_svc,  0, 0 },
    { 0, ATT_PERM_READ, 2, 5,  u16_char_decl,   (uint8_t *)cd_bat_lvl,   0, 0 },
    { 0, ATT_PERM_READ, 2, 1,  u16_bat_lvl,     RAM_PTR(bat_level),      0, 0 },
    { 0, ATT_PERM_RW,   2, 2,  u16_cccd,        RAM_PTR(cccd_bat),       0, 0 },

    /* 0x13..0x2d: Human Interface Device (HID 0x1812, 27 attributes) */
    { 27, ATT_PERM_READ,  2, 2,                     u16_primary_svc, (uint8_t *)u16_hid_svc,    0, 0 },
    { 0,  ATT_PERM_READ,  2, 5,                     u16_char_decl,   (uint8_t *)cd_hid_info,    0, 0 },
    { 0,  ATT_PERM_READ,  2, sizeof(val_hid_info),  u16_hid_info,    (uint8_t *)val_hid_info,   0, 0 },
    { 0,  ATT_PERM_READ,  2, 5,                     u16_char_decl,   (uint8_t *)cd_report_map,  0, 0 },
    { 0,  ATT_PERM_READ,  2, sizeof(sw1_report_map),u16_report_map,  (uint8_t *)sw1_report_map, 0, 0 },
    { 0,  ATT_PERM_READ,  2, 5,                     u16_char_decl,   (uint8_t *)cd_hid_ctrl,    0, 0 },
    { 0,  ATT_PERM_WRITE, 2, 1,                     u16_hid_ctrl,    RAM_PTR(dummy_val),        0, 0 },
    { 0,  ATT_PERM_READ,  2, 5,                     u16_char_decl,   (uint8_t *)cd_proto_mode,  0, 0 },
    { 0,  ATT_PERM_RW,    2, 1,                     u16_proto_mode,  RAM_PTR(proto_mode),       0, 0 },

    /* 0x1c..0x1f: Input Report 0x30 (Standard full + IMU) */
    { 0, ATT_PERM_READ, 2, 5, u16_char_decl,  (uint8_t *)cd_rep_in30, 0, 0 },
    { 0, ATT_PERM_READ, 2, 4, u16_report,     RAM_PTR(dummy_val),     0, 0 },
    { 0, ATT_PERM_RW,   2, 2, u16_cccd,       RAM_PTR(cccd_in30),     0, 0 },
    { 0, ATT_PERM_READ, 2, 2, u16_report_ref, (uint8_t *)ref_in30,    0, 0 },

    /* 0x20..0x23: Input Report 0x21 (Subcommand reply) */
    { 0, ATT_PERM_READ, 2, 5, u16_char_decl,  (uint8_t *)cd_rep_in21, 0, 0 },
    { 0, ATT_PERM_READ, 2, 4, u16_report,     RAM_PTR(dummy_val),     0, 0 },
    { 0, ATT_PERM_RW,   2, 2, u16_cccd,       RAM_PTR(cccd_in21),     0, 0 },
    { 0, ATT_PERM_READ, 2, 2, u16_report_ref, (uint8_t *)ref_in21,    0, 0 },

    /* 0x24..0x27: Input Report 0x3F (Simple HID) */
    { 0, ATT_PERM_READ, 2, 5, u16_char_decl,  (uint8_t *)cd_rep_in3f, 0, 0 },
    { 0, ATT_PERM_READ, 2, 4, u16_report,     RAM_PTR(dummy_val),     0, 0 },
    { 0, ATT_PERM_RW,   2, 2, u16_cccd,       RAM_PTR(cccd_in3f),     0, 0 },
    { 0, ATT_PERM_READ, 2, 2, u16_report_ref, (uint8_t *)ref_in3f,    0, 0 },

    /* 0x28..0x2a: Output Report 0x01 (Rumble + subcommand) */
    { 0, ATT_PERM_READ, 2, 5, u16_char_decl,  (uint8_t *)cd_rep_out01, 0, 0 },
    { 0, ATT_PERM_RW,   2, 4, u16_report,     RAM_PTR(dummy_val),      sw1_out01_write_cb, 0 },
    { 0, ATT_PERM_READ, 2, 2, u16_report_ref, (uint8_t *)ref_out01,    0, 0 },

    /* 0x2b..0x2d: Output Report 0x10 (Rumble only) */
    { 0, ATT_PERM_READ, 2, 5, u16_char_decl,  (uint8_t *)cd_rep_out10, 0, 0 },
    { 0, ATT_PERM_RW,   2, 4, u16_report,     RAM_PTR(dummy_val),      sw1_out10_write_cb, 0 },
    { 0, ATT_PERM_READ, 2, 2, u16_report_ref, (uint8_t *)ref_out10,    0, 0 },
};

/* Standard BLE HOGP Gamepad advertising packet (29 bytes <= 31B ADV_IND) */
const uint8_t sw1_adv_data[29] = {
    0x02, 0x01, 0x06,                               /* LE General Discoverable | BR/EDR Not Supported */
    0x03, 0x19, 0xc4, 0x03,                         /* Appearance: Gamepad (0x03C4) */
    0x05, 0x03, 0x12, 0x18, 0x0f, 0x18,             /* Complete 16-bit UUIDs: HID (0x1812), Battery (0x180F) */
    0x0f, 0x09, 'P','r','o',' ','C','o','n','t','r','o','l','l','e','r',
};

const uint8_t sw1_scan_rsp[16] = {
    0x0f, 0x09, 'P','r','o',' ','C','o','n','t','r','o','l','l','e','r',
};
