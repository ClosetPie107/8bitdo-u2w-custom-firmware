/*
 * Nintendo Switch 2 Pro Controller GATT attribute table & advertising data
 * for Telink TLSR8258 (U2W v1.10).
 *
 * Exact handle layout (0x0001..0x0038) matching an updated Pro Controller 2
 * (FW 2.0.0+ with headset audio characteristics, GAP at 0x0033..0x0037,
 * GATT at 0x0038).
 */

#include <stdint.h>
#include "s2_state.h"

/* Helper macro to declare a 128-bit UUID in BLE little-endian byte order */
#define UUID128_BYTES(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) \
    { 0x##p, 0x##o, 0x##n, 0x##m, 0x##l, 0x##k, 0x##j, 0x##i, \
      0x##h, 0x##g, 0x##f, 0x##e, 0x##d, 0x##c, 0x##b, 0x##a }

/* Characteristic declaration value for a 128-bit UUID (19 bytes) */
#define CHAR_DECL_128(props, h_lo, h_hi, a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) \
    { (props), (h_lo), (h_hi), \
      0x##p, 0x##o, 0x##n, 0x##m, 0x##l, 0x##k, 0x##j, 0x##i, \
      0x##h, 0x##g, 0x##f, 0x##e, 0x##d, 0x##c, 0x##b, 0x##a }

#define CHAR_DECL_16(props, h_lo, h_hi, u_lo, u_hi) \
    { (props), (h_lo), (h_hi), (u_lo), (u_hi) }

/* Standard 16-bit UUIDs */
static const uint8_t u16_primary_svc[2] = { 0x00, 0x28 };
static const uint8_t u16_char_decl[2]   = { 0x03, 0x28 };
static const uint8_t u16_cccd[2]        = { 0x02, 0x29 };
static const uint8_t u16_gap_svc[2]     = { 0x00, 0x18 };
static const uint8_t u16_gatt_svc[2]    = { 0x01, 0x18 };
static const uint8_t u16_svc_chg[2]     = { 0x05, 0x2a };
static const uint8_t u16_dev_name[2]    = { 0x00, 0x2a };
static const uint8_t u16_appearance[2]  = { 0x01, 0x2a };

/* Service 1 UUIDs: 00c5af5d-1964-4e30-8f51-1956f96bd28x */
static const uint8_t u128_svc1[16]     = UUID128_BYTES(00,c5,af,5d, 19,64, 4e,30, 8f,51, 19,56,f9,6b,d2,80);
static const uint8_t u128_svc1_281[16] = UUID128_BYTES(00,c5,af,5d, 19,64, 4e,30, 8f,51, 19,56,f9,6b,d2,81);
static const uint8_t u128_svc1_282[16] = UUID128_BYTES(00,c5,af,5d, 19,64, 4e,30, 8f,51, 19,56,f9,6b,d2,82);
static const uint8_t u128_svc1_283[16] = UUID128_BYTES(00,c5,af,5d, 19,64, 4e,30, 8f,51, 19,56,f9,6b,d2,83);

/* Service 2 UUIDs (main Pro Controller 2 service) */
static const uint8_t u128_svc2[16]       = UUID128_BYTES(ab,7d,e9,be, 89,fe, 49,ad, 82,8f, 11,8f,09,df,7f,d0);
static const uint8_t u128_common_in[16]  = UUID128_BYTES(ab,7d,e9,be, 89,fe, 49,ad, 82,8f, 11,8f,09,df,7f,d2);
static const uint8_t u128_pro2_in[16]    = UUID128_BYTES(74,92,86,6c, ec,3e, 46,19, 82,58, 32,75,5f,fc,c0,f8);
static const uint8_t u128_vib[16]        = UUID128_BYTES(cc,48,3f,51, 92,58, 42,7d, a9,39, 63,0c,31,f7,2b,05);
static const uint8_t u128_cmd[16]        = UUID128_BYTES(64,9d,4a,c9, 8e,b7, 4e,6c, af,44, 1e,a5,4f,e5,f0,05);
static const uint8_t u128_vib_cmd[16]    = UUID128_BYTES(3d,ac,bc,7e, 69,55, 40,b5, 8e,af, 6f,98,09,e8,b3,79);
static const uint8_t u128_fw_upd[16]     = UUID128_BYTES(41,47,42,3d, fd,ae, 4d,f7, a4,f7, d2,3e,5d,f5,9f,8d);
static const uint8_t u128_resp1[16]      = UUID128_BYTES(c7,65,a9,61, d9,d8, 4d,36, a2,0a, 53,15,b1,11,83,6a);
static const uint8_t u128_resp2[16]      = UUID128_BYTES(50,6d,9f,7d, 42,78, 4e,95, a5,49, 32,6b,a7,76,57,e0);
static const uint8_t u128_unk_in1[16]    = UUID128_BYTES(d3,bd,69,d2, 84,1c, 42,41, ab,15, f8,6f,40,6d,2a,80);
static const uint8_t u128_unk_in2[16]    = UUID128_BYTES(ab,7d,e9,be, 89,fe, 49,ad, 82,8f, 11,8f,09,df,7f,de);
static const uint8_t u128_unk_out[16]    = UUID128_BYTES(ab,7d,e9,be, 89,fe, 49,ad, 82,8f, 11,8f,09,df,7f,df);
static const uint8_t u128_aud_out[16]    = UUID128_BYTES(cc,48,3f,51, 92,58, 42,7d, a9,39, 63,0c,31,f7,2b,06);
static const uint8_t u128_aud_in[16]     = UUID128_BYTES(74,92,86,6c, ec,3e, 46,19, 82,58, 32,75,5f,fc,c0,f9);
static const uint8_t u128_aud_cmd[16]    = UUID128_BYTES(3d,ac,bc,7e, 69,55, 40,b5, 8e,af, 6f,98,09,e8,b3,80);

/* Vendor descriptor UUIDs */
static const uint8_t u128_desc_rate[16]  = UUID128_BYTES(67,9d,55,10, 5a,24, 4d,ee, 95,57, 95,df,80,48,6e,cb);
static const uint8_t u128_desc_resp[16]  = UUID128_BYTES(b7,46,df,8c, f3,58, 49,5b, 9c,d2, e3,bb,ed,a4,f9,79);

/* Characteristic declaration values (19 bytes for 128-bit, 5 bytes for 16-bit) */
static const uint8_t cd_svc1_281[19]  = CHAR_DECL_128(PROP_READ,                0x03, 0x00, 00,c5,af,5d, 19,64, 4e,30, 8f,51, 19,56,f9,6b,d2,81);
static const uint8_t cd_svc1_282[19]  = CHAR_DECL_128(PROP_WRITE,               0x05, 0x00, 00,c5,af,5d, 19,64, 4e,30, 8f,51, 19,56,f9,6b,d2,82);
static const uint8_t cd_svc1_283[19]  = CHAR_DECL_128(PROP_READ,                0x07, 0x00, 00,c5,af,5d, 19,64, 4e,30, 8f,51, 19,56,f9,6b,d2,83);
static const uint8_t cd_common_in[19] = CHAR_DECL_128(PROP_READ | PROP_NOTIFY,  0x0a, 0x00, ab,7d,e9,be, 89,fe, 49,ad, 82,8f, 11,8f,09,df,7f,d2);
static const uint8_t cd_pro2_in[19]   = CHAR_DECL_128(PROP_READ | PROP_NOTIFY,  0x0e, 0x00, 74,92,86,6c, ec,3e, 46,19, 82,58, 32,75,5f,fc,c0,f8);
static const uint8_t cd_vib[19]       = CHAR_DECL_128(PROP_WRITE_NR,            0x12, 0x00, cc,48,3f,51, 92,58, 42,7d, a9,39, 63,0c,31,f7,2b,05);
static const uint8_t cd_cmd[19]       = CHAR_DECL_128(PROP_WRITE_NR,            0x14, 0x00, 64,9d,4a,c9, 8e,b7, 4e,6c, af,44, 1e,a5,4f,e5,f0,05);
static const uint8_t cd_vib_cmd[19]   = CHAR_DECL_128(PROP_WRITE_NR,            0x16, 0x00, 3d,ac,bc,7e, 69,55, 40,b5, 8e,af, 6f,98,09,e8,b3,79);
static const uint8_t cd_fw_upd[19]    = CHAR_DECL_128(PROP_WRITE_NR,            0x18, 0x00, 41,47,42,3d, fd,ae, 4d,f7, a4,f7, d2,3e,5d,f5,9f,8d);
static const uint8_t cd_resp1[19]     = CHAR_DECL_128(PROP_NOTIFY,              0x1a, 0x00, c7,65,a9,61, d9,d8, 4d,36, a2,0a, 53,15,b1,11,83,6a);
static const uint8_t cd_resp2[19]     = CHAR_DECL_128(PROP_NOTIFY,              0x1e, 0x00, 50,6d,9f,7d, 42,78, 4e,95, a5,49, 32,6b,a7,76,57,e0);
static const uint8_t cd_unk_in1[19]   = CHAR_DECL_128(PROP_NOTIFY,              0x22, 0x00, d3,bd,69,d2, 84,1c, 42,41, ab,15, f8,6f,40,6d,2a,80);
static const uint8_t cd_unk_in2[19]   = CHAR_DECL_128(PROP_READ | PROP_NOTIFY,  0x26, 0x00, ab,7d,e9,be, 89,fe, 49,ad, 82,8f, 11,8f,09,df,7f,de);
static const uint8_t cd_unk_out[19]   = CHAR_DECL_128(PROP_WRITE_NR,            0x2a, 0x00, ab,7d,e9,be, 89,fe, 49,ad, 82,8f, 11,8f,09,df,7f,df);
static const uint8_t cd_aud_out[19]   = CHAR_DECL_128(PROP_WRITE_NR,            0x2c, 0x00, cc,48,3f,51, 92,58, 42,7d, a9,39, 63,0c,31,f7,2b,06);
static const uint8_t cd_aud_in[19]    = CHAR_DECL_128(PROP_READ | PROP_NOTIFY,  0x2e, 0x00, 74,92,86,6c, ec,3e, 46,19, 82,58, 32,75,5f,fc,c0,f9);
static const uint8_t cd_aud_cmd[19]   = CHAR_DECL_128(PROP_WRITE_NR,            0x32, 0x00, 3d,ac,bc,7e, 69,55, 40,b5, 8e,af, 6f,98,09,e8,b3,80);
static const uint8_t cd_dev_name[5]   = CHAR_DECL_16(PROP_READ,                 0x35, 0x00, 0x00, 0x2a);
static const uint8_t cd_appear[5]     = CHAR_DECL_16(PROP_READ,                 0x37, 0x00, 0x01, 0x2a);
static const uint8_t cd_svc_chg[5]    = CHAR_DECL_16(0x20u,                     0x3a, 0x00, 0x05, 0x2a);

static const uint8_t val_dev_name[14] = {
    'P','r','o',' ','C','o','n','t','r','o','l','l','e','r'
};
static const uint8_t val_appear[2]    = { 0xc4, 0x03 }; /* Gamepad */

/* Fixed RAM addresses inside S2_STATE (compile-time constant pointers for flash table) */
#ifdef HOST_TEST
#define RAM_PTR(field) (g_s2_test_state.field)
#else
#define RAM_PTR(field) ((uint8_t *)(0x00845f08u + __builtin_offsetof(s2_state_t, field)))
#endif

const telink_attr_t s2_gatt_table[0x3c] = {
    /* 0x00: last valid handle index (0x3b = 59, matching stock Telink convention) */
    { 0x3b, 0, 0, 0, 0, 0, 0, 0 },

    /* 0x01..0x07: Service 1 */
    { 7, ATT_PERM_READ,  2, 16, u16_primary_svc, (uint8_t *)u128_svc1,    0, 0 },
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_svc1_281,  0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_svc1_281,   RAM_PTR(dummy_val),            0, 0 },
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_svc1_282,  0, 0 },
    { 0, ATT_PERM_WRITE,16,  4, u128_svc1_282,   RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_svc1_283,  0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_svc1_283,   RAM_PTR(dummy_val),     0, 0 },

    /* 0x08..0x32: Service 2 (43 attributes) */
    { 43, ATT_PERM_READ, 2, 16, u16_primary_svc, (uint8_t *)u128_svc2,    0, 0 },
    /* 0x09..0x0c: Common input (0x05) */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_common_in, 0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_common_in,  RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,    2,  2, u16_cccd,        RAM_PTR(cccd_common),    0, 0 },
    { 0, ATT_PERM_RW,   16,  4, u128_desc_rate,  RAM_PTR(dummy_val),      0, 0 },
    /* 0x0d..0x10: Pro Controller 2 input (0x09) — value at 0x000e, CCCD at 0x000f */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_pro2_in,   0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_pro2_in,    RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,    2,  2, u16_cccd,        RAM_PTR(cccd_pro2),      0, 0 },
    { 0, ATT_PERM_RW,   16,  4, u128_desc_rate,  RAM_PTR(dummy_val),      0, 0 },
    /* 0x11..0x12: Vibration */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_vib,       0, 0 },
    { 0, ATT_PERM_WRITE,16,  4, u128_vib,        RAM_PTR(dummy_val),      s2_vib_write_cb, 0 },
    /* 0x13..0x14: Command (basic) */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_cmd,       0, 0 },
    { 0, ATT_PERM_WRITE,16,  4, u128_cmd,        RAM_PTR(dummy_val),      s2_cmd_write_cb, 0 },
    /* 0x15..0x16: Vibration + Command (pairing runs here) */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_vib_cmd,   0, 0 },
    { 0, ATT_PERM_WRITE,16,  4, u128_vib_cmd,    RAM_PTR(dummy_val),      s2_vib_cmd_write_cb, 0 },
    /* 0x17..0x18: Firmware Update */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_fw_upd,    0, 0 },
    { 0, ATT_PERM_WRITE,16,  4, u128_fw_upd,     RAM_PTR(dummy_val),      0, 0 },
    /* 0x19..0x1c: Command Response #1 (value at 0x001a) */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_resp1,     0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_resp1,      RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,    2,  2, u16_cccd,        RAM_PTR(cccd_resp1),     0, 0 },
    { 0, ATT_PERM_RW,   16,  4, u128_desc_resp,  RAM_PTR(dummy_val),      0, 0 },
    /* 0x1d..0x20: Command Response #2 (value at 0x001e) */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_resp2,     0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_resp2,      RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,    2,  2, u16_cccd,        RAM_PTR(cccd_resp2),     0, 0 },
    { 0, ATT_PERM_RW,   16,  4, u128_desc_resp,  RAM_PTR(dummy_val),      0, 0 },
    /* 0x21..0x24: Unknown Input 1 */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_unk_in1,   0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_unk_in1,    RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,    2,  2, u16_cccd,        RAM_PTR(cccd_unk1),      0, 0 },
    { 0, ATT_PERM_RW,   16,  4, u128_desc_resp,  RAM_PTR(dummy_val),      0, 0 },
    /* 0x25..0x28: Unknown Input 2 */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_unk_in2,   0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_unk_in2,    RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,    2,  2, u16_cccd,        RAM_PTR(cccd_unk2),      0, 0 },
    { 0, ATT_PERM_RW,   16,  4, u128_desc_rate,  RAM_PTR(dummy_val),      0, 0 },
    /* 0x29..0x2a: Unknown Output */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_unk_out,   0, 0 },
    { 0, ATT_PERM_WRITE,16,  4, u128_unk_out,    RAM_PTR(dummy_val),      0, 0 },
    /* 0x2b..0x2c: Audio Output */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_aud_out,   0, 0 },
    { 0, ATT_PERM_WRITE,16,  4, u128_aud_out,    RAM_PTR(dummy_val),      0, 0 },
    /* 0x2d..0x30: Audio Input */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_aud_in,    0, 0 },
    { 0, ATT_PERM_READ, 16,  4, u128_aud_in,     RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,    2,  2, u16_cccd,        RAM_PTR(cccd_aud),       0, 0 },
    { 0, ATT_PERM_RW,   16,  4, u128_desc_rate,  RAM_PTR(dummy_val),      0, 0 },
    /* 0x31..0x32: Audio Command */
    { 0, ATT_PERM_READ,  2, 19, u16_char_decl,   (uint8_t *)cd_aud_cmd,   0, 0 },
    { 0, ATT_PERM_WRITE,16,  4, u128_aud_cmd,    RAM_PTR(dummy_val),      0, 0 },

    /* 0x33..0x37: Generic Access (GAP 0x1800, 5 attributes) */
    { 5, ATT_PERM_READ,  2,  2, u16_primary_svc, (uint8_t *)u16_gap_svc,  0, 0 },
    { 0, ATT_PERM_READ,  2,  5, u16_char_decl,   (uint8_t *)cd_dev_name,  0, 0 },
    { 0, ATT_PERM_READ,  2, 14, u16_dev_name,    (uint8_t *)val_dev_name, 0, 0 },
    { 0, ATT_PERM_READ,  2,  5, u16_char_decl,   (uint8_t *)cd_appear,    0, 0 },
    { 0, ATT_PERM_READ,  2,  2, u16_appearance,  (uint8_t *)val_appear,   0, 0 },

    /* 0x38..0x3b: Generic Attribute (GATT 0x1801, 4 attributes so Telink 0x179bc returns valid endHandle 0x003b) */
    { 4, ATT_PERM_READ,  2,  2, u16_primary_svc, (uint8_t *)u16_gatt_svc, 0, 0 },
    { 0, ATT_PERM_READ,  2,  5, u16_char_decl,   (uint8_t *)cd_svc_chg,   0, 0 },
    { 0, ATT_PERM_READ,  2,  4, u16_svc_chg,     RAM_PTR(dummy_val),      0, 0 },
    { 0, ATT_PERM_RW,    2,  2, u16_cccd,        RAM_PTR(cccd_svc),       0, 0 },
};

/*
 * 31-byte Switch 2 Pro Controller ADV_IND template:
 *   [0..2]  02 01 06 (Flags: LE General Discoverable | BR/EDR Not Supported)
 *   [3..4]  1b ff    (AD Len 27, AD Type 0xFF Manufacturer Specific Data)
 *   [5..30] 26-byte Nintendo manufacturer payload:
 *           53 05 (Nintendo) | 01 00 03 | 7e 05 (VID) | 69 20 (PID Pro2) |
 *           00 01 | wake(0x00/0x81) | host_addr[6] | 0f 00 00 00 00 00 00 00
 */
static const uint8_t s2_adv_template[31] = {
    0x02, 0x01, 0x06,
    0x1b, 0xff,
    0x53, 0x05, 0x01, 0x00, 0x03, 0x7e, 0x05, 0x69, 0x20, 0x00, 0x01,
    0x00, /* +16: wake flag (0x00 discovery/reconnect, 0x81 wake) */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* +17..22: bonded host BD_ADDR (LE) */
    0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const uint8_t s2_scan_rsp[16] = {
    0x0f, 0x09, 'P','r','o',' ','C','o','n','t','r','o','l','l','e','r',
};

void s2_build_adv_data(uint8_t out[31], uint8_t wake_flag) {
    s2_state_t *s = S2_STATE;
    for (int i = 0; i < 31; i++) {
        out[i] = s2_adv_template[i];
    }
    if ((s->have_ltk & S2_BOND_VALID) && wake_flag != S2_ADV_FORCE_DISCOVERY) {
        out[16] = wake_flag;
        for (int i = 0; i < 6; i++) {
            out[17 + i] = s->host_addr[i];
        }
    }
}
