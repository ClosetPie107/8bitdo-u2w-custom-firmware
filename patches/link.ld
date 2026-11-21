/* Linker script for U2W v1.10 Switch 2 BLE TC32 patches */

SECTIONS
{
    /* In relocated code: replace the profile LED index lookup for UART command 4. */
    . = 0x000007d4;
    .hook_profile_leds : { *(.hook_profile_leds) }

    /* 12 KiB zero cave inside bootloader-covered Telink flash [0x5000, 0x1ca60) */
    . = 0x0000a040;
    .cave : {
        *(.cave.init)
        *(.cave.csa2)
        *(.cave.interval)
        *(.cave.phy_update)
        *(.text .text.*)
        . = ALIGN(4);
        *(.rodata .rodata.*)
    }

    /* Connect callback hooks (0xd448) */
    . = 0x0000d48a;
    .hook_conn_mtu : { *(.hook_conn_mtu) }
    . = 0x0000d496;
    .hook_conn_param : { *(.hook_conn_param) }

    /* Disconnect callback hook (0xd4e0) */
    . = 0x0000d4ea;
    .hook_disconn : { *(.hook_disconn) }

    /* Cold BLE startup hooks (0xd558) */
    . = 0x0000d58e;
    .hook_ble_init : { *(.hook_ble_init) }
    . = 0x0000d59a;
    .hook_gatt_init : { *(.hook_gatt_init) }
    . = 0x0000d5aa;
    .hook_smp_init : { *(.hook_smp_init) }
    . = 0x0000d5b2;
    .hook_adv_data : { *(.hook_adv_data) }
    . = 0x0000d5ba;
    .hook_scan_rsp : { *(.hook_scan_rsp) }
    . = 0x0000d5be;
    .hook_bond_count : { *(.hook_bond_count) }

    /* BLE main loop hooks (0xd6a0) */
    . = 0x0000d6d4;
    .hook_batt_notify : { *(.hook_batt_notify) }
    . = 0x0000d6da;
    .hook_input_report : { *(.hook_input_report) }
    . = 0x0000d74c;
    .hook_ble_app_loop : { *(.hook_ble_app_loop) }

    /* Non-BLE (2.4G / USB) startup hook (replaces tjl 0xff94 at 0xd9ec). */
    . = 0x0000d9ec;
    .hook_24g_init : { *(.hook_24g_init) }

    /* Pair button advertising hook (stock pairing entry at 0xe168). */
    . = 0x0000e174;
    .hook_start_pairing : { *(.hook_start_pairing) }

    /* CONNECT_IND interval guard (replaces two instructions at 0x15b5c). */
    . = 0x00015b5c;
    .hook_conn_interval : { *(.hook_conn_interval) }

    /* Validated LL PHY control dispatch, after optional RX decryption. */
    . = 0x00016d0c;
    .hook_phy_update : { *(.hook_phy_update) }

    /DISCARD/ : {
        *(.comment)
        *(.ARM.*)
        *(.note.*)
    }
}
