SECTIONS {
    .hook_usb_tx 0x1e02384 : { *(.hook_usb_tx) }
    .hook_pack   0x1e06ae6 : { *(.hook_pack) }
    .hook_send   0x1e06af0 : { *(.hook_send) }
    .hook_hk_a1  0x1e0a652 : { *(.hook_hk_a1) }
    .hook_hk_a2  0x1e0a794 : { *(.hook_hk_a2) }

    .cave_dinput 0x1e32000 : {
        *(.text.pack)
        . = ALIGN(4);
        *(.text.send)
        *(.text.hotkey)
    }

    .cave_tx_fix 0x1e32300 : {
        *usb_tx_fix.o(.text .text.* .rodata*)
    }

    /DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
