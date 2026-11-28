/*
 * 8BitDo Ultimate 2 Bluetooth 2.4G Receiver (stock public v1.03)
 * pi32v2 hook trampolines placed at exact stock call/entry addresses via link.ld.
 */

.section .hook_pack, "ax"
    call dinput_pack_hook

.section .hook_send, "ax"
    call dinput_send_hook

.section .hook_hk_a1, "ax"
    call hotkey_hook

.section .hook_hk_a2, "ax"
    call hotkey_hook

.section .hook_usb_tx, "ax"
    goto fix_usb_g_intr_write
