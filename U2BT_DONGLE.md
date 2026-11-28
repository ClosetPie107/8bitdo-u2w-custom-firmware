# 8BitDo Ultimate 2 Bluetooth — 2.4G Receiver Custom Firmware

Unofficial open-source JieLi BR22 (`pi32v2`) firmware patch for the **8BitDo Ultimate 2 Bluetooth 2.4G USB Receiver** (firmware type `98`, logical PID `0x6011`, stock public **v1.03**), adding **34-byte D-Input 6-axis gyro + battery + extra buttons (`L4`/`R4`/`PL`/`PR`)**, **runtime mode switching**, **Steam Input Ultimate 2 (`2dc8:6012`) support**, and a **fix for a stock USB interrupt-IN deadlock** during mode switches.

> [!WARNING]
> **Flashing custom firmware carries risk.**
> - This is an **unofficial**, community-made modification with **no warranty** of any kind.
> - **Supported hardware ONLY:** **8BitDo Ultimate 2 Bluetooth 2.4G USB Receiver** (logical PID `0x6011`, type 98, JieLi BR22).
> - **Do NOT flash** this onto the *Ultimate 2 Bluetooth controller*, the *Ultimate 2 Wireless controller*, or the *Ultimate 2 Wireless 2.4G receiver* (`0x6012`). The flasher checks `CMD_IDENTIFY == 0x6011` and aborts on any other hardware.
> - Unlike the controller, the 2.4G receiver has **no hardware recovery button**. The flasher never touches `uboot.boot` or `isd_config.ini` (it only writes the application region starting at `0x4000` and verifies the device's first-4K CRC before rebooting), and stock v1.03 can be restored with `--restore` or 8BitDo's official updater as long as the receiver can enter `2dc8:3208` boot mode.
> - No proprietary 8BitDo firmware binaries are distributed in this repository. `--fetch` downloads stock public v1.03 directly from 8BitDo's official API, verifies its SHA-256, and patches it locally using `dist/u2bt-dongle-v1.03-dinput-gyro.patch.json` (which contains our compiled `pi32v2` code and the 34-byte HID report descriptor preserving the stock `0x02`/`0x81` vendor report IDs).

## Features

- **34-byte D-Input HID report with 6-axis IMU & all 4 extra buttons:**
  - Replaces stock public v1.03's legacy 15-button, 145-byte D-Input HID descriptor with the 136-byte Ultimate 2 descriptor (`Report ID 0x01`, 34 bytes: hat, sticks, triggers, 24 buttons including `L4`/`R4`/`PL`/`PR`, battery byte, and 12-byte 6-axis gyro/accelerometer).
  - Automatically requests IMU streaming from the controller (`[a1 a5 dd 00]`) when connecting in D-Input mode (at the Ultimate 2 Bluetooth controller firmware's default 100 Hz IMU rate).
  - Defaults D-Input USB PID to **`2dc8:6012`** so Steam Input and Linux `60-steam-input.rules` immediately recognize it with full gyro and back/bumper paddle support (pass `--pid 0x6011` if you prefer the stock PID).
- **Runtime mode switching (persisted in receiver NVM across replugs):**
  Hold for ~1.8 seconds while connected over 2.4G:
  - `SELECT + B` → **D-Input Gyro** (`2dc8:6012`)
  - `SELECT + X` → **XInput** (`2dc8:310b` active / `2dc8:3109` idle)
  - `SELECT + Y` → **Switch Pro** (`057e:2009`)
  - `SELECT + START` → **Default**
- **USB interrupt-IN deadlock fix for runtime mode switching (`usb_g_intr_write` @ `0x1e02384`):**
  - Stock v1.03's USB stack calls `usb_g_intr_write()` from Priority-3 `sys_hi_timer` (IRQ 5) using a Priority-1 `jiffies` timeout while blocking Priority-3 USB ISR (IRQ 9), and never clears its `usb_configured` flag on USB re-enumeration. Official stock v1.03 doesn't expose this because it lacks runtime mode switching, but once runtime mode switching is active (as in this patch or 8BitDo's unpublished beta firmware), switching modes while connected (especially XInput → Switch) could freeze the receiver until a ~4 s hardware watchdog reset or fail `SET_CONFIGURATION #1` (`-110`).
  - The replacement gates IN writes on `USB_CON0` bit 2 and true USB 2.0 `USB_STATE_CONFIGURED` (`4`), and replaces the Priority-1 timeout with Priority-6 Timer0 (`2 ms`) + per-endpoint non-blocking backoff that auto-recovers as soon as the host polls the endpoint.

## Prerequisites

1. **Python 3.10+** and **`hidapi`**:
   ```bash
   pip install hidapi
   ```
   *(On Linux, you may need `sudo` or a `udev` rule granting access to `2dc8:3208` and the receiver's app-mode VID/PIDs.)*
2. **No compiler needed to flash:** [dist/u2bt-dongle-v1.03-dinput-gyro.patch.json](dist/u2bt-dongle-v1.03-dinput-gyro.patch.json) contains our compiled `pi32v2` patch sections and 34-byte HID descriptor.

## Flashing Instructions

Plug in your **Ultimate 2 Bluetooth 2.4G USB receiver** (unplug other 8BitDo controllers/receivers first, or pass `--path`).

### 1. Dry-Run First (No USB Writes)
```bash
python3 tools/u2bt_flash.py --fetch
```
This downloads stock public v1.03 from 8BitDo (cached to `backups/u2bt-dongle-stock-v1.03.dat`), verifies its SHA-256, applies `dist/u2bt-dongle-v1.03-dinput-gyro.patch.json`, verifies all JLFS/BANKCB/app CRCs, and checks visible receiver interfaces without writing anything.

### 2. Flash the Receiver
```bash
python3 tools/u2bt_flash.py --fetch --execute
```
The tool automatically reboots the receiver into `2dc8:3208` boot mode, verifies logical PID `0x6011`, updates only the application region (`>= 0x4000`), commits and verifies the checksum + first-4K CRC, and resets the receiver back into application mode.

*(Note: if your controller is powered off after flashing, the receiver may briefly enumerate in XInput mode before settling into `2dc8:3109` idle.)*

### 3. Restoring Stock Public v1.03
```bash
python3 tools/u2bt_flash.py --fetch --restore --execute
```
*(Or pass `--base backups/u2bt-dongle-stock-v1.03.dat --restore --execute` offline, or use 8BitDo's official Firmware Updater.)*

### Optional Flags
- `--pid 0x6011` — Keep stock D-Input PID instead of `0x6012`.
- `--name "8BitDo UM 2 Receiver"` — Custom 20-character ASCII USB product string for D-Input mode.
- `--base <path/to/stock.dat>` — Use a local stock public v1.03 `.dat` instead of `--fetch`.

## Building from Source (Developers)

Requires the JieLi `pi32v2` LLVM toolchain (install to `~/.local/opt/pi32v2` from `https://pkgman.jieliapp.com/s/linux-toolchain`, or set `PI32V2_BIN=/path/to/pi32v2/bin`):

```bash
# Run host unit tests (C report packer, hotkeys, USB TX deadlock fix + Python JLFS/repack tests)
cc -std=gnu99 -Wall -Wextra -Werror -DHOST_TEST \
    patches/u2bt_dongle/dinput_gyro.c \
    patches/u2bt_dongle/usb_tx_fix.c \
    patches/u2bt_dongle/test_u2bt_dongle.c -o /tmp/test_u2bt_dongle
/tmp/test_u2bt_dongle
python3 patches/u2bt_dongle/test_u2bt_tools.py

# Rebuild patched .dat and regenerate dist/u2bt-dongle-v1.03-dinput-gyro.patch.json
python3 patches/u2bt_dongle/build.py \
    --fetch \
    --export-patch dist/u2bt-dongle-v1.03-dinput-gyro.patch.json
```

## Credits & References

- [kagaimiq/jielie](https://kagaimiq.github.io/jielie/datafmt/newfw.html) & [kagaimiq/jl-misctools](https://github.com/kagaimiq/jl-misctools) — JieLi BR22 JLFS/BANKCB container format, ENC/SFC scrambling, and chip-key derivation
- [JieLi `pi32v2` Linux LLVM toolchain](https://pkgman.jieliapp.com/s/linux-toolchain) — Compiler, linker, and disassembler for JieLi `pi32v2`
- [libsdl-org/SDL](https://github.com/libsdl-org/SDL) (`SDL_hidapi_8bitdo.c`) — 8BitDo Ultimate 2 34-byte D-Input report layout and IMU format
- Hardware notes for developers:
  - The 2.4G receiver firmware (type `98`) contains `JL-BR22` / `br22xx` SDK strings and executes JieLi `pi32v2` code, pointing to a **JieLi BR22-family** SoC.
  - For reference, the **Ultimate 2 Bluetooth controller** itself (type `97`, not patched here) uses a **Yichip `YC3121-L`** (QFN56) per the [FCC ID `2AOWFULT2N` internal photos](https://fccid.io/2AOWFULT2N) ([YC3121-L datasheet](https://atta.szlcsc.com/upload/public/pdf/source/20211111/30894FF75D776498D9212EBEB2236820.pdf) · [Yichip YC3121 repo](https://github.com/Yichip-Microelectronics/YC3121)).
