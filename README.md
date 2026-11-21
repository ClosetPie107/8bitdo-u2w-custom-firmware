# 8BitDo Ultimate 2 Wireless — Custom Firmware

Unofficial open-source Telink TLSR8258 firmware patches for the **8BitDo Ultimate 2 Wireless** (`VID 0x2DC8`, `PID 0x6012`, stock firmware **v1.10**), adding native **Nintendo Switch 2 Pro Controller BLE** support and **persistent Bluetooth/2.4G mode memory** while keeping stock 8BitDo Bluetooth, 2.4G dongle, and USB modes intact.

> [!WARNING]
> **Flashing custom firmware carries risk.**
> - This is an **unofficial**, community-made modification with **no warranty** of any kind.
> - Flashing the wrong controller model or interrupting power/USB mid-write can **brick** your controller.
> - **Supported hardware ONLY:** **8BitDo Ultimate 2 Wireless** (Xbox layout, logical PID `0x6012`) running **stock firmware v1.10**.
> - **Do NOT flash** this onto the *Ultimate 2 Bluetooth* (Switch layout), *Ultimate 2C*, or any other controller/version.
> - No proprietary 8BitDo firmware binaries are distributed in this repository. The flasher patches your own stock v1.10 firmware in memory and saves a local backup first.

## Features

- **Native Nintendo Switch 2 BLE Pro Controller emulation:**
  - BLE 5.0 CSA #2 channel selection, 2M PHY, and **5 ms (200 Hz)** polling interval
  - Full Switch 2 pairing (`0x15` handshake + LTK encryption) with **dual-sector flash bond persistence**
  - **Wake Switch 2 from sleep** with the Home button
  - **6-axis IMU / gyro & accelerometer** (Switch 2 native 40-byte `0x09` motion packing + fixed-point quaternion integration)
  - **Vibration / rumble:** both in-game streaming HD Rumble 2 (handle `0x0012`) and system UI vibration presets (`0x0A/02` connect buzz, vibration toggle, Search for Controllers)
  - **Rear paddles mapped to GL / GR** (Nintendo Pro Controller 2 grip buttons)
  - **Battery level reporting** and stock front profile LED behavior
- **Persistent mode memory (saved in flash across power cycles):**
  - **Bluetooth (BT switch):**
    - `Home + Y` → **Switch 2 BLE** (saved)
    - `Home + B` → **Stock 8BitDo BLE (DInput)** (saved, full PC/Steam gyro & paddles)
    - `Home` → **Last used** Bluetooth mode (defaults to Switch 2 BLE)
  - **2.4G Dongle (2.4G switch):**
    - `Home + B` → **DInput** (saved)
    - `Home + X` → **XInput** (saved)
    - `Home + Y` → **Switch** (saved)
    - `Home` → **Last used** 2.4G mode (defaults to XInput)

> [!NOTE]
> **Why Switch 2 Bluetooth only (no Switch 1 Bluetooth)?**
> This is a hardware limitation: the Ultimate 2 Wireless uses a **Telink TLSR8258** radio SoC, which supports **Bluetooth Low Energy (BLE) only** and has no Bluetooth Classic (BR/EDR) hardware. Nintendo Switch 1 controllers communicate over Bluetooth Classic, whereas Nintendo Switch 2 moved to BLE. For Switch 1, use the controller's **2.4G USB dongle** (`Home + Y` Switch mode) or a wired USB connection.

## Prerequisites

1. **Update your Ultimate 2 Wireless to stock v1.10** using 8BitDo's official Ultimate Software / Firmware Updater before starting.
2. **Python 3.10+** and **`hidapi`**:
   ```bash
   pip install hidapi
   ```
   *(On Linux, you may need `sudo` or a `udev` rule granting access to `2dc8:3208`.)*
3. **No compiler needed to flash:** `dist/u2w-v1.10-switch2.patch.json` contains only the original compiled TC32 patch sections (~10 KB, zero 8BitDo bytes).

## Flashing Instructions

### 1. Enter USB Boot Mode
1. Unplug the controller and turn it **off** (hold **Home** until LEDs turn off).
2. Hold **LB + RB** together.
3. While holding both bumpers, plug the controller into your PC via **USB**.
4. Release the bumpers. The controller enumerates as `2dc8:3208` (`lsusb` on Linux).

### 2. Dry-Run First (Read-Only)
```bash
python3 tools/flash.py
```
*(Use `sudo python3 tools/flash.py` on Linux if `/dev/hidraw*` requires root.)*

This will:
- Verify the device is an **Ultimate 2 Wireless** (`PID 0x6012`) running **stock v1.10** (`SHA-256 43d8cd22...`).
- Automatically save a local backup of your stock v1.10 app to `backups/u2w-stock-v1.10.image.bin`.
- Apply `dist/u2w-v1.10-switch2.patch.json` in memory, verify all safety invariants (RISC-V peer byte-identical, CRC-32 + checksum valid), and print which flash sectors would change without writing anything.

### 3. Flash the Controller
```bash
python3 tools/flash.py --execute
```
Wait ~15–25 seconds until you see `SUCCESS: ... sector(s) flashed and verified`. Header sector `0x5000` is always programmed last, and every sector is read back and verified immediately after writing.

### 4. Power-Cycle & Pair with Switch 2
1. Unplug USB.
2. Slide the rear switch to **BT**, hold **Home** to power off if needed, then press **Home** (or **Home + Y** if you previously used 8BitDo BLE).
3. On Switch 2, open **Controllers → Change Grip/Order**, hold the controller's **Pair** button until the LED flashes rapidly, and wait for it to pair.
4. Once paired, pressing **Home** wakes the console and reconnects automatically.

## Upgrading or Restoring Stock v1.10

- **Upgrade to a newer patch** (when already running this custom firmware):
  ```bash
  python3 tools/flash.py --base backups/u2w-stock-v1.10.image.bin --execute
  ```
- **Restore exact stock v1.10:**
  ```bash
  python3 tools/flash.py --restore backups/u2w-stock-v1.10.image.bin --execute
  ```
  Restoring stock v1.10 replaces the application region `[0x5000, 0x28800)` while leaving the separate flash sectors (`0x2b000`/`0x2c000`) used for Switch 2 bond and mode memory untouched. You can also re-flash stock firmware at any time using 8BitDo's official firmware updater while in `LB + RB + USB` boot mode.
- **Optional full 512 KiB flash dump (read-only):**
  ```bash
  python3 tools/u2w_read_flash.py --boot --start 0 --length 0x80000 -o backups/u2w-full-512k.bin
  ```
  *(Saved with owner-only `0600` permissions since full flash dumps contain device MAC/bond data.)*

## Building from Source (Developers)

Requires the Telink `tc32-elf-gcc` toolchain (on `PATH`, at `~/.local/opt/tc32/bin`, or via `TC32_BIN=/path/to/tc32/bin`) and a local copy of your stock v1.10 app (`backups/u2w-stock-v1.10.image.bin` or `u2w-1.10.dat`):

```bash
# Run host unit tests (exercises C protocol, GATT, CSA #2, and PHY code directly)
cc -std=gnu99 -Wall -Wextra -Werror -DHOST_TEST \
    patches/csa2.c patches/s2_gatt.c patches/s2_proto.c patches/test_s2_proto.c -o /tmp/test_s2_proto
/tmp/test_s2_proto
python3 patches/test_csa2.py

# Build patched image and regenerate dist/u2w-v1.10-switch2.patch.json
python3 patches/build.py \
    --base backups/u2w-stock-v1.10.image.bin \
    --export-patch dist/u2w-v1.10-switch2.patch.json
```

## License

MIT — see [LICENSE](LICENSE).

## Credits & References

### 8BitDo Firmware, Bootloader & Flashing
- [Crazyawk/8bitdo-ultimate2c-macos-rumble](https://github.com/Crazyawk/8bitdo-ultimate2c-macos-rumble) — Field-tested Telink `0x3208` boot-mode backup and flashing protocol (HID report ID `5`, class-`05` read/erase/write/reset framing)
- [ascendedent/8bitdo-Linux-Software](https://github.com/ascendedent/8bitdo-Linux-Software) — Ultimate 2 protocol framing, device IDs, and report-`81` / `0x0005` firmware notes
- [crackheadakira/8BitDoRetroKeyboardFlasher](https://github.com/crackheadakira/8BitDoRetroKeyboardFlasher) — 8BitDo Telink HID firmware flashing precedent
- [aeromodes/8cryptdo](https://github.com/aeromodes/8cryptdo) — 8BitDo `.dat` firmware container encryption/decryption reference
- [fwupd/fwupd (`plugins/ebitdo`)](https://github.com/fwupd/fwupd/tree/main/plugins/ebitdo) & [fwupd/8bitdo-firmware](https://github.com/fwupd/8bitdo-firmware) — Legacy 8BitDo HID firmware format and archive reference
- [FCC ID `2AOWF-24GULT2` Internal Photos](https://fccid.io/2AOWF24GULT2/Internal-Photos/Internal-Photos-8068309) — Hardware identification of the Telink `TLSR8258` (QFN48) and CH32V203 peer MCU
- [libsdl-org/SDL](https://github.com/libsdl-org/SDL) (`SDL_hidapi_8bitdo.c`) — 8BitDo Ultimate 2 Wireless IMU axis orientation and scale reference

### Telink TLSR8258 & TC32 Tooling
- [Telink Platform SDK Handbook](https://doc.telink-semi.cn/doc/en/software/res/sdk/driver/platform_sdk_handbook_en/) & [telink-semi/tc_ble_single_sdk](https://github.com/telink-semi/tc_ble_single_sdk) — `TLNK` image layout, flash/OTA flow, and BLE stack conventions
- [flyskywhy/tc32](https://github.com/flyskywhy/tc32) — Prebuilt `tc32-elf-*` GCC toolchain binaries
- [rgov/Ghidra_TELink_TC32](https://github.com/rgov/Ghidra_TELink_TC32) & [HackSucks/OpenTC32](https://github.com/HackSucks/OpenTC32) — TC32 Ghidra disassembly/decompilation tooling

### Switch 2 BLE Protocol & Motion
- [ndeadly/switch2_controller_research](https://github.com/ndeadly/switch2_controller_research) — Switch 2 BLE GATT layout, pairing handshake, commands, and rumble captures
- [novakpetya/linux-switch2](https://github.com/novakpetya/linux-switch2) — Switch 2 native 40-byte `0x09` motion decoding, quaternion smallest-three format, and calibration layout
- [espp Switch 2 Pro BLE documentation](https://esp-cpp.github.io/espp/ble/switch2_pro.html) — Device-side Switch 2 BLE reference
- [Reverse Engineering the Switch 2 Pro Controller's Bluetooth Protocol](https://leonsnotes.ca/2026/04/04/reverse-engineering-the-switch-2-pro-controllers-bluetooth-protocol/) — Protocol analysis notes
