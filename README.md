# 8BitDo Ultimate 2 Wireless — Custom Firmware (Switch 1 BLE + Stock + Xbox Elite BLE)

Unofficial open-source Telink TLSR8258 firmware patches for the **8BitDo Ultimate 2 Wireless** (`VID 0x2DC8`, `PID 0x6012`, stock firmware **v1.10**), replacing Switch 2 mode with native **Nintendo Switch 1 Pro Controller (`057e:2009`) BLE HOGP** support (for **macOS**, Steam, SDL3, and Linux), **4 universal Bluetooth device slots**, **Xbox Elite Series 2 BLE** mode, and **persistent Bluetooth/2.4G mode memory** while keeping stock 8BitDo Bluetooth, 2.4G dongle, and USB modes intact.

> [!WARNING]
> **Flashing custom firmware carries risk.**
> - This is an **unofficial**, community-made modification with **no warranty** of any kind.
> - Flashing the wrong controller model or interrupting power/USB mid-write can **brick** your controller.
> - **Supported hardware ONLY:** **8BitDo Ultimate 2 Wireless** (Xbox layout, logical PID `0x6012`) running **stock firmware v1.10**.
> - **Do NOT flash** this onto the *Ultimate 2 Bluetooth* (Switch layout), *Ultimate 2C*, or any other controller/version.
> - No proprietary 8BitDo firmware binaries are distributed in this repository.

## Features

- **Native Nintendo Switch 1 Pro Controller (`057e:2009`) over BLE HOGP (`Pro Controller`):**
  - Standard BLE HID over GATT (`0x1812`), Device Information (`0x180A`, PnP ID `057e:2009`), Battery Service (`0x180F`), and standard Just Works SMP bonding (`kSmpSlotAddr[slot]` per-slot flash isolation)
  - Recognized natively on **macOS** by `AppleGameControllerPersonality.kext` + `JoyConHIDServicePlugin` (`GameController` framework, Steam, SDL3, browsers/emulators) as well as Linux BlueZ / `hid-nintendo`
  - **BLE 5.0 CSA #2** channel selection and **2M PHY** support
  - **Input reports `0x3F` (simple HID), `0x30` (standard full 60–120 Hz + IMU), and `0x21` (subcommand replies)**
  - **6-axis IMU (gyro & accelerometer):** live U2W IMU samples mapped into Switch 1 frames with factory calibration at SPI `0x6020` (`accelCoeff=0x4000`, `gyroCoeff=0x343B`)
  - **12-bit analog sticks** with quirk-safe Switch 1 SPI factory calibration at `0x603D`, plus `0x6000` serial and `0x6050` colors
  - **HD Rumble (`0x10` / `0x01`)** decoded into independent left/right U2W ERM motors
  - **Square (Turbo) button mapped to Capture**
- **Home Ring low-brightness mode:**
  - Switches the white Home Ring LED from blinding full-on GPIO output to ~1% brightness PWM (`PWM5` on `PB5`, transport-independent across BT, 2.4G, and USB)
- **Linux / Steam Input Bluetooth PnP ID fix:**
  - Fixes stock v1.10's invalid BLE PnP ID Vendor Source byte (`0x12` → `0x02`) so Linux BlueZ properly exposes `2dc8:6012`
- **Xbox Elite Series 2 BLE mode (`8BitDo Ultimate 2 Xbox`, `045e:0b05`):**
  - Exposes rumble (Apple Core Haptics & SDL), battery level, all 4 rear paddles (`P1`–`P4`), and raw IMU tail bytes
- **4 Universal Bluetooth device slots (saved in flash across power cycles):**
  - Each D-pad direction (**Up**, **Down**, **Left**, **Right**) is an independent device slot with its own **BLE mode**, **MAC address**, and **SMP pairing bond**.
  - **Factory defaults:** `Up` = Stock 8BitDo, `Down` = Stock 8BitDo, `Left` = Xbox Elite, `Right` = Switch 1 Pro Controller.
  - **Bluetooth (BT switch, from off):**
    - `Home` → Reconnect to the **last-used** slot
    - `Home + Up / Down / Left / Right` → **Recall** that slot in whatever BLE mode is saved to it
    - `D-pad + B / X / Y + Home` → **Assign** that slot's BLE mode (`B` = Stock 8BitDo, `X` = Xbox Elite, `Y` = Switch 1 Pro). Changing a slot's mode bumps its MAC and clears its old SMP bond so you can re-pair cleanly.
    - `Home + B / X / Y` (no D-pad) → Jump to the **last-used slot** of that mode (or reassign the active slot if no slot uses it)
  - **2.4G Dongle (2.4G switch, from off):**
    - `Home + B` → **DInput** (saved)
    - `Home + X` → **XInput** (saved)
    - `Home + Y` → **Switch** (saved)
    - `Home` → **Last used** 2.4G mode (defaults to XInput)

## Building & Flashing

Requires the Telink `tc32-elf-gcc` toolchain (on `PATH`, at `~/.local/opt/tc32/bin`, or via `TC32_BIN=/path/to/tc32/bin`), Python 3.10+, and `hidapi` (`pip install hidapi`).

### 1. Run Host Unit Tests & Verify ELF Build
```bash
cc -std=gnu99 -Wall -Wextra -Werror -DHOST_TEST \
    patches/csa2.c patches/sw1_gatt.c patches/sw1_proto.c patches/test_sw1_proto.c -o /tmp/test_sw1_proto
/tmp/test_sw1_proto

cc -std=gnu99 -Wall -Wextra -Werror -DHOST_TEST \
    patches/elite.c patches/test_elite.c -o /tmp/test_elite
/tmp/test_elite

python3 patches/test_csa2.py
python3 patches/test_home_ring.py
python3 patches/build.py --elf-only
```

### 2. Backup Stock Firmware & Build Patch
Put the controller into USB boot mode (`LB + RB` while plugging USB in -> `2dc8:3208`), back up stock v1.10 (or use an existing `u2w-1.10.dat` / backup), and build:
```bash
# Optional: if controller is still on stock v1.10, dump app region first:
python3 tools/u2w_read_flash.py --boot --start 0x5000 --length 0x23800 -o backups/u2w-stock-v1.10.image.bin

# Build patched image and export dist/u2w-v1.10-switch1.patch.json:
python3 patches/build.py \
    --base backups/u2w-stock-v1.10.image.bin \
    --export-patch dist/u2w-v1.10-switch1.patch.json

# Dry-run first, then flash:
python3 tools/flash.py --base backups/u2w-stock-v1.10.image.bin
python3 tools/flash.py --base backups/u2w-stock-v1.10.image.bin --execute
```

### 3. Pair on macOS (Switch 1 BLE)
1. Unplug USB, set the rear switch to **BT**, and power off (**hold Home**).
2. Power on with **Home + Right** (or **Home + Y**) — wait until LED blinks, then hold **Pair**.
3. On macOS, open **System Settings → Bluetooth** and connect to **`Pro Controller`**.
4. In **System Settings → Game Controllers** (or Steam / SDL3 apps), the controller appears as a Nintendo Switch Pro Controller with buttons, sticks, rumble, and 6-axis gyro/accel motion.
   *(If you previously paired the same Bluetooth MAC under another mode, forget the device on macOS first or re-assign the slot mode / hold Pair at boot to bump the slot MAC.)*

## License

MIT — see [LICENSE](LICENSE).

## Credits & References

### 8BitDo Firmware, Bootloader & Flashing
- [craigmdennis/8bitdo-ultimate-toolkit](https://github.com/craigmdennis/8bitdo-ultimate-toolkit/blob/u2w-xbox-rumble/u2w_xbox/README.md) — Xbox Elite Series 2 (`045e:0b05`) BLE identity, 39-byte HID report layout, paddle mapping, and Core Haptics / SDL rumble scaling reference
- [Crazyawk/8bitdo-ultimate2c-macos-rumble](https://github.com/Crazyawk/8bitdo-ultimate2c-macos-rumble) — Field-tested Telink `0x3208` boot-mode backup and flashing protocol (HID report ID `5`, class-`05` read/erase/write/reset framing)
- [ascendedent/8bitdo-Linux-Software](https://github.com/ascendedent/8bitdo-Linux-Software) — Ultimate 2 protocol framing, device IDs, and report-`81` / `0x0005` firmware notes
- [crackheadakira/8BitDoRetroKeyboardFlasher](https://github.com/crackheadakira/8BitDoRetroKeyboardFlasher) — 8BitDo Telink HID firmware flashing precedent
- [aeromodes/8cryptdo](https://github.com/aeromodes/8cryptdo) — 8BitDo `.dat` firmware container encryption/decryption reference
- [fwupd/fwupd (`plugins/ebitdo`)](https://github.com/fwupd/fwupd/tree/main/plugins/ebitdo) & [fwupd/8bitdo-firmware](https://github.com/fwupd/8bitdo-firmware) — Legacy 8BitDo HID firmware format and archive reference
- The FCC internal photos show the **Telink TLSR8258** (QFN48). [FCC ID `2AOWF-24GULT2` internal photos](https://fccid.io/2AOWF24GULT2/Internal-Photos/Internal-Photos-8068309)
- The stock v1.09 and v1.10 peer firmware points to **WCH CH32X035**. Both images handle the fourth UART at `0x40004c00` on IRQ 43, which matches CH32X035. WCH's CH32V20x SDK assigns that UART to IRQ 61 or 66, depending on the variant. Biase-d also reported reading CH32X035 on their unit. [CH32X035 SDK](https://github.com/openwch/ch32x035/blob/main/EVT/EXAM/SRC/Peripheral/inc/ch32x035.h) · [CH32V20x SDK](https://github.com/openwch/ch32v20x/blob/main/EVT/EXAM/SRC/Peripheral/inc/ch32v20x.h) · [Biase-d's report](https://gist.github.com/barraIhsan/783a82bcf32bed896c85d27dbb8018a5?permalink_comment_id=6490288)
- [libsdl-org/SDL](https://github.com/libsdl-org/SDL) (`SDL_hidapi_8bitdo.c` & `SDL_hidapi_switch.c`) — 8BitDo Ultimate 2 Wireless IMU axis orientation/scale and Nintendo Switch 1 Pro Controller protocol reference

### Telink TLSR8258 & TC32 Tooling
- [Telink Platform SDK Handbook](https://doc.telink-semi.cn/doc/en/software/res/sdk/driver/platform_sdk_handbook_en/) & [telink-semi/tc_ble_single_sdk](https://github.com/telink-semi/tc_ble_single_sdk) — `TLNK` image layout, flash/OTA flow, and BLE stack conventions
- [flyskywhy/tc32](https://github.com/flyskywhy/tc32) — Prebuilt `tc32-elf-*` GCC toolchain binaries
- [rgov/Ghidra_TELink_TC32](https://github.com/rgov/Ghidra_TELink_TC32) & [HackSucks/OpenTC32](https://github.com/HackSucks/OpenTC32) — TC32 Ghidra disassembly/decompilation tooling
