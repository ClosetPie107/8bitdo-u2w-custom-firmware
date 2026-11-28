#!/usr/bin/env python3
"""Patch and flash the 8BitDo Ultimate 2 Bluetooth 2.4G receiver (type 98, JieLi BR22).

Implements the WebHID JieLi receiver update sequence (without needing a browser or
manual second-device picker after rebooting into 2dc8:3208 boot mode).

Default workflow (no compiler needed; downloads stock public v1.03 directly from 8BitDo):
  1. Dry-run (validates image & checks receiver with no USB writes):
       python3 tools/u2bt_flash.py --fetch
  2. Flash patched firmware (auto-reboots receiver into 2dc8:3208):
       python3 tools/u2bt_flash.py --fetch --execute
  3. Restore exact stock public v1.03:
       python3 tools/u2bt_flash.py --fetch --restore --execute

Safety guarantees:
  - Validates the stock .dat, pinned BR22 container layout (chip key, app_base, entry point,
    app.bin size), and patched output via u2bt_extract.extract() before opening any USB device.
  - Only flashes 2dc8:3208 devices whose logical PID (CMD_IDENTIFY) is 0x6011
    (refuses to touch U2W 0x6012 or any other 8BitDo hardware).
  - Snapshots pre-existing 2dc8:3208 boot devices before rebooting so auto-reboot only
    attaches to the newly rebooted receiver.
  - Never erases uboot.boot or isd_config.ini: uses the bootloader-returned offset
    (0x4000) to update only the application area.
  - Dry-run by default (no USB writes); requires --execute to erase/program/reset.
"""

from __future__ import annotations

import argparse
import hashlib
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import u2bt_patch as up  # noqa: E402

try:
    import hid
except ImportError:
    hid = None  # type: ignore[assignment]

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATCH = ROOT / "dist" / "u2bt-dongle-v1.03-dinput-gyro.patch.json"
DEFAULT_BACKUP = ROOT / "backups" / "u2bt-dongle-stock-v1.03.dat"

VID_8BITDO = 0x2DC8
VID_NINTENDO = 0x057E
BOOT_PID = 0x3208
LOGICAL_PID = 0x6011

# App-mode VID/PIDs the Ultimate 2 Bluetooth 2.4G receiver can enumerate as
APP_VID_PIDS = {
    (VID_8BITDO, 0x6011),    # Stock D-Input
    (VID_8BITDO, 0x6012),    # Patched D-Input (Steam Input Ultimate 2 profile)
    (VID_8BITDO, 0x3109),    # Idle / XInput
    (VID_8BITDO, 0x310B),    # Active XInput (controller connected)
    (VID_NINTENDO, 0x2009),  # Switch mode
}

CLASS = 0x05
CMD_WRITE = 0x0003
CMD_ERASE = 0x0004
CMD_RESET = 0x0007
CMD_IDENTIFY = 0x0008
CMD_JUMP_BOOT = 0x0050
CMD_INIT = 0x00C1
CMD_FINISH = 0x00C2
CMD_CRC_COMMIT = 0x00C3

REPORT_SIZE = 64
HEADER_LEN = 18
WRITE_CHUNK = 46


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def sum_u32_le(data: bytes) -> int:
    if len(data) % 4 != 0:
        raise ValueError(f"Payload length {len(data)} is not 4-byte aligned")
    total = 0
    for (word,) in struct.iter_unpack("<I", data):
        total = (total + word) & 0xFFFFFFFF
    return total


def build_packet(
    cmd: int,
    *,
    report_id: int = 0x00,
    pkt_class: int = CLASS,
    cmd_params: int = 0,
    addr: int = 0,
    length: int = 0,
    total_len: int = 0,
    payload: bytes = b"",
) -> bytes:
    if len(payload) > WRITE_CHUNK:
        raise ValueError(f"payload too long: {len(payload)}")
    crc = crc16_modbus(payload) if payload else 0

    pkt = bytearray(REPORT_SIZE + (1 if report_id == 0 else 0))
    pkt[0] = report_id
    fields = memoryview(pkt)[1:]
    fields[0] = pkt_class
    fields[1:3] = cmd.to_bytes(2, "little")
    fields[3:5] = cmd_params.to_bytes(2, "little")
    fields[5:7] = length.to_bytes(2, "little")
    fields[7:9] = crc.to_bytes(2, "little")
    fields[9:13] = total_len.to_bytes(4, "little")
    fields[13:17] = addr.to_bytes(4, "little")
    fields[17 : 17 + len(payload)] = payload
    return bytes(pkt)


def drain_hid_input(dev, report_size: int = REPORT_SIZE, max_packets: int = 64) -> None:
    """Drain queued HID input reports without blocking on cython-hidapi or ctypes hid."""
    if hasattr(dev, "set_nonblocking"):
        dev.set_nonblocking(1)
        try:
            for _ in range(max_packets):
                if not dev.read(report_size):
                    break
        finally:
            dev.set_nonblocking(0)
    else:
        for _ in range(max_packets):
            if not dev.read(report_size, 1):
                break


def open_hid_path(path: bytes):
    if hid is None:
        sys.exit("python hidapi is required: pip install hidapi")
    if hasattr(hid, "device"):
        dev = hid.device()
        dev.open_path(path)
        return dev
    return hid.Device(path=path)


def _usb_device_group(d: dict) -> tuple:
    """Group multiple HID interfaces belonging to the same physical USB device."""
    raw_path = d.get("path") or b""
    path_s = raw_path.decode("utf-8", "replace") if isinstance(raw_path, bytes) else str(raw_path)
    # Linux hidraw-libusb paths look like "0003:0006:01" (bus:dev:iface); strip interface suffix.
    parts = path_s.split(":")
    if len(parts) == 3 and all(p.isalnum() for p in parts):
        loc = f"{parts[0]}:{parts[1]}"
    elif path_s.startswith("/dev/hidraw"):
        # Resolve sysfs parent USB device directory (strips :1.x interface suffix).
        try:
            real = str(Path("/sys/class/hidraw", Path(path_s).name, "device").resolve())
            loc = real.rsplit("/", 1)[0].rsplit(":", 1)[0]
        except OSError:
            loc = path_s
    else:
        loc = path_s.rsplit("#", 1)[0] if "#" in path_s else path_s
    return (d.get("vendor_id"), d.get("product_id"), loc)


def find_boot_devices(path: str | None = None) -> list[dict]:
    if hid is None:
        sys.exit("python hidapi is required: pip install hidapi")
    devs = [d for d in hid.enumerate(VID_8BITDO, BOOT_PID)]
    if path is not None:
        want = path.encode()
        devs = [d for d in devs if d["path"] == want]
    return devs


def _is_likely_u2bt_receiver(d: dict, *, explicit_path: bool = False) -> bool:
    """Filter out unrelated 2dc8:6012 devices (e.g. a wired Ultimate 2 Wireless controller)."""
    if explicit_path:
        return True
    if d.get("vendor_id") == VID_8BITDO and d.get("product_id") == 0x6012:
        prod = (d.get("product_string") or "").lower()
        # Require a receiver-identifying product string unless --path was passed explicitly.
        if not prod:
            return False
        if "wireless" in prod and "receiver" not in prod and "gyrod" not in prod:
            return False
        if "receiver" not in prod and "gyrod" not in prod and "um 2" not in prod:
            return False
    return True


def find_app_candidates(path: str | None = None) -> list[dict]:
    if hid is None:
        sys.exit("python hidapi is required: pip install hidapi")
    out = []
    want = path.encode() if path is not None else None
    for vid, pid in APP_VID_PIDS:
        for d in hid.enumerate(vid, pid):
            if want is not None and d["path"] != want:
                continue
            if _is_likely_u2bt_receiver(d, explicit_path=(want is not None)):
                out.append(d)
    return out


class BootDongle:
    def __init__(self, dev_info: dict, *, timeout_ms: int = 2000, retries: int = 3):
        self.info = dev_info
        self.timeout_ms = timeout_ms
        self.retries = retries
        self.dev = open_hid_path(dev_info["path"])

    def close(self) -> None:
        try:
            self.dev.close()
        except Exception:
            pass

    def flush(self) -> None:
        drain_hid_input(self.dev, REPORT_SIZE)

    def send_raw(self, pkt: bytes) -> None:
        written = self.dev.write(pkt)
        if written < len(pkt):
            raise RuntimeError(f"hid_write returned {written}, expected >= {len(pkt)}")

    def command(
        self,
        cmd: int,
        *,
        addr: int = 0,
        length: int = 0,
        total_len: int = 0,
        payload: bytes = b"",
        timeout_ms: int | None = None,
        expect_reply: bool = True,
        min_reply_len: int = 0,
        retries: int | None = None,
    ) -> bytes:
        pkt = build_packet(
            cmd,
            report_id=0x00,
            addr=addr,
            length=length,
            total_len=total_len,
            payload=payload,
        )
        wait_ms = self.timeout_ms if timeout_ms is None else timeout_ms
        attempts = self.retries if retries is None else retries
        if not expect_reply:
            self.flush()
            self.send_raw(pkt)
            return b""

        last: Exception | None = None
        for _ in range(attempts):
            self.flush()
            try:
                self.send_raw(pkt)
                raw = bytes(self.dev.read(REPORT_SIZE, wait_ms))
                if not raw:
                    last = TimeoutError(f"timeout waiting for cmd {cmd:#06x}")
                    continue
                if len(raw) < HEADER_LEN + min_reply_len:
                    last = ValueError(
                        f"short reply ({len(raw)} bytes, need >= {HEADER_LEN + min_reply_len})"
                    )
                    continue
                if not raw.startswith(bytes((0x00, CLASS))):
                    last = ValueError(f"unexpected reply prefix {raw[:4].hex(' ')}")
                    continue
                status = int.from_bytes(raw[2:4], "little")
                echo = int.from_bytes(raw[4:6], "little")
                if status != 0:
                    raise RuntimeError(f"device error status {status:#06x} for cmd {cmd:#06x}")
                if echo != cmd:
                    last = ValueError(f"reply echoed {echo:#06x}, expected {cmd:#06x}")
                    continue
                return raw[HEADER_LEN:]
            except Exception as exc:
                last = exc
        raise RuntimeError(f"cmd {cmd:#06x} failed after {attempts} attempt(s): {last}")

    def identify(self) -> int:
        data = self.command(CMD_IDENTIFY, length=4, total_len=4, min_reply_len=2)
        return int.from_bytes(data[:2], "little")


def try_reboot_app_to_boot(path: str | None = None) -> bool:
    """Send jump-to-boot (class 0x00, cmd 0x0050 on Report ID 0x81) to enter 2dc8:3208."""
    cands = find_app_candidates(path)
    if not cands:
        return False

    # Refuse if multiple distinct physical USB devices match and --path was not specified.
    groups = {_usb_device_group(d) for d in cands}
    if path is None and len(groups) > 1:
        lines = [
            f"  {d['vendor_id']:04x}:{d['product_id']:04x} path={d['path']} product={d.get('product_string')!r}"
            for d in cands
        ]
        raise RuntimeError(
            "Multiple physical 8BitDo/Switch HID devices found; unplug other controllers/receivers or pass --path:\n"
            + "\n".join(lines)
        )

    pkt = build_packet(CMD_JUMP_BOOT, report_id=0x81, pkt_class=0x00)
    sent_any = False
    for d in cands:
        try:
            dev = open_hid_path(d["path"])
            try:
                if dev.write(pkt) >= len(pkt):
                    sent_any = True
            finally:
                dev.close()
        except Exception:
            continue
    return sent_any


def acquire_boot_dongle(*, allow_reboot: bool, path: str | None = None) -> BootDongle:
    boots = find_boot_devices(path)
    if not boots and allow_reboot:
        # Snapshot ALL pre-existing 2dc8:3208 boot devices so we never attach to an unrelated one.
        pre_existing_boot_paths = {d["path"] for d in find_boot_devices()}
        print("Dongle not in 2dc8:3208 boot mode; sending jump-to-boot (0x0050)...")
        if try_reboot_app_to_boot(path):
            for _ in range(50):
                time.sleep(0.1)
                current = find_boot_devices()
                new_boots = [d for d in current if d["path"] not in pre_existing_boot_paths]
                if new_boots:
                    boots = new_boots
                    break
    if not boots:
        raise RuntimeError(
            "No newly rebooted or matching 2dc8:3208 bootloader device found. Make sure the "
            "Ultimate 2 Bluetooth 2.4G receiver is plugged in (or pass --path for a specific device)."
        )

    matched: list[BootDongle] = []
    for info in boots:
        dongle = BootDongle(info)
        try:
            pid = dongle.identify()
            if pid == LOGICAL_PID:
                matched.append(dongle)
                continue
            print(f"Skipping 2dc8:3208 device with logical PID {pid:#06x} (want {LOGICAL_PID:#06x})")
            dongle.close()
        except Exception:
            dongle.close()

    if not matched:
        raise RuntimeError(f"Found 2dc8:3208 device(s), but none reported logical PID {LOGICAL_PID:#06x}")
    if len(matched) > 1:
        for d in matched:
            d.close()
        raise RuntimeError("Multiple 2dc8:3208 devices with logical PID 0x6011 found; pass --path.")
    return matched[0]


def resolve_target_dat(args: argparse.Namespace) -> tuple[bytes, str]:
    if args.dat is not None:
        raw = args.dat.read_bytes()
        up.validate_container_layout(raw)
        return raw, f"pre-built .dat ({args.dat})"

    if args.fetch:
        stock_dat = up.fetch_public_103(cache_path=args.backup_out)
    elif args.base is not None:
        stock_dat = up.load_stock_dat(args.base, require_exact_v103=args.restore)
    elif args.backup_out.is_file():
        stock_dat = up.load_stock_dat(args.backup_out, require_exact_v103=args.restore)
        print(f"Using cached stock public v1.03 at {args.backup_out}")
    else:
        raise SystemExit(
            "Provide --fetch (download stock v1.03 from 8BitDo), --base <stock.dat>, or --dat <image.dat>."
        )

    if args.restore:
        return stock_dat, "RESTORE exact stock public v1.03"

    patched = up.apply_patch_file(
        stock_dat,
        args.patch,
        pid=args.pid,
        product_name=args.name,
    )
    if args.output is not None:
        up.atomic_write_bytes(args.output, patched)
        print(f"Wrote patched image to {args.output}")
    return patched, f"PATCH & FLASH using {args.patch} (D-Input PID {args.pid:#06x})"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--fetch",
        action="store_true",
        help="Download stock public v1.03 from 8BitDo's official API (cached to --backup-out)",
    )
    ap.add_argument(
        "--base",
        type=Path,
        help="Path to local stock type 98 public v1.02/v1.03 .dat",
    )
    ap.add_argument(
        "--dat",
        type=Path,
        help="Flash a pre-built, already-patched type 98 .dat directly",
    )
    ap.add_argument(
        "--patch",
        type=Path,
        default=DEFAULT_PATCH,
        help=f"Path to u2bt-br22-patch-v1 JSON file (default: {DEFAULT_PATCH.relative_to(ROOT)})",
    )
    ap.add_argument(
        "--backup-out",
        type=Path,
        default=DEFAULT_BACKUP,
        help=f"Cache path for downloaded stock v1.03 .dat (default: {DEFAULT_BACKUP.relative_to(ROOT)})",
    )
    ap.add_argument(
        "--restore",
        action="store_true",
        help="Restore exact stock public v1.03 firmware instead of applying the patch",
    )
    ap.add_argument(
        "--pid",
        type=lambda s: int(s, 0),
        default=up.DEFAULT_DINPUT_PID,
        help=f"D-Input USB PID (allowed: 0x6011 or 0x6012; default: {up.DEFAULT_DINPUT_PID:#06x})",
    )
    ap.add_argument(
        "--name",
        default=up.DEFAULT_PRODUCT_NAME,
        help=f"20-char ASCII USB product string for D-Input mode (default: {up.DEFAULT_PRODUCT_NAME!r})",
    )
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Optionally write the patched .dat to disk",
    )
    ap.add_argument("--path", help="Select a specific hidraw path")
    ap.add_argument(
        "--execute",
        action="store_true",
        help="Actually erase, write, commit checksum, and reset the dongle (default: dry-run, no USB writes)",
    )
    ap.add_argument(
        "--no-reboot",
        action="store_true",
        help="Do not attempt to reboot an app-mode dongle into 2dc8:3208",
    )
    args = ap.parse_args()

    if args.fetch and args.base:
        ap.error("--fetch and --base are mutually exclusive")
    if args.dat and (args.restore or args.fetch or args.base):
        ap.error("--dat cannot be combined with --fetch, --base, or --restore")

    raw, mode_desc = resolve_target_dat(args)
    files, meta = up.validate_container_layout(raw)
    sha = hashlib.sha256(raw).hexdigest()
    print(f"Mode: {mode_desc}")
    print(f"Validated image ({len(raw)} bytes, SHA-256 {sha})")
    print(
        f"  Type 98 v{meta['version']}, chip_key={meta['chip_key']:04x}, "
        f"app_base={meta['app_base']:#x}, app.bin={len(files['app.bin'])} B"
    )

    if not args.execute:
        print("\nDRY-RUN mode (no USB writes). Pass --execute to flash.")
        if hid is not None:
            boots = find_boot_devices(args.path)
            apps = find_app_candidates(args.path)
            print(f"  Visible 2dc8:3208 boot devices: {len(boots)}")
            print(f"  Visible app-mode receiver interfaces: {len(apps)}")
        else:
            print("  (python hidapi not installed; install with `pip install hidapi` to flash)")
        return 0

    dongle = acquire_boot_dongle(allow_reboot=not args.no_reboot, path=args.path)
    boot_path = dongle.info["path"]
    try:
        print(f"Connected to 2dc8:3208 (verified logical PID {LOGICAL_PID:#06x})")

        # 1. CMD_INIT (0x00c1): returns 4-byte skip offset (0x4000) so uboot.boot stays untouched
        init_reply = dongle.command(CMD_INIT, min_reply_len=4)
        skip_off = int.from_bytes(init_reply[:4], "little")
        if skip_off != meta["app_base"]:
            raise RuntimeError(
                f"Unexpected boot init offset {skip_off:#x} (expected {meta['app_base']:#x})"
            )
        payload = raw[28 + skip_off :]
        total_len = len(payload)
        print(f"Boot init OK (skip={skip_off:#x}; flashing {total_len} bytes of app area)")

        # 2. CMD_ERASE (0x0004) — single attempt (non-idempotent destructive command)
        print(f"Erasing {total_len:#x} bytes...")
        dongle.command(CMD_ERASE, addr=0, length=0, total_len=total_len, timeout_ms=10000, retries=1)

        # 3. CMD_WRITE (0x0003) in 46-byte chunks
        off = 0
        last_pct = -1
        while off < total_len:
            chunk = payload[off : off + WRITE_CHUNK]
            dongle.command(
                CMD_WRITE,
                addr=off,
                length=len(chunk),
                total_len=total_len,
                payload=chunk,
                retries=1,
            )
            off += len(chunk)
            pct = (off * 100) // total_len
            if pct != last_pct and (pct % 5 == 0 or off == total_len):
                print(f"\rWriting... {pct:3d}% ({off}/{total_len})", end="", flush=True)
                last_pct = pct
        print()

        # 4. CMD_FINISH (0x00c2)
        dongle.command(CMD_FINISH, retries=1)

        # 5. CMD_CRC_COMMIT (0x00c3): commits 32-bit word-sum checksum + checks first 4 KiB CRC
        checksum = sum_u32_le(payload)
        crc_payload = struct.pack("<5I", 0, 4096, 0, checksum, total_len)
        crc_reply = dongle.command(
            CMD_CRC_COMMIT,
            addr=0,
            length=len(crc_payload),
            total_len=len(crc_payload),
            payload=crc_payload,
            min_reply_len=2,
            retries=1,
        )
        dev_crc = int.from_bytes(crc_reply[:2], "little")
        exp_crc = crc16_modbus(payload[:4096])
        print(
            f"Checksum committed (sum={checksum:#010x}, len={total_len:#x}); "
            f"first-4K CRC dev={dev_crc:#06x} vs host={exp_crc:#06x}"
        )
        if dev_crc != exp_crc:
            raise RuntimeError(f"CRC mismatch: device {dev_crc:#06x} != host {exp_crc:#06x}")

        # 6. CMD_RESET (0x0007): reboot dongle back to app mode and verify boot device disconnects
        print("Resetting dongle back into application mode...")
        try:
            dongle.command(CMD_RESET, timeout_ms=1000, expect_reply=True)
        except Exception:
            pass
    finally:
        dongle.close()

    # Wait up to 5 s for boot device to vanish and an app-mode interface to appear.
    left_boot = False
    for _ in range(50):
        time.sleep(0.1)
        if not any(d["path"] == boot_path for d in find_boot_devices()):
            left_boot = True
            if find_app_candidates():
                print("Done! Receiver rebooted into application mode.")
                return 0
    if left_boot:
        print("Done! Receiver left boot mode (app interface may still be settling).")
        return 0
    print(
        "Flash and CRC verification succeeded, but the receiver did not auto-reset; "
        "unplug and replug it to boot the new firmware."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
