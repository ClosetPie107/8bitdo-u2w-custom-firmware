#!/usr/bin/env python3
"""Patch and flash an 8BitDo Ultimate 2 Wireless (v1.10, PID 0x6012) in USB boot mode.

Default workflow (no toolchain or external .dat needed if your controller is on stock v1.10):
  1. Dry-run (read-only):   python3 tools/flash.py
  2. Flash + auto-backup:   python3 tools/flash.py --execute

Re-flashing a controller that already has this patch (or restoring stock v1.10):
  - Upgrade using your saved backup:
      python3 tools/flash.py --base backups/u2w-stock-v1.10.image.bin --execute
  - Restore exact stock v1.10:
      python3 tools/flash.py --restore backups/u2w-stock-v1.10.image.bin --execute
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from u2w_boot import (
    APP_END,
    APP_START,
    BOOTLOADER_SHA256,
    ERASE_END,
    ERASED_SECTOR,
    IMAGE_SIZE,
    LOGICAL_PID,
    METADATA_LEN,
    METADATA_START,
    SECTOR_SIZE,
    STOCK_IMAGE_SHA256,
    WRITE_CHUNK,
    BootPad,
    apply_patch_file,
    atomic_write_bytes,
    expected_sector_bytes,
    find_boot_device,
    load_stock_image,
    sha256,
    validate_and_fix_image,
    validate_update_metadata,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATCH = ROOT / "dist" / "u2w-v1.10-switch2.patch.json"
DEFAULT_BACKUP = ROOT / "backups" / "u2w-stock-v1.10.image.bin"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--patch",
        type=Path,
        default=DEFAULT_PATCH,
        help=f"path to u2w-telink-patch-v1 JSON file (default: {DEFAULT_PATCH.relative_to(ROOT)})",
    )
    ap.add_argument(
        "--base",
        type=Path,
        help="path to stock U2W v1.10 .dat or raw 0x23800 app backup (optional if controller is currently on stock v1.10)",
    )
    ap.add_argument(
        "--image",
        type=Path,
        help="flash a pre-built 0x23800 image directly (requires --base for invariant verification)",
    )
    ap.add_argument(
        "--restore",
        type=Path,
        help="restore exact stock v1.10 from a saved backup .bin or stock .dat",
    )
    ap.add_argument(
        "--backup-out",
        type=Path,
        default=DEFAULT_BACKUP,
        help=f"where to save stock v1.10 app backup before flashing (default: {DEFAULT_BACKUP.relative_to(ROOT)})",
    )
    ap.add_argument("--path", help="select a specific hidraw path")
    ap.add_argument(
        "--recover-unknown",
        action="store_true",
        help="allow --base/--restore even if the currently installed app image is unrecognized/corrupt",
    )
    ap.add_argument(
        "--execute",
        action="store_true",
        help="actually send erase/write commands (default is read-only dry-run)",
    )
    args = ap.parse_args()

    if args.restore and args.image:
        sys.exit("cannot combine --restore and --image")
    if args.image and not args.base:
        sys.exit("--image requires --base (stock v1.10 .dat or backup .bin) for safety checks")
    if args.recover_unknown and not (args.base or args.restore):
        sys.exit("--recover-unknown requires --base or --restore")

    stock: bytes | None = None
    if args.restore:
        stock = load_stock_image(args.restore)
    elif args.base:
        stock = load_stock_image(args.base)

    dev_info = find_boot_device(args.path)
    print(f"opening 2dc8:3208 at {dev_info['path']}", file=sys.stderr)
    pad = BootPad(dev_info)
    try:
        fw_pid = pad.identify()
        print(f"identify: logical PID {fw_pid:#06x}")
        if fw_pid != LOGICAL_PID:
            sys.exit(
                f"ABORT: logical PID {fw_pid:#06x} != {LOGICAL_PID:#06x} "
                "(this firmware is ONLY for 8BitDo Ultimate 2 Wireless)"
            )

        boot_region = pad.read_range(0, APP_START, label="preflight bootloader")
        boot_sha = sha256(boot_region)
        if boot_sha != BOOTLOADER_SHA256:
            sys.exit(
                f"ABORT: bootloader [0x0000, 0x5000) SHA-256 {boot_sha} "
                f"!= verified U2W v1.10 bootloader {BOOTLOADER_SHA256}"
            )

        meta_region = pad.read_range(
            METADATA_START, METADATA_LEN, label="preflight metadata"
        )
        try:
            validate_update_metadata(meta_region)
        except ValueError as exc:
            sys.exit(f"ABORT: {exc}")

        app_and_tail = pad.read_range(
            APP_START, ERASE_END - APP_START, label="preflight app+tail"
        )
        cur_app = app_and_tail[:IMAGE_SIZE]
        tail = app_and_tail[IMAGE_SIZE:]
        if tail != ERASED_SECTOR[: len(tail)]:
            sys.exit(f"ABORT: non-erased bytes in tail [{APP_END:#07x}, {ERASE_END:#07x})")

        cur_sha = sha256(cur_app)
        print(f"device app SHA-256: {cur_sha}")

        if stock is None:
            if cur_sha != STOCK_IMAGE_SHA256:
                sys.exit(
                    "ABORT: controller is not running stock U2W v1.10 "
                    f"(expected {STOCK_IMAGE_SHA256}).\n"
                    "  - If on an older/newer stock version, update to stock v1.10 first.\n"
                    "  - If already running a custom build, pass --base <backup-or-v1.10.dat>."
                )
            stock = cur_app
            if args.backup_out.exists():
                try:
                    load_stock_image(args.backup_out)
                except ValueError as exc:
                    sys.exit(
                        f"ABORT: existing backup {args.backup_out} failed verification ({exc}).\n"
                        "  Remove or move the corrupt file, or pass a different --backup-out path."
                    )
                print(f"verified existing stock v1.10 backup at {args.backup_out}")
            else:
                atomic_write_bytes(args.backup_out, stock, mode=0o600)
                print(f"saved stock v1.10 backup to {args.backup_out}")
        elif cur_sha != STOCK_IMAGE_SHA256:
            try:
                validate_and_fix_image(bytearray(cur_app), stock, fix_sums=False)
            except ValueError as exc:
                if not args.recover_unknown:
                    sys.exit(
                        f"ABORT: installed app image is not stock v1.10 or a valid v1.10 Telink patch ({exc}).\n"
                        "  - If on a different stock version, update to stock v1.10 first.\n"
                        "  - If recovering from an interrupted flash, re-run with --recover-unknown."
                    )
                print(f"WARNING: --recover-unknown set; overwriting unrecognized app ({exc})")

        if args.restore:
            target = stock
            print("mode: RESTORE exact stock U2W v1.10")
        elif args.image:
            target = validate_and_fix_image(
                bytearray(args.image.read_bytes()), stock, fix_sums=False
            )
            print(f"mode: FLASH custom image {args.image}")
        else:
            target = apply_patch_file(stock, args.patch)
            print(f"mode: PATCH & FLASH using {args.patch}")

        print(f"target app SHA-256: {sha256(target)}")

        # Plan changed sectors, leaving header sector 0x5000 until last.
        order = [*range(APP_START + SECTOR_SIZE, ERASE_END, SECTOR_SIZE), APP_START]
        planned: list[int] = []
        for addr in order:
            off = addr - APP_START
            cur = app_and_tail[off : off + SECTOR_SIZE]
            exp = expected_sector_bytes(target, addr)
            if cur != exp:
                planned.append(addr)

        print(f"pre-flight checks passed; sectors to update ({len(planned)}): {[hex(a) for a in planned]}")
        if not planned:
            print("Device already matches target image. Nothing to do.")
            return 0

        if not args.execute:
            print("DRY RUN ONLY — pass --execute to erase and program these sectors.")
            return 0

        t0 = time.monotonic()
        for idx, addr in enumerate(planned, 1):
            exp = expected_sector_bytes(target, addr)
            prog_len = min(SECTOR_SIZE, APP_END - addr)
            print(
                f"[{idx:2d}/{len(planned)}] sector {addr:#07x}: erasing...",
                end="",
                flush=True,
            )
            pad.erase_sector(addr)

            print(" programming...", end="", flush=True)
            off = addr - APP_START
            for chunk_off in range(0, prog_len, WRITE_CHUNK):
                chunk = target[off + chunk_off : off + min(chunk_off + WRITE_CHUNK, prog_len)]
                pad.write_chunk(addr + chunk_off, chunk)

            print(" verifying...", end="", flush=True)
            got = pad.read_range(addr, SECTOR_SIZE)
            if got != exp:
                print(" MISMATCH!")
                sys.exit(f"ABORT: readback mismatch on sector {addr:#07x}")
            print(" OK")

        post_app_tail = pad.read_range(
            APP_START, ERASE_END - APP_START, label="post-verify app+tail"
        )
        if post_app_tail[:IMAGE_SIZE] != target:
            sys.exit("ABORT: final app readback does not match target image!")
        if post_app_tail[IMAGE_SIZE:] != ERASED_SECTOR[: ERASE_END - APP_END]:
            sys.exit("ABORT: final tail readback is not all 0xff!")

        elapsed = time.monotonic() - t0
        print(f"\nSUCCESS: {len(planned)} sector(s) flashed and verified in {elapsed:.1f}s.")
        print(f"Verified app SHA-256: {sha256(post_app_tail[:IMAGE_SIZE])}")
        print("Unplug USB and power-cycle the controller normally.")
        return 0
    finally:
        pad.close()


if __name__ == "__main__":
    raise SystemExit(main())
