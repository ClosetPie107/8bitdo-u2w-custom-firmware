#!/usr/bin/env python3
"""Repack 8BitDo Ultimate 2 Bluetooth 2.4G receiver (type 98) firmware with a modified app.bin.

Preserves the 28-byte 8BitDo header, top-level JLFS header, uboot.boot,
isd_config.ini, cfg_tool.bin, and all file offsets. Recomputes the app.bin
and app_area_head JLFS CRC16s, re-applies JieLi SFC scrambling with the
recovered chip key, and self-verifies the output with u2bt_extract.extract().

Usage:
    # Verify byte-for-byte round-trip against stock v1.03:
    python3 tools/u2bt_repack.py backups/u2bt-dongle-stock-v1.03.dat --self-test

    # Repack with a modified app.bin:
    python3 tools/u2bt_repack.py backups/u2bt-dongle-stock-v1.03.dat \
        --app /tmp/app_patched.bin -o /tmp/u2bt-patched.dat
"""

import argparse
import hashlib
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import u2bt_extract as ex


def patch_jlfs_record(record: bytes, new_data_crc: int) -> bytes:
    """Update data_crc and recompute hcrc in a 32-byte JLFS record."""
    if len(record) != 32:
        raise ValueError(f"Expected 32-byte JLFS record, got {len(record)}")
    body = bytearray(record[2:])
    struct.pack_into("<H", body, 0, new_data_crc)
    hcrc = ex.crc16(body)
    return struct.pack("<H", hcrc) + bytes(body)


def repack(base_raw: bytes, new_app: bytes) -> bytes:
    """Replace app.bin inside a validated type 98 .dat image and re-verify."""
    files, meta = ex.extract(base_raw)
    old_app = files["app.bin"]
    if len(new_app) != len(old_app):
        raise ValueError(
            f"app.bin size mismatch: got {len(new_app)}, expected {len(old_app)} "
            "(in-place repack requires identical size)"
        )

    key = meta["chip_key"]
    app_base = meta["app_base"]

    payload = bytearray(base_raw[28:])
    area_header_raw = ex.sfc(bytes(payload[app_base : app_base + 32]), key)
    area_header = ex.entry(area_header_raw)
    area_size = area_header.size

    area = bytearray(ex.sfc(bytes(payload[app_base : app_base + area_size]), key))

    # Locate app.bin entry inside area directory (starts at offset 32)
    off = 32
    app_rec_off = None
    app_entry = None
    while True:
        item = ex.entry(bytes(area[off : off + 32]))
        if item.name == "app.bin":
            app_rec_off = off
            app_entry = item
            break
        if item.index:
            break
        off += 32

    if app_rec_off is None or app_entry is None:
        raise ValueError("app.bin record not found in application area")

    # Replace app.bin bytes and update its JLFS record CRC
    area[app_entry.offset : app_entry.offset + app_entry.size] = new_app
    new_app_crc = ex.crc16(new_app)
    area[app_rec_off : app_rec_off + 32] = patch_jlfs_record(
        bytes(area[app_rec_off : app_rec_off + 32]), new_app_crc
    )

    # Recompute app_area_head CRC over area[32:]
    new_area_crc = ex.crc16(bytes(area[32:]))
    area[0:32] = patch_jlfs_record(bytes(area[0:32]), new_area_crc)

    # Re-scramble application area with JieLi SFC
    payload[app_base : app_base + area_size] = ex.sfc(bytes(area), key)

    out_raw = base_raw[:28] + bytes(payload)

    # Self-verify full extraction and CRCs
    out_files, _ = ex.extract(out_raw)
    if out_files["app.bin"] != new_app:
        raise RuntimeError("Self-verification failed: extracted app.bin differs")
    return out_raw


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input", type=Path, help="Base type 98 .dat image (stock public v1.02/v1.03)")
    parser.add_argument("--self-test", action="store_true", help="Verify byte-for-byte round-trip")
    parser.add_argument("--app", type=Path, help="Replacement app.bin file")
    parser.add_argument(
        "--replace-utf16",
        nargs=2,
        metavar=("OLD", "NEW"),
        help="Replace an equal-length UTF-16LE string in app.bin",
    )
    parser.add_argument("-o", "--output", type=Path, help="Output .dat path")
    args = parser.parse_args()

    base_raw = args.input.read_bytes()
    files, _ = ex.extract(base_raw)

    if args.self_test:
        rt = repack(base_raw, files["app.bin"])
        if rt != base_raw:
            parser.exit(1, "FAIL: round-trip output differs from stock input!\n")
        sha = hashlib.sha256(rt).hexdigest()
        print(f"PASS: byte-for-byte identical round-trip ({len(rt)} bytes, SHA-256 {sha})")
        return

    if bool(args.app) == bool(args.replace_utf16):
        parser.exit(2, "error: specify exactly one of --app or --replace-utf16 (or --self-test)\n")
    if not args.output:
        parser.exit(2, "error: -o/--output is required\n")

    if args.app:
        new_app = args.app.read_bytes()
    else:
        old_s, new_s = args.replace_utf16
        old_b = old_s.encode("utf-16le")
        new_b = new_s.encode("utf-16le")
        if len(old_b) != len(new_b):
            parser.exit(2, f"error: UTF-16LE length mismatch ({len(old_b)} vs {len(new_b)} bytes)\n")
        app_buf = bytearray(files["app.bin"])
        count = app_buf.count(old_b)
        if count == 0:
            parser.exit(1, f"error: UTF-16LE string {old_s!r} not found in app.bin\n")
        idx = app_buf.find(old_b)
        app_buf[idx : idx + len(old_b)] = new_b
        new_app = bytes(app_buf)
        print(f"Replaced {old_s!r} -> {new_s!r} at app+{idx:#x} (count={count})")

    out_raw = repack(base_raw, new_app)
    args.output.write_bytes(out_raw)
    sha = hashlib.sha256(out_raw).hexdigest()
    print(f"Wrote {args.output} ({len(out_raw)} bytes, SHA-256 {sha})")
    print("Verified: all JLFS/BANKCB/app CRCs pass u2bt_extract.extract().")


if __name__ == "__main__":
    main()
