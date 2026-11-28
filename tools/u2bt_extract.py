#!/usr/bin/env python3
"""Validate and extract 8BitDo Ultimate 2 Bluetooth 2.4G receiver (type 98) firmware.

Offline only: never opens a USB device or produces a flashable update.
JieLi JLFS/BANKCB and ENC/SFC formats:
  https://kagaimiq.github.io/jielie/datafmt/newfw.html
  https://github.com/kagaimiq/jl-misctools

Usage:
    python3 tools/u2bt_extract.py backups/u2bt-dongle-stock-v1.03.dat -o /tmp/u2bt-public
"""

import argparse
import binascii
from dataclasses import dataclass
from pathlib import Path
import struct


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0)


def check_crc(data: bytes, expected: int, label: str) -> None:
    actual = crc16(data)
    if actual != expected:
        raise ValueError(f"{label}: CRC {actual:04x}, expected {expected:04x}")


def enc(data: bytes, key: int = 0xFFFF) -> bytes:
    """Symmetric JieLi ENC XOR; reset the seed for each logical record."""
    result = bytearray(data)
    for i in range(len(result)):
        result[i] ^= key & 0xFF
        key = ((key << 1) ^ (0x1021 if key & 0x8000 else 0)) & 0xFFFF
    return bytes(result)


def sfc(data: bytes, key: int) -> bytes:
    """SFC block addresses are relative to the application-area base."""
    result = bytearray()
    for off in range(0, len(data), 32):
        result.extend(enc(data[off : off + 32], key ^ (off >> 2)))
    return bytes(result)


def region(data: bytes, offset: int, size: int) -> bytes:
    if offset < 0 or size < 0 or offset + size > len(data):
        raise ValueError(f"Region {offset:#x}+{size:#x} exceeds {len(data):#x} bytes")
    return data[offset : offset + size]


@dataclass(frozen=True)
class Entry:
    data_crc: int
    offset: int
    size: int
    attr: int
    index: int
    name: str


def entry(record: bytes) -> Entry:
    if len(record) != 32:
        raise ValueError("Truncated JLFS record")
    hcrc, dcrc, off, size, attr, reserved, index, name = struct.unpack("<HHIIBBH16s", record)
    check_crc(record[2:], hcrc, "JLFS header")
    return Entry(dcrc, off, size, attr, index, name.split(b"\0", 1)[0].decode("ascii"))


def chip_key(config: bytes) -> int:
    blob = region(config, 0, 32)
    check_crc(blob, int.from_bytes(region(config, 32, 2), "little"), "Chip-key blob")
    threshold = sum(blob[:16]) & 0xFF
    if threshold >= 0xE0:
        threshold = 0xAA
    elif threshold <= 0x10:
        threshold = 0x55
    return sum(1 << i for i in range(16) if (blob[16 + i] ^ blob[15 - i]) < threshold)


def bootloader(data: bytes) -> tuple[bytes, dict[str, tuple[bytes, int]]]:
    """Decode each BANKCB header and bank separately, verifying both CRCs."""
    result = bytearray(data)
    banks: dict[str, tuple[bytes, int]] = {}
    count = 1
    index = 0
    while index < count:
        off = index * 16
        header = enc(region(data, off, 16))
        number, size, load, bankoff, dcrc, hcrc = struct.unpack("<HHIIHH", header)
        check_crc(header[:14], hcrc, f"Bank {index} header")
        if index == 0:
            count = number
            if not 1 <= count <= len(data) // 16:
                raise ValueError("Invalid bank count")
        code = enc(region(data, bankoff, size))
        check_crc(code, dcrc, f"Bank {index} data")
        result[off : off + 16] = header
        result[bankoff : bankoff + size] = code
        banks[f"uboot-bank{index}.bin"] = (code, load)
        index += 1
    return bytes(result), banks


def extract(raw: bytes) -> tuple[dict[str, bytes], dict]:
    """Return verified files and metadata without modifying the input."""
    if len(raw) < 28:
        raise ValueError("Truncated 8BitDo header")
    version, address, size, pid = struct.unpack_from("<IIII", raw)
    if pid != 0x6011 or address != 0x3000 or size != 0x40000 or len(raw) != size + 28:
        raise ValueError("Expected a type 98 receiver .dat (PID 6011, address 3000, 256 KiB payload)")
    payload = raw[28:]
    header = enc(payload[:32])
    check_crc(header[2:], int.from_bytes(header[:2], "little"), "Flash header")
    flash_size = struct.unpack_from("<I", header, 8)[0]
    entries: dict[str, Entry] = {}
    off = 32
    while True:
        item = entry(enc(region(payload, off, 32)))
        if item.name in entries:
            raise ValueError(f"Duplicate top-level file: {item.name}")
        entries[item.name] = item
        off += 32
        if item.index:
            break

    def get_file(name: str) -> bytes:
        item = entries[name]
        return region(payload, item.offset, item.size)

    config = get_file("isd_config.ini")
    check_crc(config, entries["isd_config.ini"].data_crc, "isd_config.ini")
    key = chip_key(config)
    boot, banks = bootloader(get_file("uboot.boot"))
    check_crc(boot, entries["uboot.boot"].data_crc, "uboot.boot")
    base = entries["app_dir_head"].offset
    area_header = entry(sfc(region(payload, base, 32), key))
    if area_header.name != "app_area_head" or area_header.attr != 0x83 or area_header.size < 32:
        raise ValueError("Unsupported application-area layout")
    area = sfc(region(payload, base, area_header.size), key)
    check_crc(area[32:], area_header.data_crc, "Application area")
    files: dict[str, bytes] = {"uboot.boot": boot, "isd_config.ini": config}
    for name, (code, load) in banks.items():
        files[name] = code
    app_entries = []
    off = 32
    while True:
        item = entry(region(area, off, 32))
        app_entries.append(item)
        if not item.attr & 0x10:
            if item.name not in ("app.bin", "cfg_tool.bin") or item.attr != 0x82:
                raise ValueError(f"Unsupported application file: {item.name}")
            content = region(area, item.offset, item.size)
            check_crc(content, item.data_crc, item.name)
            files[item.name] = content
        off += 32
        if item.index:
            break
    if "app.bin" not in files:
        raise ValueError("Missing app.bin")
    metadata = dict(
        version=version,
        chip_key=key,
        flash_size=flash_size,
        app_base=base,
        entry_point=area_header.offset,
        top_entries=list(entries.values()),
        app_entries=app_entries,
        bank_loads={name: load for name, (code, load) in banks.items()},
    )
    return files, metadata


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path)
    parser.add_argument("-o", "--output", required=True, type=Path, help="New output directory; must not exist")
    args = parser.parse_args()
    try:
        files, meta = extract(args.input.read_bytes())
        args.output.mkdir(parents=True, exist_ok=False)
        for name, data in files.items():
            (args.output / name).write_bytes(data)
        print(f"Validated type 98 version {meta['version']}; chip key {meta['chip_key']:04x}")
        print(f"Flash-size field {meta['flash_size']:#x}; app base {meta['app_base']:#x}; entry {meta['entry_point']:#x}")
        for name, data in files.items():
            load = meta["bank_loads"].get(name)
            suffix = f", load {load:#x}" if load is not None else ""
            print(f"  {name}: {len(data)} bytes{suffix}")
        print("All parsed header and extracted-file CRCs match. Not a flashing/repacking tool.")
    except (OSError, ValueError, KeyError, struct.error) as error:
        parser.exit(1, f"error: {error}\n")


if __name__ == "__main__":
    main()
