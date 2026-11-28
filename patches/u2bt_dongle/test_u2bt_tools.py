#!/usr/bin/env python3
"""Host unit tests for U2BT 2.4G receiver extraction, repacking, and JSON patch integrity."""

from __future__ import annotations

import json
import struct
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import u2bt_extract as ex  # noqa: E402
import u2bt_patch as up  # noqa: E402
import u2bt_repack as rp  # noqa: E402


def make_jlfs_record(name: str, *, offset: int, size: int, attr: int, index: int, data_crc: int) -> bytes:
    body = struct.pack(
        "<HIIBBH16s",
        data_crc,
        offset,
        size,
        attr,
        0,
        index,
        name.encode("ascii").ljust(16, b"\x00"),
    )
    return struct.pack("<H", ex.crc16(body)) + body


def build_synthetic_dat(app_data: bytes) -> bytes:
    """Build a minimal valid 256 KiB + 28 B type-98 JieLi .dat image for offline testing."""
    payload = bytearray(0x40000)

    # 1. isd_config.ini at 0x1000 (34 bytes: 32-byte key blob + 2-byte CRC)
    cfg_blob = bytes(range(32))
    cfg = cfg_blob + struct.pack("<H", ex.crc16(cfg_blob))
    cfg_off = 0x1000
    payload[cfg_off : cfg_off + len(cfg)] = cfg
    key = ex.chip_key(cfg)

    # 2. uboot.boot at 0x2000 (1 bank: 16-byte BANKCB header + 16-byte bank data)
    bank_code = b"\x11\x22\x33\x44" * 4
    bank_hdr_plain = struct.pack("<HHIIHH", 1, len(bank_code), 0x1000, 16, ex.crc16(bank_code), 0)
    bank_hdr_plain = bank_hdr_plain[:14] + struct.pack("<H", ex.crc16(bank_hdr_plain[:14]))
    uboot_plain = bank_hdr_plain + bank_code
    uboot_enc = ex.enc(bank_hdr_plain) + ex.enc(bank_code)
    uboot_off = 0x2000
    payload[uboot_off : uboot_off + len(uboot_enc)] = uboot_enc

    # 3. app_area at 0x4000 (32-byte app_area_head + 32-byte app.bin rec + app.bin)
    app_base = 0x4000
    app_rec = make_jlfs_record(
        "app.bin", offset=64, size=len(app_data), attr=0x82, index=1, data_crc=ex.crc16(app_data)
    )
    area_body = app_rec + app_data
    area_head = make_jlfs_record(
        "app_area_head",
        offset=0x1E000C0,
        size=32 + len(area_body),
        attr=0x83,
        index=0,
        data_crc=ex.crc16(area_body),
    )
    area_plain = area_head + area_body
    payload[app_base : app_base + len(area_plain)] = ex.sfc(area_plain, key)

    # 4. Top-level flash header + directory
    flash_hdr_body = struct.pack("<HHI22s", 0, 0, 0x40000, b"\x00" * 22)
    flash_hdr = struct.pack("<H", ex.crc16(flash_hdr_body)) + flash_hdr_body
    payload[0:32] = ex.enc(flash_hdr)

    top_recs = [
        make_jlfs_record("isd_config.ini", offset=cfg_off, size=len(cfg), attr=0x82, index=0, data_crc=ex.crc16(cfg)),
        make_jlfs_record("uboot.boot", offset=uboot_off, size=len(uboot_enc), attr=0x82, index=0, data_crc=ex.crc16(uboot_plain)),
        make_jlfs_record("app_dir_head", offset=app_base, size=len(area_plain), attr=0x83, index=1, data_crc=0),
    ]
    for i, rec in enumerate(top_recs):
        payload[32 + i * 32 : 64 + i * 32] = ex.enc(rec)

    hdr_28 = struct.pack("<7I", 103, 0x3000, 0x40000, 0x6011, 0, 0, 0)
    return hdr_28 + bytes(payload)


def test_extract_and_repack_roundtrip() -> None:
    orig_app = "8BitDo UM 2 Receiver".encode("utf-16le") + b"\x00" * 256
    dat = build_synthetic_dat(orig_app)

    files, meta = ex.extract(dat)
    assert files["app.bin"] == orig_app
    assert meta["app_base"] == 0x4000

    # Synthetic container has wrong chip_key & app.bin size -> validate_container_layout must reject it
    try:
        up.validate_container_layout(dat)
        raise AssertionError("expected validate_container_layout to reject synthetic chip_key")
    except ValueError:
        pass

    # Identity repack must be byte-for-byte identical
    rt = rp.repack(dat, orig_app)
    assert rt == dat

    # Modified repack updates CRCs and passes extraction
    mod_app = "8BitDo UM 2 GyroD   ".encode("utf-16le") + b"\xaa" * 256
    out = rp.repack(dat, mod_app)
    assert out != dat
    out_files, _ = ex.extract(out)
    assert out_files["app.bin"] == mod_app

    # Corrupted byte must fail CRC verification
    bad = bytearray(out)
    bad[28 + 0x4000 + 80] ^= 0xFF
    try:
        ex.extract(bytes(bad))
        raise AssertionError("expected CRC failure on corrupted .dat")
    except ValueError:
        pass


def test_dist_patch_manifest_and_guards() -> None:
    patch_path = ROOT / "dist" / "u2bt-dongle-v1.03-dinput-gyro.patch.json"
    doc = json.loads(patch_path.read_text())
    assert doc["format"] == "u2bt-br22-patch-v1"
    assert doc["base_app_sha256"] == up.STOCK_APP_SHA256
    assert doc["base_dat_sha256"] == up.STOCK_DAT_SHA256
    assert len(doc["patched_app_sha256"]) == 64
    assert len(doc["patched_dat_sha256"]) == 64

    sec_names = [s["name"] for s in doc["sections"]]
    assert set(sec_names) == set(up.PATCH_SECTION_MAP)
    for sec in doc["sections"]:
        exp_addr, min_len, max_len, _ = up.PATCH_SECTION_MAP[sec["name"]]
        assert sec["flash_addr"] == exp_addr
        blob = bytes.fromhex(sec["data_hex"])
        assert min_len <= len(blob) <= max_len

    # Invalid PID / non-ASCII name must be rejected
    dummy_app = bytearray(up.EXPECTED_APP_SIZE)
    struct.pack_into("<H", dummy_app, up.app_off(up.ADDR_DINPUT_PID), up.STOCK_DINPUT_PID)
    dummy_app[0:40] = up.STOCK_PRODUCT_NAME.encode("utf-16le")
    for bad_pid in (0x1234, 0x0000):
        try:
            up.patch_pid_and_name(bytearray(dummy_app), pid=bad_pid, product_name=up.DEFAULT_PRODUCT_NAME)
            raise AssertionError(f"expected rejection for PID {bad_pid:#x}")
        except ValueError:
            pass
    for bad_name in ("8BitDo UM 2 GyroD", "8BitDo UM 2 GyroD  é", "8BitDo Wireless Pad "):
        try:
            up.patch_pid_and_name(bytearray(dummy_app), pid=up.DEFAULT_DINPUT_PID, product_name=bad_name)
            raise AssertionError(f"expected rejection for name {bad_name!r}")
        except ValueError:
            pass

    # If cached stock v1.03 exists locally, verify applying manifest + rejecting truncated hook with custom PID
    cached_stock = ROOT / "backups" / "u2bt-dongle-stock-v1.03.dat"
    if cached_stock.is_file():
        stock_dat = up.load_stock_dat(cached_stock, require_exact_v103=True)
        out_default = up.apply_patch_file(stock_dat, patch_path)
        assert up.sha256(out_default) == doc["patched_dat_sha256"]

        out_6011 = up.apply_patch_file(stock_dat, patch_path, pid=0x6011, product_name="8BitDo UM 2 Receiver")
        assert out_6011 != out_default

        # Truncated hook must be rejected even when passing custom --pid / --name
        bad_doc = json.loads(patch_path.read_text())
        for sec in bad_doc["sections"]:
            if sec["name"] == ".hook_pack":
                sec["data_hex"] = sec["data_hex"][:2]  # 1 byte instead of 4
        with tempfile.TemporaryDirectory() as td:
            bad_path = Path(td) / "bad.patch.json"
            bad_path.write_text(json.dumps(bad_doc))
            try:
                up.apply_patch_file(stock_dat, bad_path, pid=0x6011, product_name="8BitDo UM 2 Receiver")
                raise AssertionError("expected truncated hook to be rejected")
            except ValueError:
                pass


def main() -> int:
    test_extract_and_repack_roundtrip()
    test_dist_patch_manifest_and_guards()
    print("PASS: all U2BT Python tool & patch-manifest tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
