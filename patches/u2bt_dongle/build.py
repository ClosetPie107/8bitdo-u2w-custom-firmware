#!/usr/bin/env python3
"""Build U2BT 2.4G receiver (type 98, public v1.02/v1.03) custom firmware from pi32v2 C/ASM sources."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PATCH_DIR = Path(__file__).resolve().parent
BUILD = PATCH_DIR / "build"

sys.path.insert(0, str(ROOT / "tools"))
import u2bt_extract as ex  # noqa: E402
import u2bt_patch as up  # noqa: E402
import u2bt_repack as rp  # noqa: E402

ELF_SECTIONS = [
    ".hook_usb_tx",
    ".hook_pack",
    ".hook_send",
    ".hook_hk_a1",
    ".hook_hk_a2",
    ".cave_dinput",
    ".cave_tx_fix",
]


def find_pi32v2_bin() -> Path:
    env = os.environ.get("PI32V2_BIN")
    if env:
        p = Path(env)
        if (p / "clang").is_file() and (p / "ld").is_file():
            return p
    which = shutil.which("pi32v2-clang") or shutil.which("clang")
    if which:
        p = Path(which).resolve().parent
        if (p / "ld").is_file() and "pi32" in str(p):
            return p
    opt_root = Path.home() / ".local" / "opt" / "pi32v2"
    for candidate in sorted(opt_root.glob("**/pi32v2/bin"), reverse=True):
        if (candidate / "clang").is_file() and (candidate / "ld").is_file():
            return candidate
    sys.exit(
        "JieLi pi32v2 toolchain not found. Install it to ~/.local/opt/pi32v2 "
        "(https://pkgman.jieliapp.com/s/linux-toolchain) or set PI32V2_BIN=/path/to/pi32v2/bin."
    )


def run(cmd: list[str]) -> str:
    res = subprocess.run(cmd, text=True, capture_output=True)
    if res.returncode != 0:
        sys.stderr.write(res.stderr)
        res.check_returncode()
    return res.stdout


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--base",
        type=Path,
        help="path to stock U2BT type 98 public v1.02/v1.03 .dat (omit with --fetch)",
    )
    ap.add_argument(
        "--fetch",
        action="store_true",
        help="download stock public v1.03 directly from 8BitDo's official API",
    )
    ap.add_argument(
        "--pid",
        type=lambda s: int(s, 0),
        default=up.DEFAULT_DINPUT_PID,
        help=f"D-Input USB PID (default: {up.DEFAULT_DINPUT_PID:#06x})",
    )
    ap.add_argument(
        "--name",
        default=up.DEFAULT_PRODUCT_NAME,
        help=f"20-char USB product string (default: {up.DEFAULT_PRODUCT_NAME!r})",
    )
    ap.add_argument(
        "--export-patch",
        type=Path,
        help="write a redistribution-safe u2bt-br22-patch-v1 JSON file (original bytes only)",
    )
    ap.add_argument(
        "-o",
        "--output",
        type=Path,
        default=BUILD / "u2bt-public-103-gyro-6012.dat",
        help="output patched .dat path",
    )
    args = ap.parse_args()

    if args.fetch:
        cache = ROOT / "backups" / "u2bt-dongle-stock-v1.03.dat"
        stock_dat = up.fetch_public_103(cache_path=cache)
    elif args.base is not None:
        stock_dat = up.load_stock_dat(args.base)
    else:
        ap.error("Provide --base <stock.dat> or pass --fetch")

    tc_bin = find_pi32v2_bin()
    clang = str(tc_bin / "clang")
    ld = str(tc_bin / "ld")
    objcopy = str(tc_bin / "objcopy")
    objdump = str(tc_bin / "objdump")

    BUILD.mkdir(parents=True, exist_ok=True)
    objs: list[str] = []
    for src in ("dinput_gyro.c", "usb_tx_fix.c"):
        obj = BUILD / (Path(src).stem + ".o")
        run([
            clang, "-target", "pi32v2", "-Os", "-Wall", "-Wextra", "-Werror",
            "-ffreestanding", "-fno-builtin",
            "-c", str(PATCH_DIR / src), "-o", str(obj),
        ])
        objs.append(str(obj))

    hooks_o = BUILD / "hooks.o"
    run([clang, "-target", "pi32v2", "-c", str(PATCH_DIR / "hooks.s"), "-o", str(hooks_o)])
    objs.append(str(hooks_o))

    elf = BUILD / "patch.elf"
    run([ld, "-T", str(PATCH_DIR / "link.ld"), *objs, "-o", str(elf)])

    # Verify ALL allocated ELF sections (TEXT/DATA/BSS) match ELF_SECTIONS and PATCH_SECTION_MAP exactly
    sec_hdrs = run([objdump, "-h", str(elf)])
    alloc_addrs: dict[str, int] = {}
    for line in sec_hdrs.splitlines():
        m = re.match(r"^\s*\d+\s+(\.\S+)\s+([\da-f]+)\s+([\da-f]+)\s*(.*)$", line)
        if m:
            name, size, vma, sec_type = m.groups()
            if any(tok in sec_type.split() for tok in ("TEXT", "DATA", "BSS")) and int(size, 16) > 0:
                alloc_addrs[name] = int(vma, 16)
    expected_addrs = {name: up.PATCH_SECTION_MAP[name][0] for name in ELF_SECTIONS}
    if alloc_addrs != expected_addrs:
        raise RuntimeError(f"ELF allocated sections differ from layout: {alloc_addrs} != {expected_addrs}")

    (BUILD / "patch.dis").write_text(run([objdump, "-d", "--triple=pi32v2", str(elf)]))

    files, _ = up.validate_container_layout(stock_dat)
    stock_app = files["app.bin"]
    app = bytearray(stock_app)
    up.patch_pid_and_name(app, pid=args.pid, product_name=args.name)

    ram_init_off = up.app_off(up.ADDR_TX_FIX_RAM_INIT)
    if any(b != 0 for b in app[ram_init_off : ram_init_off + 4]):
        raise RuntimeError(f"EP_BUSY_MASK flash backing at {up.ADDR_TX_FIX_RAM_INIT:#x} is not zero")

    section_blobs: dict[str, bytes] = {
        ".rdesc_len": bytes([0x60, 0x28]),  # r0 = 0x88 (136 B)
        ".rdesc": up.RDESC_34B + bytes(145 - len(up.RDESC_34B)),
    }
    for sec in ELF_SECTIONS:
        bin_path = BUILD / f"{sec.lstrip('.')}.bin"
        run([objcopy, "-O", "binary", "-j", sec, str(elf), str(bin_path)])
        section_blobs[sec] = bin_path.read_bytes()

    patch_sections = []
    for name, flash_addr, min_len, max_len, stock_hex in up.PATCH_SECTIONS:
        blob = section_blobs[name]
        if not (min_len <= len(blob) <= max_len):
            raise RuntimeError(f"section {name} size {len(blob)} outside {min_len}..{max_len}")
        off = up.app_off(flash_addr)
        end = off + len(blob)
        if off < 0 or end > len(app):
            raise RuntimeError(f"section {name} out of bounds at {flash_addr:#x}")
        if name in up.CAVE_SECTIONS and any(b != 0 for b in app[off:end]):
            raise RuntimeError(f"cave {name} at {flash_addr:#x} is not all zeros in stock")
        if stock_hex is not None:
            exp_stock = bytes.fromhex(stock_hex)
            if bytes(app[off : off + len(exp_stock)]) != exp_stock:
                raise RuntimeError(f"stock bytes differ at {name} ({flash_addr:#x})")
        if name == ".rdesc" and bytes(app[off : off + 4]) != bytes([0x05, 0x01, 0x09, 0x05]):
            raise RuntimeError(f"unexpected stock rdesc at {flash_addr:#x}")
        app[off:end] = blob
        patch_sections.append({
            "name": name,
            "flash_addr": flash_addr,
            "data_hex": blob.hex(),
        })
        print(f"patched {name:14s} @ flash {flash_addr:#09x} ({len(blob):4d} bytes)")

    patched_app = bytes(app)
    out_dat = rp.repack(stock_dat, patched_app)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(out_dat)

    if args.export_patch:
        args.export_patch.parent.mkdir(parents=True, exist_ok=True)
        patch_doc = {
            "format": "u2bt-br22-patch-v1",
            "target_model": "8BitDo Ultimate 2 Bluetooth 2.4G Receiver (type 98, logical PID 0x6011)",
            "base_version": "1.03",
            "base_dat_sha256": up.STOCK_DAT_SHA256,
            "base_app_sha256": up.sha256(stock_app),
            "default_pid": args.pid,
            "default_product_name": args.name,
            "patched_app_sha256": up.sha256(patched_app),
            "patched_dat_sha256": up.sha256(out_dat),
            "sections": patch_sections,
        }
        args.export_patch.write_text(json.dumps(patch_doc, indent=2) + "\n")
        print(f"wrote patch {args.export_patch}")

    print(f"wrote {args.output} ({len(out_dat)} bytes, SHA-256 {up.sha256(out_dat)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
