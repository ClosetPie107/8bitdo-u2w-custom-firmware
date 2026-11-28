#!/usr/bin/env python3
"""Shared patch layout, verification, and 8BitDo API fetch helpers for U2BT 2.4G receiver (type 98).

Contains no proprietary 8BitDo firmware bytes — only pinned SHA-256 verification hashes,
patch section constraints, and redistribution-safe JSON patch application.
"""

from __future__ import annotations

import hashlib
import json
import os
import struct
import sys
import tempfile
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import u2bt_extract as ex
import u2bt_repack as rp

APP_BASE = 0x1E000C0
STOCK_APP_SHA256 = "217088cda39042866b468a2cec8d44d3e04e82aa073f0f0f142b40cd64face81"
STOCK_DAT_SHA256 = "4e48804cacfd2baa436346b31aed3f6063855bcfb232a6cca3429a29939996d1"

# Pinned container invariants for verified type 98 public v1.02/v1.03 hardware.
EXPECTED_CHIP_KEY = 0x2F1B
EXPECTED_APP_BASE = 0x4000
EXPECTED_ENTRY_POINT = 0x1E000C0
EXPECTED_APP_SIZE = 208804

ADDR_DINPUT_PID = 0x1E11327
ADDR_RDESC_LEN_INSN = 0x1E001DA
ADDR_DINPUT_RDESC = 0x1E11BCB
ADDR_CAVE_DINPUT = 0x1E32000
ADDR_CAVE_TX_FIX = 0x1E32300
ADDR_TX_FIX_RAM_INIT = 0x1E32FA0  # Boot-zeroed flash backing for EP_BUSY_MASK @ RAM 0x4c04

DEFAULT_DINPUT_PID = 0x6012
STOCK_DINPUT_PID = 0x6011
ALLOWED_DINPUT_PIDS = {STOCK_DINPUT_PID, DEFAULT_DINPUT_PID}
STOCK_PRODUCT_NAME = "8BitDo UM 2 Receiver"
DEFAULT_PRODUCT_NAME = "8BitDo UM 2 GyroD   "

# Ultimate 2 34-byte D-Input HID Report Descriptor (136 bytes, Report ID 0x01:
# hat, sticks, triggers, 24 buttons incl. L4/R4/PL/PR, battery, 12-byte 6-axis IMU;
# plus rumble Report ID 0x05 and vendor Report IDs 0x02/0x81 matching the stock protocol).
RDESC_34B = bytes.fromhex(
    "05010905a1018501050115002507463b01950175046514093981427501950481"
    "01150026ff0009300931093209359504750881020502150026ff0009c409c595"
    "0275088102050919012918150025017501951881020506092015002564750895"
    "178102050f0970850515002564750895049102850209027508953f8103858109"
    "037508953f9183c0"
)

# Authoritative section layout shared by patches/u2bt_dongle/build.py and apply_patch_file().
# Tuple: (section_name, flash_addr, min_len, max_len, expected_stock_hex_or_None)
PATCH_SECTIONS: list[tuple[str, int, int, int, str | None]] = [
    (".rdesc_len", ADDR_RDESC_LEN_INSN, 2, 2, "6031"),
    (".hook_usb_tx", 0x1E02384, 4, 4, "7c04c4ff"),
    (".hook_pack", 0x1E06AE6, 4, 4, "bfeaacf6"),
    (".hook_send", 0x1E06AF0, 4, 4, "bfeaa9dc"),
    (".hook_hk_a1", 0x1E0A652, 4, 4, "bfea8ab0"),
    (".hook_hk_a2", 0x1E0A794, 4, 4, "bfeae9af"),
    (".rdesc", ADDR_DINPUT_RDESC, 145, 145, None),
    (".cave_dinput", ADDR_CAVE_DINPUT, 1, 0x300, None),
    (".cave_tx_fix", ADDR_CAVE_TX_FIX, 1, 0x300, None),
]
PATCH_SECTION_MAP: dict[str, tuple[int, int, int, str | None]] = {
    name: (addr, min_len, max_len, stock_hex)
    for name, addr, min_len, max_len, stock_hex in PATCH_SECTIONS
}
CAVE_SECTIONS = {".cave_dinput", ".cave_tx_fix"}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def app_off(addr: int) -> int:
    return addr - APP_BASE


def atomic_write_bytes(path: Path, data: bytes, *, mode: int = 0o644) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    tmp_path = Path(tmp_name)
    try:
        os.chmod(tmp_path, mode)
        with os.fdopen(fd, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, path)
    except Exception:
        try:
            tmp_path.unlink(missing_ok=True)
        except Exception:
            pass
        raise


def validate_container_layout(raw: bytes) -> tuple[dict[str, bytes], dict]:
    """Validate that a type 98 .dat matches the pinned BR22 receiver layout and SFC chip key."""
    files, meta = ex.extract(raw)
    if meta["chip_key"] != EXPECTED_CHIP_KEY:
        raise ValueError(
            f"Unsupported chip key {meta['chip_key']:04x} (expected {EXPECTED_CHIP_KEY:04x})"
        )
    if meta["app_base"] != EXPECTED_APP_BASE:
        raise ValueError(
            f"Unsupported app_base {meta['app_base']:#x} (expected {EXPECTED_APP_BASE:#x})"
        )
    if meta["entry_point"] != EXPECTED_ENTRY_POINT:
        raise ValueError(
            f"Unsupported entry_point {meta['entry_point']:#x} (expected {EXPECTED_ENTRY_POINT:#x})"
        )
    if len(files["app.bin"]) != EXPECTED_APP_SIZE:
        raise ValueError(
            f"Unsupported app.bin size {len(files['app.bin'])} (expected {EXPECTED_APP_SIZE})"
        )
    return files, meta


def fetch_public_103(*, cache_path: Path | None = None) -> bytes:
    """Download stock public v1.03 (type 98) .dat directly from 8BitDo's official API."""
    if cache_path is not None and cache_path.is_file():
        cached = cache_path.read_bytes()
        if sha256(cached) == STOCK_DAT_SHA256:
            return cached

    ua = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/131.0.0.0 Safari/537.36"
    sel_req = urllib.request.Request(
        "https://support.8bitdo.com/restapi/link/v1/client/firmware/select",
        data=json.dumps({"firmware_type": 98, "beta": 0}).encode("utf-8"),
        headers={"Content-Type": "application/json", "User-Agent": ua},
    )
    with urllib.request.urlopen(sel_req, timeout=20) as resp:
        info = json.loads(resp.read().decode("utf-8"))
    items = info.get("data") or info.get("list") or []
    if not items:
        raise RuntimeError(f"Unexpected 8BitDo API response: {info}")

    chosen = None
    for item in items:
        ver_s = str(item.get("file_version") or item.get("version") or "").lstrip("vV")
        if ver_s == "1.03":
            chosen = item
            break
    if chosen is None:
        available = [str(i.get("file_version") or i.get("version") or "?") for i in items]
        raise RuntimeError(
            f"Pinned stock v1.03 not found in 8BitDo API (available: {available}). "
            "Provide a local stock v1.03 .dat via --base."
        )

    url = chosen.get("fileURL") or chosen.get("filePathName") or chosen.get("url")
    if not url:
        raise RuntimeError(f"No download URL in 8BitDo API item: {chosen}")
    if not url.startswith("http"):
        url = "https://support.8bitdo.com" + ("" if url.startswith("/") else "/") + url
    print(f"Fetching stock public firmware v1.03 from {url}...")
    dl_req = urllib.request.Request(url, headers={"User-Agent": ua})
    with urllib.request.urlopen(dl_req, timeout=30) as resp:
        raw = resp.read()
    actual = sha256(raw)
    if actual != STOCK_DAT_SHA256:
        raise ValueError(
            f"Downloaded .dat SHA-256 mismatch: {actual} != {STOCK_DAT_SHA256}\n"
            "  If 8BitDo updated their download archive, pass a local stock v1.03 .dat via --base."
        )
    validate_container_layout(raw)
    if cache_path is not None:
        atomic_write_bytes(cache_path, raw)
    return raw


def load_stock_dat(path: Path, *, require_exact_v103: bool = False) -> bytes:
    """Load and validate a stock type 98 public v1.02/v1.03 .dat image."""
    if not path.is_file():
        raise ValueError(f"not a regular file: {path}")
    raw = path.read_bytes()
    if require_exact_v103 and sha256(raw) != STOCK_DAT_SHA256:
        raise ValueError(
            f"{path}: full .dat SHA-256 {sha256(raw)} != pinned stock v1.03 {STOCK_DAT_SHA256}"
        )
    files, _ = validate_container_layout(raw)
    app_sha = sha256(files["app.bin"])
    if app_sha != STOCK_APP_SHA256:
        raise ValueError(
            f"{path}: app.bin SHA-256 {app_sha} != expected stock public v1.02/v1.03 {STOCK_APP_SHA256}"
        )
    return raw


def patch_pid_and_name(
    app: bytearray,
    *,
    pid: int,
    product_name: str,
    expected_orig_pid: int = STOCK_DINPUT_PID,
    expected_orig_name: str = STOCK_PRODUCT_NAME,
) -> None:
    if pid not in ALLOWED_DINPUT_PIDS:
        allowed = ", ".join(f"{p:#06x}" for p in sorted(ALLOWED_DINPUT_PIDS))
        raise ValueError(
            f"Unsupported D-Input USB PID {pid:#06x} (allowed: {allowed} so recovery discovery keeps working)"
        )
    if not product_name.isascii() or len(product_name) != len(STOCK_PRODUCT_NAME):
        raise ValueError(
            f"Product name must be exactly {len(STOCK_PRODUCT_NAME)} ASCII chars "
            f"(got {len(product_name)}: {product_name!r})"
        )
    if "wireless" in product_name.lower() and "receiver" not in product_name.lower() and "gyrod" not in product_name.lower():
        raise ValueError(
            f"Product name {product_name!r} would conflict with Ultimate 2 Wireless controller filtering"
        )

    pid_off = app_off(ADDR_DINPUT_PID)
    orig_pid = struct.unpack_from("<H", app, pid_off)[0]
    if orig_pid != expected_orig_pid:
        raise ValueError(
            f"Expected PID {expected_orig_pid:#06x} at {ADDR_DINPUT_PID:#x}, got {orig_pid:#06x}"
        )
    struct.pack_into("<H", app, pid_off, pid)

    old_b = expected_orig_name.encode("utf-16le")
    new_b = product_name.encode("utf-16le")
    str_off = app.find(old_b)
    if str_off == -1:
        raise ValueError(f"Product string {expected_orig_name!r} not found")
    app[str_off : str_off + len(old_b)] = new_b


def apply_patch_file(
    stock_dat: bytes,
    patch_path: Path,
    *,
    pid: int = DEFAULT_DINPUT_PID,
    product_name: str = DEFAULT_PRODUCT_NAME,
) -> bytes:
    """Apply a redistribution-safe u2bt-br22-patch-v1 JSON file onto a stock type 98 .dat."""
    files, _ = validate_container_layout(stock_dat)
    stock_app = files["app.bin"]
    doc = json.loads(patch_path.read_text())

    if doc.get("format") != "u2bt-br22-patch-v1":
        raise ValueError(f"unsupported patch format: {doc.get('format')!r}")
    if doc.get("base_app_sha256") != sha256(stock_app):
        raise ValueError("stock app.bin SHA-256 does not match patch base_app_sha256")
    exp_app_sha = doc.get("patched_app_sha256")
    if not isinstance(exp_app_sha, str) or len(exp_app_sha) != 64:
        raise ValueError("patch file is missing a valid 64-char patched_app_sha256")

    default_pid = int(doc.get("default_pid", DEFAULT_DINPUT_PID))
    default_name = str(doc.get("default_product_name", DEFAULT_PRODUCT_NAME))

    sections = doc.get("sections")
    if not isinstance(sections, list) or not sections:
        raise ValueError("patch file has no sections")

    app = bytearray(stock_app)
    patch_pid_and_name(app, pid=default_pid, product_name=default_name)

    ram_init_off = app_off(ADDR_TX_FIX_RAM_INIT)
    if any(b != 0 for b in app[ram_init_off : ram_init_off + 4]):
        raise ValueError(f"EP_BUSY_MASK flash backing at {ADDR_TX_FIX_RAM_INIT:#x} is not zero")

    seen: set[str] = set()
    placed: list[tuple[int, int, str]] = []
    for sec in sections:
        name = sec["name"]
        if name not in PATCH_SECTION_MAP:
            raise ValueError(f"unexpected patch section {name!r}")
        if name in seen:
            raise ValueError(f"duplicate patch section {name!r}")
        seen.add(name)

        exp_addr, min_len, max_len, stock_hex = PATCH_SECTION_MAP[name]
        flash_addr = int(sec["flash_addr"])
        if flash_addr != exp_addr:
            raise ValueError(f"section {name} address {flash_addr:#x} != expected {exp_addr:#x}")

        blob = bytes.fromhex(sec["data_hex"])
        if not (min_len <= len(blob) <= max_len):
            raise ValueError(f"section {name} size {len(blob)} outside {min_len}..{max_len}")

        off = app_off(flash_addr)
        end = off + len(blob)
        if off < 0 or end > len(app):
            raise ValueError(f"section {name} out of bounds at {flash_addr:#x}")
        for p_start, p_end, p_name in placed:
            if off < p_end and p_start < end:
                raise ValueError(f"section {name} overlaps {p_name}")
        placed.append((off, end, name))

        if name in CAVE_SECTIONS and any(b != 0 for b in app[off:end]):
            raise ValueError(f"code cave {name} at {flash_addr:#x} is not all zeros in stock")
        if stock_hex is not None:
            exp_stock = bytes.fromhex(stock_hex)
            if bytes(app[off : off + len(exp_stock)]) != exp_stock:
                raise ValueError(f"stock bytes differ at {name} ({flash_addr:#x})")
        if name == ".rdesc" and bytes(app[off : off + 4]) != bytes([0x05, 0x01, 0x09, 0x05]):
            raise ValueError(f"unexpected D-Input rdesc header at {flash_addr:#x}")

        app[off:end] = blob

    if seen != set(PATCH_SECTION_MAP):
        missing = sorted(set(PATCH_SECTION_MAP) - seen)
        raise ValueError(f"patch file is missing required sections: {missing}")

    # Always verify the default patched app.bin hash before applying any identity overrides.
    if sha256(bytes(app)) != exp_app_sha:
        raise ValueError(f"patched app.bin SHA-256 {sha256(bytes(app))} != expected {exp_app_sha}")

    if pid != default_pid or product_name != default_name:
        patch_pid_and_name(
            app,
            pid=pid,
            product_name=product_name,
            expected_orig_pid=default_pid,
            expected_orig_name=default_name,
        )

    patched_app = bytes(app)
    out_dat = rp.repack(stock_dat, patched_app)

    if pid == default_pid and product_name == default_name:
        exp_dat_sha = doc.get("patched_dat_sha256")
        if exp_dat_sha and sha256(stock_dat) == doc.get("base_dat_sha256") and sha256(out_dat) != exp_dat_sha:
            raise ValueError(f"patched .dat SHA-256 {sha256(out_dat)} != expected {exp_dat_sha}")

    return out_dat
