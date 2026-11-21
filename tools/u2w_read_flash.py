#!/usr/bin/env python3
"""Read-only flash dump for the 8BitDo Ultimate 2 Wireless (and 2C) over HID.

This tool sends exactly two commands, both of which only read:

  * class 0x05 command 0x0008 (identify; returns the firmware product id)
  * class 0x05 command 0x0005 (read up to 46 bytes of flash)

It never sends erase (0x0004), write (0x0003), multi-write (0x000b),
save-head (0x00c2), save-code-block (0x00c4), reset (0x0007), or the
jump-to-bootloader command (0x000c). There is no write path in this file.

Default target is the application firmware, which serves reads starting at
address 0 on the Ultimate 2 Wireless. For --boot, turn the controller off,
hold LB + RB, and plug in USB while holding both buttons.

Protocol (little-endian):

  request : 05 <cmd:2> <params:2> <len:2> <crc:2> <total:4> <addr:4> <payload>
  response: <prefix> 05 <status:2> <echo:2> <len:2> <crc:2> <total:4> <addr:4> <data>

App reports have IDs 0x81 (output) and 0x02 (input), followed by 63 bytes.
The 0x3208 bootloader uses unnumbered 64-byte reports. HIDAPI requires a
leading zero on writes. On reads the first byte is a zero sent by the
bootloader itself, not a report ID; the class byte follows it.

Usage:
    ./tools/u2w_read_flash.py --probe
    ./tools/u2w_read_flash.py --start 0 --length 0x80000 -o dump.bin
    ./tools/u2w_read_flash.py --boot --start 0 --length 0x5000 -o boot.bin
    ./tools/u2w_read_flash.py --dry-run

Nothing here is sent until you run it against a device. Use a spare unit for
the first run if you have one.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import tempfile
import time
from pathlib import Path

try:
    import hid
except ImportError:
    sys.exit("python hidapi is required: pip install hidapi")

VID = 0x2DC8
REPORT_APP = 0x81
REPORT_APP_REPLY = 0x02
CLASS = 0x05
CMD_IDENTIFY = 0x0008
CMD_READ = 0x0005
HEADER = 18  # payload offset in received buffers (both modes)
REPORT_SIZE = 64
MAX_CHUNK = 46

# Pads whose report 0x81/0x02 config channel is known to carry this protocol.
APP_PIDS = {
    0x310B: "Ultimate 2 Wireless (XInput USB / dongle)",
    0x6012: "Ultimate 2 Wireless (DInput USB)",
}
BOOT_PID = 0x3208
# Never open these: idle receivers and the older bootloader that has no
# documented read frame in this project.
REFUSE = {
    0x3109: "idle receiver",
    0x3107: "idle Ultimate 2 receiver",
    0x301C: "idle Ultimate 2C receiver",
    0x5750: "older 8BitDo bootloader; no verified frame",
}


def u16(buf: bytes, off: int) -> int:
    return int.from_bytes(buf[off : off + 2], "little")


def frame(
    cmd: int,
    addr: int = 0,
    length: int = 0,
    *,
    boot: bool,
    payload: bytes = b"",
    report_id: int | None = None,
) -> bytes:
    """Build a HIDAPI write buffer, including its leading report ID byte."""
    if cmd not in (CMD_IDENTIFY, CMD_READ):
        raise ValueError(f"not a read-only command: 0x{cmd:04x}")
    if boot and report_id not in (None, 0):
        raise ValueError("bootloader reports are unnumbered")
    if length > MAX_CHUNK:
        raise ValueError(f"length {length} exceeds {MAX_CHUNK}")
    if len(payload) > MAX_CHUNK:
        raise ValueError("payload too long")
    packet = bytearray(REPORT_SIZE + int(boot))
    packet[0] = 0 if boot else (REPORT_APP if report_id is None else report_id)
    fields = memoryview(packet)[1:]
    fields[0] = CLASS
    fields[1:3] = cmd.to_bytes(2, "little")
    fields[3:5] = (0).to_bytes(2, "little")
    fields[5:7] = length.to_bytes(2, "little")
    fields[7:9] = (0).to_bytes(2, "little")
    fields[9:13] = length.to_bytes(4, "little")
    fields[13:17] = addr.to_bytes(4, "little")
    fields[17 : 17 + len(payload)] = payload
    return bytes(packet)


def list_candidates(boot: bool) -> list[dict]:
    want = BOOT_PID if boot else None
    out = []
    for dev in hid.enumerate(VID):
        pid = dev["product_id"]
        if pid in REFUSE:
            continue
        if boot and pid != want:
            continue
        if not boot and pid not in APP_PIDS:
            continue
        out.append(dev)
    return out


def candidate_score(dev: dict) -> int:
    """Lower is better: the U2W config channel is interface 2 / page 0xFF7A."""
    page = dev.get("usage_page") or 0
    if page in (0xFF7A, 0xFF7B):
        return 0
    if dev.get("interface_number") == 2:
        return 1
    return 2


def choose_device(args) -> dict:
    if args.path:
        want = args.path.encode() if isinstance(args.path, str) else args.path
        for dev in hid.enumerate(VID):
            if dev["path"] == want:
                pid = dev["product_id"]
                if pid in REFUSE:
                    sys.exit(f"refusing {pid:#06x}: {REFUSE[pid]}")
                if args.boot and pid != BOOT_PID:
                    sys.exit(f"refusing {pid:#06x} in --boot mode; expected {BOOT_PID:#06x}")
                if not args.boot and pid not in APP_PIDS:
                    sys.exit(f"refusing {pid:#06x}: not a known app-mode pad")
                return dev
        sys.exit(f"no HID device at {args.path}")
    candidates = list_candidates(args.boot)
    if args.pid:
        candidates = [d for d in candidates if d["product_id"] == args.pid]
    if not candidates:
        target = "0x3208 bootloader" if args.boot else "an application-mode pad (0x310b/0x6012)"
        sys.exit(
            f"no {target} found. Attach the controller by USB, put it in the "
            "matching mode, and check `lsusb`. Idle receivers are refused."
        )
    candidates.sort(key=candidate_score)
    if len(candidates) > 1 and candidate_score(candidates[0]) == candidate_score(candidates[1]):
        lines = [
            f"  pid=0x{d['product_id']:04x} if={d.get('interface_number')} "
            f"usage=0x{(d.get('usage_page') or 0):04x}:{d.get('usage') or 0} "
            f"path={d['path']}"
            for d in candidates
        ]
        sys.exit("more than one equally good candidate; pass --path:\n" + "\n".join(lines))
    return candidates[0]


def parse_reply(reply: bytes, echo: int, *, boot: bool, addr: int, length: int) -> bytes:
    """Validate one reply and return its data."""
    # Identify requests four bytes but the Wireless bootloader returns two.
    size = 2 if echo == CMD_IDENTIFY else length
    end = HEADER + size
    if len(reply) < end:
        raise ValueError(f"short reply ({len(reply)} bytes, need {end})")
    prefix = bytes((0 if boot else REPORT_APP_REPLY, CLASS))
    if not reply.startswith(prefix):
        raise ValueError(f"unexpected reply prefix {reply[:4].hex(' ')}")
    status = u16(reply, 2)
    got = u16(reply, 4)
    if status != 0:
        raise ValueError(f"device status 0x{status:04x} for command 0x{echo:04x}")
    if got != echo:
        raise ValueError(f"reply echoes 0x{got:04x}, expected 0x{echo:04x}")
    if boot and echo == CMD_READ:
        got_addr = int.from_bytes(reply[14:18], "little")
        got_length = u16(reply, 6)
        if (got_addr, got_length) != (addr, length):
            raise ValueError(
                f"read reply for 0x{got_addr:08x}+{got_length}, expected 0x{addr:08x}+{length}"
            )
    return reply[HEADER:end]


class Pad:
    def __init__(
        self,
        dev: dict,
        *,
        boot: bool,
        report_id: int | None = None,
        timeout_ms: int = 1000,
        retries: int = 3,
    ):
        self.boot = boot
        self.report_id = report_id
        self.timeout_ms = timeout_ms
        self.retries = retries
        if hasattr(hid, "device"):  # cython-hidapi ("pip install hidapi")
            self.dev = hid.device()
            self.dev.open_path(dev["path"])
        else:  # ctypes binding ("pip install hid" / distro python-hid)
            self.dev = hid.Device(path=dev["path"])
        self.info = dev

    def close(self) -> None:
        try:
            self.dev.close()
        except Exception:
            pass

    def flush(self) -> None:
        if hasattr(self.dev, "set_nonblocking"):
            self.dev.set_nonblocking(1)
            try:
                for _ in range(64):
                    if not self.dev.read(REPORT_SIZE):
                        break
            finally:
                self.dev.set_nonblocking(0)
        else:
            for _ in range(64):
                if not self.dev.read(REPORT_SIZE, 1):
                    break

    def command(self, cmd: int, addr: int = 0, length: int = 0, payload: bytes = b"") -> bytes:
        pkt = frame(
            cmd, addr, length, boot=self.boot, payload=payload, report_id=self.report_id
        )
        last = None
        for attempt in range(self.retries):
            self.flush()
            written = self.dev.write(pkt)
            if written != len(pkt):
                last = RuntimeError(f"hid_write returned {written}, expected {len(pkt)}")
                continue
            deadline = time.monotonic() + self.timeout_ms / 1000
            while time.monotonic() < deadline:
                reply = bytes(self.dev.read(REPORT_SIZE, 50))
                if not reply:
                    continue
                try:
                    return parse_reply(reply, cmd, boot=self.boot, addr=addr, length=length)
                except ValueError as exc:
                    last = exc
                    # A stale reply from an earlier command is possible after
                    # a timeout; keep reading until the deadline.
            if attempt + 1 < self.retries:
                time.sleep(0.05)
        raise RuntimeError(f"command 0x{cmd:04x} failed: {last or 'timeout'}")

    def identify(self) -> int:
        data = self.command(CMD_IDENTIFY, length=4)
        return u16(data, 0)

    def read(self, addr: int, length: int) -> bytes:
        if length < 1 or length > MAX_CHUNK:
            raise ValueError(f"read length {length} outside 1..{MAX_CHUNK}")
        return self.command(CMD_READ, addr=addr, length=length)


def dump(pad: Pad, start: int, length: int, chunk: int, *, verify: bool, quiet: bool) -> bytes:
    out = bytearray()
    done = 0
    while done < length:
        want = min(chunk, length - done)
        out += pad.read(start + done, want)
        done += want
        if not quiet and (done % (chunk * 32) < chunk or done == length):
            print(f"\rread {done}/{length} bytes", end="", file=sys.stderr, flush=True)
    if not quiet:
        print(file=sys.stderr)
    if verify:
        if not quiet:
            print("verifying by reading again...", file=sys.stderr)
        check = bytearray()
        done = 0
        while done < length:
            want = min(chunk, length - done)
            check += pad.read(start + done, want)
            done += want
        if bytes(check) != bytes(out):
            for i, (a, b) in enumerate(zip(out, check)):
                if a != b:
                    sys.exit(f"verify mismatch at 0x{start + i:08x}: {a:02x} != {b:02x}")
            sys.exit("verify mismatch: lengths differ")
    return bytes(out)


def probe(pad: Pad) -> None:
    print(f"device: pid=0x{pad.info['product_id']:04x} path={pad.info['path']}")
    try:
        print(f"identify: firmware product id 0x{pad.identify():04x}")
    except Exception as exc:
        print(f"identify failed: {exc}")
    print("read probes (16 bytes each):")
    for addr in (0x0, 0x1000, 0x4000, 0x4FF0, 0x5000, 0x6000, 0x10000):
        try:
            data = pad.read(addr, 16)
            text = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
            print(f"  0x{addr:08x}: {data.hex(' ')}  {text}")
        except Exception as exc:
            print(f"  0x{addr:08x}: FAILED: {exc}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boot", action="store_true", help="target the 0x3208 bootloader (unnumbered HID)")
    ap.add_argument("--pid", type=lambda s: int(s, 0), help="select a product id")
    ap.add_argument("--path", help="select a hidraw path (e.g. /dev/hidraw5)")
    ap.add_argument(
        "--report-id",
        type=lambda s: int(s, 0),
        help="override the application output report id (default 0x81)",
    )
    ap.add_argument("--probe", action="store_true", help="read a few addresses and exit; no dump")
    ap.add_argument("--list", action="store_true", help="list 8BitDo HID interfaces and exit")
    ap.add_argument("--start", type=lambda s: int(s, 0), default=0, help="flash address (default 0)")
    ap.add_argument("--length", type=lambda s: int(s, 0), default=0x80000, help="bytes to read (default 0x80000)")
    ap.add_argument("--chunk", type=lambda s: int(s, 0), default=MAX_CHUNK, help=f"bytes per read (max {MAX_CHUNK})")
    ap.add_argument("-o", "--output", help="output .bin path")
    ap.add_argument("--verify", action="store_true", help="read the range twice and compare")
    ap.add_argument("--timeout-ms", type=int, default=1000)
    ap.add_argument("--dry-run", action="store_true", help="print the first packet and exit")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if args.pid == BOOT_PID:
        args.boot = True
    if args.boot and args.report_id not in (None, 0):
        ap.error("--boot uses unnumbered HID reports; --report-id must be 0 if given")
    if args.chunk > MAX_CHUNK or args.chunk < 1:
        sys.exit(f"--chunk must be 1..{MAX_CHUNK}")
    if args.length < 1:
        sys.exit("--length must be positive")
    if args.list:
        for dev in hid.enumerate(VID):
            pid = dev["product_id"]
            note = REFUSE.get(pid) or APP_PIDS.get(pid) or (
                "bootloader" if pid == BOOT_PID else "unknown"
            )
            print(
                f"pid=0x{pid:04x} if={dev.get('interface_number')} "
                f"usage=0x{(dev.get('usage_page') or 0):04x}:{dev.get('usage') or 0} "
                f"path={dev['path']}  {note}"
            )
        return 0
    if args.dry_run:
        print(
            frame(
                CMD_READ,
                args.start,
                min(args.chunk, args.length),
                boot=args.boot,
                report_id=args.report_id,
            ).hex(" ")
        )
        return 0

    out_path = Path(args.output) if args.output else None
    if out_path is not None:
        if out_path.is_dir():
            sys.exit(f"output path is a directory: {out_path}")
        out_path.parent.mkdir(parents=True, exist_ok=True)

    dev = choose_device(args)
    print(
        f"opening pid=0x{dev['product_id']:04x} "
        f"({APP_PIDS.get(dev['product_id'], 'bootloader' if args.boot else '?')})",
        file=sys.stderr,
    )
    pad = Pad(dev, boot=args.boot, report_id=args.report_id, timeout_ms=args.timeout_ms)
    try:
        if args.probe:
            probe(pad)
            return 0
        try:
            fw_pid = pad.identify()
            print(f"identify: firmware product id 0x{fw_pid:04x}", file=sys.stderr)
        except Exception as exc:
            print(f"identify skipped: {exc}", file=sys.stderr)
        data = dump(
            pad, args.start, args.length, args.chunk, verify=args.verify, quiet=args.quiet
        )
        if out_path is not None:
            fd, tmp_name = tempfile.mkstemp(prefix=f".{out_path.name}.", dir=out_path.parent)
            tmp_path = Path(tmp_name)
            try:
                os.chmod(tmp_path, 0o600)
                with os.fdopen(fd, "wb") as f:
                    f.write(data)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp_path, out_path)
            except Exception:
                tmp_path.unlink(missing_ok=True)
                raise
            print(f"saved {len(data)} bytes to {out_path} (mode 0600)", file=sys.stderr)
        print(f"sha256: {hashlib.sha256(data).hexdigest()}")
        return 0
    finally:
        pad.close()


if __name__ == "__main__":
    raise SystemExit(main())
