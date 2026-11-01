#!/usr/bin/env python3
"""Collect a reproducible ESP32 boot/flash diagnostic bundle.

Run with the main ESP32 connected and no motors powered. This does not erase or
write flash. It captures the reset loop, chip/flash identification, and a readback
of the bootloader, partition table, and application region.
"""
from __future__ import annotations

import argparse
import hashlib
import pathlib
import subprocess
import sys
import time


def run(cmd: list[str], output: pathlib.Path) -> int:
    with output.open("w", encoding="utf-8") as f:
        f.write("$ " + " ".join(cmd) + "\n\n")
        p = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, text=True)
    return p.returncode


def sha256(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--expected-bin", type=pathlib.Path, help="matching cas_box.ino.bin")
    ap.add_argument("--seconds", type=int, default=20)
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path("esp32_diagnostic"))
    ap.add_argument("--esptool", default="esptool")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    try:
        import serial  # type: ignore
    except ImportError:
        print("Install pyserial first: python3 -m pip install pyserial", file=sys.stderr)
        return 2

    # Open before reset so the complete boot sequence is captured.
    with serial.Serial(args.port, 115200, timeout=0.2) as ser:
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.rts = False
        deadline = time.monotonic() + args.seconds
        data = bytearray()
        while time.monotonic() < deadline:
            chunk = ser.read(4096)
            if chunk:
                data.extend(chunk)
    (args.out / "serial_115200.log").write_bytes(bytes(data))

    base = [args.esptool, "--chip", "esp32", "--port", args.port, "--baud", "115200"]
    rc = 0
    rc |= run(base + ["chip_id"], args.out / "chip_id.txt")
    rc |= run(base + ["flash_id"], args.out / "flash_id.txt")
    # Read through the application end used by the verified default 4 MB build.
    dump = args.out / "flash_read_0_to_0x140000.bin"
    rc |= subprocess.run(base + ["read_flash", "0x0", "0x140000", str(dump)], check=False).returncode
    if args.expected_bin and args.expected_bin.exists():
        expected = args.expected_bin
        actual_app = args.out / "app_readback.bin"
        with dump.open("rb") as src, actual_app.open("wb") as dst:
            src.seek(0x10000)
            dst.write(src.read(expected.stat().st_size))
        report = args.out / "hash_report.txt"
        report.write_text(
            f"expected={expected}\nexpected_sha256={sha256(expected)}\n"
            f"readback={actual_app}\nreadback_sha256={sha256(actual_app)}\n"
            f"match={sha256(expected) == sha256(actual_app)}\n",
            encoding="utf-8",
        )
    (args.out / "README.txt").write_text(
        "Send this entire directory without editing serial_115200.log.\n"
        "The diagnostic is read-only; it does not erase or flash the board.\n",
        encoding="utf-8",
    )
    print(f"Diagnostic bundle written to {args.out.resolve()}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
