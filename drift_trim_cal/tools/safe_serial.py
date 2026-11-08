#!/usr/bin/env python3
"""safe_serial.py - talk to an ESP32 over USB WITHOUT resetting it (2026-11-06).

Normal serial monitors toggle the DTR / RTS lines when they open the port. On ESP32 boards these
lines are wired to EN (reset) and IO0 (boot mode), so opening the port restarts the board - and a
board in the wrong state can even end up in the bootloader. This tool keeps DTR and RTS released
before the port is opened, so the running program just continues.

  python3 safe_serial.py list                           # serial ports (read-only)
  python3 safe_serial.py listen /dev/ttyUSB0 [seconds]  # print what the board sends (default 20 s)
  python3 safe_serial.py frame /dev/ttyUSB0 out.png     # cam_strip_bench: ask for one frame ('f')
  python3 safe_serial.py send /dev/ttyUSB0 p            # send one command character

Only reads the port, and only sends the single characters the bench sketch knows ('f', 'p').
"""
import sys
import time

import serial
from serial.tools import list_ports


def open_quiet(port, baud=115200):
    s = serial.Serial()
    s.port = port
    s.baudrate = baud
    s.timeout = 0.2
    s.dsrdtr = False
    s.rtscts = False
    s.dtr = False          # set BEFORE open: the lines stay released, no reset pulse
    s.rts = False
    s.open()
    return s


def cmd_list():
    ports = [p for p in list_ports.comports() if p.vid]     # USB only (hides the built-in ttyS*)
    if not ports:
        print("no USB serial ports found - is the board plugged in with a DATA cable?")
    for p in ports:
        print(f"{p.device}  {p.description}  vid:pid={p.vid and hex(p.vid)}:{p.pid and hex(p.pid)}  {p.hwid}")


def cmd_listen(port, seconds=20.0):
    s = open_quiet(port)
    end = time.time() + seconds
    try:
        while time.time() < end:
            line = s.readline()
            if line:
                print(line.decode(errors="replace").rstrip())
    finally:
        s.close()


def cmd_frame(port, out):
    import numpy as np
    s = open_quiet(port)
    try:
        s.reset_input_buffer()
        s.write(b"f")
        rows, w, h = [], 0, 0
        end = time.time() + 15
        while time.time() < end:
            line = s.readline().decode(errors="replace").strip()
            if line.startswith("FRAME"):
                _, w, h = line.split()
                w, h = int(w), int(h)
                rows = []
            elif line == "END" and w:
                break
            elif w and len(line) == 2 * w:
                rows.append(bytes.fromhex(line))
    finally:
        s.close()
    if not rows or len(rows) != h:
        print(f"incomplete frame ({len(rows)} of {h} rows)")
        return
    img = np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(h, w)
    try:
        import cv2
        cv2.imwrite(out, cv2.resize(img, (w * 4, h * 4), interpolation=cv2.INTER_NEAREST))
    except ImportError:
        with open(out.rsplit(".", 1)[0] + ".pgm", "wb") as f:
            f.write(f"P5 {w} {h} 255\n".encode() + img.tobytes())
    print(f"saved {out} ({w}x{h})")


def cmd_send(port, ch):
    if ch not in ("f", "p"):
        print("only 'f' or 'p'")
        return
    s = open_quiet(port)
    try:
        s.write(ch.encode())
    finally:
        s.close()


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a or a[0] == "list":
        cmd_list()
    elif a[0] == "listen":
        cmd_listen(a[1], float(a[2]) if len(a) > 2 else 20.0)
    elif a[0] == "frame":
        cmd_frame(a[1], a[2] if len(a) > 2 else "frame.png")
    elif a[0] == "send":
        cmd_send(a[1], a[2])
    else:
        print(__doc__)
