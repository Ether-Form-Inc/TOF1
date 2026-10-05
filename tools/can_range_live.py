#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
"""Prints the ToF node's range readings and the distance mode it is using, continuously, until Ctrl-C.
Uses the serial USB-CAN adapter (1 Mbit/s CAN, 2 Mbaud serial) and needs: pip install pyserial.

Usage: can_range_live.py [-d /dev/ttyUSB0] [--quiet] [NODE_ID]      (NODE_ID defaults to 32)

One line per reading:

    10:42:07.312   valid       0.198 m   short
    10:42:07.412   too_far     4.000 m   long

The mode is the one the node reported in its latest NodeStatus (sub_mode: 1 short, 2 long), sent once a second,
so right after a switch the column can lag by up to a second. For a TOO_FAR reading the range is also the longest
range of the mode that took it (1.36 m short, 4.0 m long), which shows the mode of that very reading.
Lines marked ">>>" show a change of mode, which in auto mode (DIST_MODE 3) is the firmware's choice.
Every 5 seconds a summary line gives the reading rate, the share of valid readings and the CRC errors.

--quiet prints only the mode changes and the summary lines.
"""
import argparse
import collections
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("This tool needs pyserial: pip install pyserial (in a virtual environment: python3 -m venv .venv && "
             ".venv/bin/pip install -r tools/requirements.txt)")

ap = argparse.ArgumentParser(add_help=True)
ap.add_argument("-d", "--device", default="/dev/ttyUSB0")
ap.add_argument("--quiet", action="store_true", help="only mode changes and summary lines")
ap.add_argument("node", type=int, nargs="?", default=32)
args = ap.parse_args()

RANGE_ID, STATUS_ID = 1050, 341
SIGNATURE_RANGE = 0x68FFFE70FC771952
MODES = {1: "short", 2: "long"}
READINGS = ["undefined", "valid", "too_close", "too_far"]
HEALTH = ["OK", "WARNING", "ERROR", "CRITICAL"]
COLORS = {"valid": "32", "too_far": "33", "too_close": "33", "undefined": "31"}   # green, yellow, yellow, red
use_color = sys.stdout.isatty()


def color(text, code):
    return "\033[%sm%s\033[0m" % (code, text) if use_color else text


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def bits(data, off, n):
    return int.from_bytes(data, "little") >> off & ((1 << n) - 1)


def half(v):
    return struct.unpack("<e", struct.pack("<H", v))[0]


def decode_range(p):
    """Layout of uavcan.equipment.range_sensor.Measurement: timestamp 56, sensor_id 8, orientation 16,
    field_of_view float16, sensor_type 5 + reading_type 3 (sub-byte fields are packed from the top of the byte),
    range float16."""
    return {"sensor_id": bits(p, 56, 8), "reading": READINGS[bits(p, 96, 8) & 7], "range_m": half(bits(p, 104, 16))}


def frames(buf):
    """Yields (can_id, data) for each complete frame in buf and removes what it used."""
    i = 0
    used = 0
    out = []
    while i < len(buf):
        if buf[i] == 0xAA and i + 1 < len(buf) and (buf[i + 1] & 0xC0) == 0xC0:
            end = i + 6 + (buf[i + 1] & 0x0F)
            if end >= len(buf):
                break
            if buf[end] == 0x55:
                out.append((int.from_bytes(buf[i + 2:i + 6], "little"), bytes(buf[i + 6:end])))
                i = end + 1
                used = i
                continue
        i += 1
        used = i
    del buf[:used]
    return out


# The adapter only streams while DTR and RTS are asserted, so set them before the port opens
port = serial.Serial(None, 2000000, timeout=0.05, stopbits=serial.STOPBITS_TWO, exclusive=True)
port.port = args.device
port.dtr = True
port.rts = True
try:
    port.open()
except serial.SerialException as e:
    sys.exit("cannot use %s: %s (is another program using the adapter, or is your user missing from the dialout "
             "group?)" % (args.device, e))
cfg = bytearray([0xAA, 0x55, 0x12, 0x01, 0x01] + [0] * 8 + [0x00, 0x01, 0, 0, 0, 0])   # 1 Mbit/s, normal mode
cfg.append(sum(cfg[2:]) & 0xFF)
port.write(cfg)
time.sleep(0.3)
port.reset_input_buffer()

buf = bytearray()
transfers = {}
mode = None            # the node's latest reported mode: 1 or 2
health = None
counts = collections.Counter()
crc_errors = 0
window_start = time.time()
last_data = time.time()
print("Listening for node %d on %s (Ctrl-C to stop)" % (args.node, args.device), flush=True)

try:
    while True:
        buf += port.read(4096)
        for cid, d in frames(buf):
            typ, src = (cid >> 8) & 0xFFFF, cid & 0x7F
            if cid & 0x80 or src != args.node or not d:   # service frames and other nodes are not wanted
                continue
            tail = d[-1]
            sot, eot = tail >> 7, tail >> 6 & 1
            if sot:
                transfers[typ] = bytearray()
            if typ not in transfers:
                continue
            transfers[typ] += d[:-1]
            if not eot:
                continue
            p = bytes(transfers.pop(typ))
            if typ == RANGE_ID:
                # Multi-frame transfers start with a CRC-16 over the data type signature and the payload
                if crc16(SIGNATURE_RANGE.to_bytes(8, "little") + p[2:]) != int.from_bytes(p[:2], "little"):
                    crc_errors += 1
                    continue
                r = decode_range(p[2:])
                now = time.time()
                last_data = now
                counts[r["reading"]] += 1
                if not args.quiet:
                    # A TOO_FAR reading carries the longest range of its own mode
                    own = {1.36: "short", 4.0: "long"}.get(round(r["range_m"], 2)) if r["reading"] == "too_far" else None
                    shown = own or MODES.get(mode, "?")
                    stamp = time.strftime("%H:%M:%S", time.localtime(now)) + ".%03d" % (now % 1 * 1000)
                    print("%s   %s %7.3f m   %s" % (stamp, color("%-10s" % r["reading"], COLORS[r["reading"]]),
                                                   r["range_m"], shown), flush=True)
            elif typ == STATUS_ID and len(p) >= 7:
                new_mode = p[4] & 7
                health = p[4] >> 6
                if new_mode != mode:
                    old = MODES.get(mode, "none yet")
                    print(color(">>> %s  mode %s -> %s" % (time.strftime("%H:%M:%S"), old, MODES.get(new_mode, new_mode)),
                                "1;36"), flush=True)
                    mode = new_mode
        now = time.time()
        if now - window_start >= 5:
            total = sum(counts.values())
            rate = total / (now - window_start)
            valid = 100.0 * counts["valid"] / total if total else 0.0
            line = "--- %.1f readings/s, %.0f%% valid (%d valid, %d too_far, %d too_close, %d undefined), %d CRC errors, " \
                   "mode %s, health %s" % (rate, valid, counts["valid"], counts["too_far"], counts["too_close"],
                                           counts["undefined"], crc_errors, MODES.get(mode, "?"),
                                           HEALTH[health] if health is not None else "?")
            print(line, flush=True)
            counts.clear()
            crc_errors = 0
            window_start = now
except KeyboardInterrupt:
    print()
finally:
    port.close()
