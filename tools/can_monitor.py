#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
"""Minimal DroneCAN ToF monitor for the serial USB-CAN analyzer (1 Mbit/s CAN, 2 Mbaud serial).

Prints the range_sensor.Measurement (1050) broadcasts of one node and the rate they arrive at.
Usage: can_monitor.py [/dev/ttyUSB0] [node_id] [seconds] [all]
"""
import struct
import sys
import time
import collections
import serial

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
node = int(sys.argv[2]) if len(sys.argv) > 2 else 32
duration = float(sys.argv[3]) if len(sys.argv) > 3 else 10
show_all = len(sys.argv) > 4 and sys.argv[4] == "all"  # print every reading, not just a sample

# The adapter only streams while DTR and RTS are asserted, so set them before the port opens.
# With pyserial's defaults the stream stops after about a second.
s = serial.Serial(None, 2000000, timeout=0.05, stopbits=serial.STOPBITS_TWO)
s.port = dev
s.dtr = True
s.rts = True
s.open()
cmd = bytearray([0xAA, 0x55, 0x12, 0x01, 0x01] + [0] * 8 + [0x00, 0x01, 0, 0, 0, 0])  # 1 Mbit/s, normal
cmd.append(sum(cmd[2:]) & 0xFF)
s.write(cmd)
s.reset_input_buffer()


consumed = 0


def frames(buf):
    """Yields (can_id, data) for each complete frame; sets `consumed` to the bytes fully handled."""
    global consumed
    i = 0
    consumed = 0
    while i < len(buf):
        if buf[i] == 0xAA and i + 1 < len(buf) and (buf[i + 1] & 0xC0) == 0xC0:
            dlc = buf[i + 1] & 0x0F
            end = i + 6 + dlc
            if end >= len(buf):
                break  # incomplete frame: keep it for the next read
            if buf[end] == 0x55:
                yield int.from_bytes(buf[i + 2:i + 6], "little"), bytes(buf[i + 6:end])
                i = end + 1
                consumed = i
                continue
        i += 1
        consumed = i


def bits(data, off, n, signed=False):
    v = int.from_bytes(data, "little") >> off & ((1 << n) - 1)
    if signed and v >> (n - 1):
        v -= 1 << n
    return v


def half(v):
    return struct.unpack("<e", struct.pack("<H", v))[0]


def msb_bits(data, off, n, signed=False):
    """A field of n <= 8 bits that starts at bit `off` of the DroneCAN bit stream, where bit 0 is the top bit of
    byte 0 and a field is written most significant bit first."""
    v = 0
    for i in range(n):
        v = v << 1 | (data[(off + i) // 8] >> (7 - (off + i) % 8) & 1)
    return v - (1 << n) if signed and v >> (n - 1) else v


def beam_text(p):
    """The beam orientation of a Measurement payload, as text."""
    roll, pitch, yaw = (msb_bits(p, 64 + 5 * i, 5, True) for i in range(3))
    if not msb_bits(p, 79, 1):
        return "not defined"
    return "roll %d, pitch %d, yaw %d degrees" % (roll * 15, pitch * 15, yaw * 15) + \
        (" (down)" if pitch == -6 else " (up)" if pitch == 6 else "")


def decode_range(p):
    # Field layout from uavcan.equipment.range_sensor.Measurement (1050): timestamp 56, sensor_id 8,
    # CoarseOrientation 16 (3 x int5 + bool), field_of_view float16, sensor_type 5 + reading_type 3, range float16.
    # DroneCAN packs sub-byte fields from the top of the byte, so sensor_type is the upper 5 bits.
    o = 56
    out = {"sensor_id": bits(p, o, 8)}; o += 8
    o += 16
    out["fov_deg"] = round(half(bits(p, o, 16)) * 180 / 3.14159265, 1); o += 16
    out["sensor_type"] = bits(p, o, 8) >> 3
    out["reading"] = ["undefined", "valid", "too_close", "too_far"][bits(p, o, 8) & 7]; o += 8
    out["range_m"] = round(half(bits(p, o, 16)), 3)
    out["beam"] = beam_text(p)
    return out


SIGNATURES = {1050: 0x68FFFE70FC771952}


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


ranges = []
crc_bad = collections.Counter()
last_node_status = None
transfers = {}
count = collections.Counter()
times = collections.defaultdict(list)
t0 = time.time()
buf = bytearray()
while time.time() - t0 < duration:
    buf += s.read(4096)
    consumed = 0
    for cid, d in frames(buf):
        typ, src = (cid >> 8) & 0xFFFF, cid & 0x7F
        if cid & 0x80 or src != node or not d:
            continue
        tail = d[-1]
        sot, eot, tog, tid = tail >> 7, tail >> 6 & 1, tail >> 5 & 1, tail & 31
        key = (typ, src)
        if sot:
            transfers[key] = bytearray()
        if key not in transfers:
            continue
        transfers[key] += d[:-1]
        if eot:
            p = bytes(transfers.pop(key))
            if typ in SIGNATURES:
                # Multi-frame transfers start with a CRC-16 over the data type signature and payload
                sig = SIGNATURES[typ].to_bytes(8, "little")
                if crc16(sig + p[2:]) != int.from_bytes(p[:2], "little"):
                    crc_bad[typ] += 1
                p = p[2:]
            count[typ] += 1
            if typ == 341 and len(p) >= 7:
                # NodeStatus: uptime 32, health 2, mode 3, sub mode 3, vendor status 16 (little endian)
                # DroneCAN packs sub-byte fields from the top of the byte: health in bits 7..6, mode in 5..3
                last_node_status = (bits(p, 0, 32), p[4] >> 6, (p[4] >> 3) & 7, bits(p, 40, 16))
            times[typ].append(time.time())
            if typ == 1050:
                r = decode_range(p)
                ranges.append(r["range_m"] if r["reading"] == "valid" else None)
                if show_all or count[typ] <= 5 or count[typ] % 20 == 0:
                    print("Range", r)
    del buf[:consumed]

for typ, n in sorted(count.items()):
    t = times[typ]
    rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0
    print(f"type {typ}: {n} transfers, {rate:.2f} Hz, CRC errors: {crc_bad[typ]}")
if last_node_status:
    names = ["OK", "WARNING", "ERROR", "CRITICAL"]
    print(f"NodeStatus: uptime {last_node_status[0]} s, health {names[last_node_status[1]]}, mode {last_node_status[2]}, vendor status {last_node_status[3]}")
valid = [r for r in ranges if r is not None]
if ranges:
    print(f"{len(valid)} of {len(ranges)} readings valid" + (f", range {min(valid):.3f} to {max(valid):.3f} m" if valid else ""))
if not count:
    print("no broadcasts received from node", node)
