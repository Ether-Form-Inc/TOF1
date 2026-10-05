#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
"""Reads and sets the ToF sensor's parameters over CAN through the serial USB-CAN adapter (needs: pip install dronecan
pyserial; the protocol is in can_link.py).

Usage: can_param.py [-d /dev/ttyUSB0] NODE_ID command [name] [value]

  list                 show every parameter
  get NAME             show one parameter
  set NAME VALUE       store a value (0 removes it, back to the default)
  erase                clear every stored value, back to the defaults
  restart              restart the node (applies a new NODE_ID)

Parameters:
  NODE_ID          0 to 127. 0 = not set: the node ID is 32 plus the J3 jumpers. Used after a restart.
  DIST_MODE        short (1), long (2) or auto (3). 0 = not set, which is long. Applied at once.
                   Auto starts in long mode and switches to short for near targets and back (see README).
  MEAS_PERIOD_MS   25 to 250 ms between measurements. 0 = not set, which is 100 (10 Hz). Applied at once.
  BEAM_ORIENT      The direction the beam points, reported in every range message so the flight controller knows it.
                   0 = not set, which is down. 1 down, 2 up, 3 forward, then horizontal beams turned to the right in
                   45 degree steps: 4 = 45, 5 = 90 (right), 6 = 135, 7 = 180 (back), 8 = 225, 9 = 270 (left), 10 = 315.
                   Applied at once.
DIST_MODE accepts the words short, long and auto, and BEAM_ORIENT the words down, up, forward, right, back and left, as
well as the numbers.
"""
import argparse
import sys

from can_link import CanLink, LinkError

ap = argparse.ArgumentParser(add_help=False)
ap.add_argument("-d", "--device", default="/dev/ttyUSB0")
ap.add_argument("node", type=int)
ap.add_argument("command", choices=["list", "get", "set", "erase", "restart"])
ap.add_argument("args", nargs="*")
args = ap.parse_args()

WORDS = {"DIST_MODE": {"short": 1, "long": 2, "auto": 3},
         "BEAM_ORIENT": {"down": 1, "up": 2, "forward": 3, "right": 5, "back": 7, "left": 9}}
ORIENT_NAMES = {0: "down (default)", 1: "down", 2: "up", 3: "forward", 4: "45 right", 5: "right", 6: "135 right",
                7: "back", 8: "225 right", 9: "left", 10: "315 right"}


def show(p):
    extra = {"DIST_MODE": {0: " (long)", 1: " (short)", 2: " (long)", 3: " (auto)"}.get(p.value, ""),
             "MEAS_PERIOD_MS": " (100 ms)" if p.value == 0 else "",
             "BEAM_ORIENT": " (%s)" % ORIENT_NAMES.get(p.value, "?")}.get(p.name, "")
    print(f"{p.name:16s} = {p.value:4d}{extra}   max {p.max}")


def main():
    link = CanLink(args.device, args.node)
    cmd, a = args.command, args.args
    if cmd == "list":
        for p in link.params():
            show(p)
    elif cmd == "get":
        if len(a) != 1:
            sys.exit("get needs a parameter name")
        show(link.get(a[0]))
    elif cmd == "set":
        if len(a) != 2:
            sys.exit("set needs a name and a value")
        if a[1].lstrip("-").isdigit():
            wanted = int(a[1])
        else:
            wanted = WORDS.get(a[0], {}).get(a[1].lower())
            if wanted is None:
                sys.exit("%s takes a number%s" % (a[0], (" or one of: " + ", ".join(WORDS[a[0]])) if a[0] in WORDS else ""))
        p = link.set(a[0], wanted)
        show(p)
        if p.value != wanted:
            print(f"  not applied: the board kept {p.value} (value out of range)")
        elif a[0] == "NODE_ID":
            print("  applies after a restart: can_param.py %d restart" % args.node)
    elif cmd == "erase":
        print("erase:", "ok" if link.erase_all() else "failed")
    elif cmd == "restart":
        print("restart:", "accepted" if link.restart() else "refused")


try:
    main()
except LinkError as e:
    sys.exit(str(e))
