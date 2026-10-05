#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ether Form Inc.
"""Programs a board over SWD with OpenOCD, then checks that it works.

For one board connected to an ST-LINK (and, optionally, to a serial USB-CAN adapter) it:
  1. reads the settings pages (stored node ID, distance mode and measurement period) and the chip's unique ID,
  2. programs and verifies the firmware (the settings pages are not touched),
  3. optionally stores a node ID (--set-node-id), then resets the board and works out which node ID it should
     be using, from the stored ID or the jumpers,
  4. asks that node for its GetNodeInfo over CAN and checks the reply,
and prints PASS or FAIL. The exit code is 0 only for PASS.

Examples:
  tools/flash_and_check.py                      # program the newest build, then check over CAN
  tools/flash_and_check.py --can-device none    # program only, no CAN check
  tools/flash_and_check.py --check-only         # do not program: just read and check the board
  tools/flash_and_check.py --set-node-id 42     # also store node ID 42 (1 to 127) in the board's flash
  tools/flash_and_check.py --clear-node-id      # also remove a stored node ID so the jumpers decide
  tools/flash_and_check.py --check-only --set-node-id 42   # change only the stored node ID

OpenOCD: set OPENOCD to the executable and OPENOCD_SCRIPTS to its script directory if they are not on the PATH
(the OpenOCD that ships with STM32CubeIDE is ST's fork and needs the default --interface stlink-dap.cfg;
a stock OpenOCD wants --interface stlink.cfg --stock).
"""
import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

ORIENT_PAGE = (60, 0x0801E000)   # beam orientation preset
MODE_PAGE = (61, 0x0801E800)     # distance mode: 1 short, 2 long
NODE_ID_PAGE = (62, 0x0801F000)
RATE_PAGE = (63, 0x0801F800)     # measurement period in ms
NODE_ID_OFFSET = 32          # the jumpers select 32 + (0 to 15)
DEFAULT_MODE, DEFAULT_PERIOD_MS = "long", 100
GPIOB_IDR, GPIOA_IDR = 0x50000410, 0x50000010
UID_ADDRESS = 0x1FFF7590


def newest_elf():
    found = glob.glob(os.path.join(REPO, "build*", "stm_generated", "can_tof_firmware.elf"))
    return max(found, key=os.path.getmtime) if found else None


def find_openocd():
    """The OpenOCD to use when none is given: the one on the PATH, else the one in STM32CubeIDE."""
    if shutil.which("openocd"):
        return "openocd", os.environ.get("OPENOCD_SCRIPTS")
    for root in sorted(glob.glob("/opt/st/stm32cubeide_*"), reverse=True):
        exe = glob.glob(os.path.join(root, "plugins", "com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_*",
                                     "tools", "bin", "openocd"))
        scripts = glob.glob(os.path.join(root, "plugins", "com.st.stm32cube.ide.mcu.debug.openocd_*", "resources",
                                         "openocd", "st_scripts"))
        if exe and scripts:
            return exe[0], scripts[0]
    return "openocd", os.environ.get("OPENOCD_SCRIPTS")


def parse_args():
    default_exe, default_scripts = find_openocd()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elf", default=newest_elf(), help="firmware to program (default: the newest build)")
    ap.add_argument("--openocd", default=os.environ.get("OPENOCD", default_exe))
    ap.add_argument("--scripts", default=os.environ.get("OPENOCD_SCRIPTS", default_scripts), help="OpenOCD script directory")
    ap.add_argument("--interface", default="stlink-dap.cfg", help="OpenOCD interface file (default stlink-dap.cfg)")
    ap.add_argument("--stock", action="store_true", help="stock OpenOCD (no 'transport select dapdirect_swd')")
    ap.add_argument("--probe-serial", help="select one ST-LINK when several are connected")
    ap.add_argument("--can-device", default="/dev/ttyUSB0", help="USB-CAN adapter, or 'none' to skip the CAN check")
    ap.add_argument("--check-only", action="store_true", help="do not program, only read and check")
    ap.add_argument("--set-node-id", type=int, metavar="N",
                    help="store node ID N (1 to 127) in the board's flash; 0 removes it so the jumpers decide")
    ap.add_argument("--clear-node-id", action="store_true", help="same as --set-node-id 0")
    ap.add_argument("--boot-wait", type=float, default=3.0, help="seconds to wait after the reset")
    return ap.parse_args()


class Probe:
    def __init__(self, args):
        self.args = args

    def run(self, commands, timeout=120):
        a = self.args
        cmd = [a.openocd]
        if a.scripts:
            cmd += ["-s", a.scripts]
        cmd += ["-f", "interface/" + a.interface]
        if not a.stock:
            cmd += ["-c", "transport select dapdirect_swd"]
        if a.probe_serial:
            cmd += ["-c", "adapter serial " + a.probe_serial]
        cmd += ["-f", "target/stm32g0x.cfg", "-c", "init"]
        for c in commands:
            cmd += ["-c", c]
        cmd += ["-c", "shutdown"]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        except FileNotFoundError:
            sys.exit("FAIL: OpenOCD not found (%s). Set OPENOCD / OPENOCD_SCRIPTS or use --openocd." % a.openocd)
        except subprocess.TimeoutExpired:
            return None
        return r.stdout + r.stderr

    def read_state(self):
        """Halts the core, reads the settings, the jumpers and the unique ID, then lets it run again."""
        out = self.run(["catch {halt}",
                        'echo "ID [read_memory 0x%08X 32 2]"' % NODE_ID_PAGE[1],
                        'echo "RATE [read_memory 0x%08X 32 2]"' % RATE_PAGE[1],
                        'echo "MODE [read_memory 0x%08X 32 2]"' % MODE_PAGE[1],
                        'echo "ORIENT [read_memory 0x%08X 32 2]"' % ORIENT_PAGE[1],
                        'echo "PINS [read_memory 0x%08X 32 1] [read_memory 0x%08X 32 1]"' % (GPIOB_IDR, GPIOA_IDR),
                        'echo "UID [read_memory 0x%08X 32 3]"' % UID_ADDRESS,
                        "resume"])
        if out is None:
            return None
        state = {}
        for line in out.splitlines():
            m = re.match(r"^(ID|RATE|MODE|ORIENT|PINS|UID) (.*)$", line.strip())
            if m:
                state[m.group(1)] = [int(x, 16) for x in m.group(2).split()]
        return state if {"ID", "RATE", "MODE", "ORIENT", "PINS", "UID"} <= set(state) else None


def stored_byte(words):
    """The one byte setting held in a flash page, or None if the page is erased."""
    raw = (words[1] << 32) | words[0]
    value = raw & 0xFF
    return None if raw == 0xFFFFFFFFFFFFFFFF or value == 0xFF else value


def expected_node_id(state):
    stored = stored_byte(state["ID"])
    if stored is not None and 1 <= stored <= 127:
        return stored, "stored in flash"
    gpiob, gpioa = state["PINS"]
    # J3 GPIO1 = PB5 (bit 0), GPIO2 = PB4, GPIO3 = PB3, GPIO4 = PA15 (bit 3); a fitted jumper pulls its pin low
    bits = [1 - ((gpiob >> 5) & 1), 1 - ((gpiob >> 4) & 1), 1 - ((gpiob >> 3) & 1), 1 - ((gpioa >> 15) & 1)]
    jumpers = sum(b << i for i, b in enumerate(bits))
    return NODE_ID_OFFSET + jumpers, "32 + jumpers (%d)" % jumpers


def describe(state):
    node = stored_byte(state["ID"])
    rate = stored_byte(state["RATE"])
    mode = stored_byte(state["MODE"])
    orient = stored_byte(state["ORIENT"])
    uid = "".join("%08x" % w for w in reversed(state["UID"]))
    return ("stored node ID: %s | stored distance mode: %s | stored period: %s | stored beam orientation: %s | "
            "chip unique ID: %s" %
            ("none (erased)" if node is None else node,
             {1: "short", 2: "long", 3: "auto"}.get(mode, "none (default %s)" % DEFAULT_MODE),
             "none (default %d ms)" % DEFAULT_PERIOD_MS if rate is None else "%d ms" % rate,
             "none (default down)" if orient is None else orient, uid))


def main():
    args = parse_args()
    if args.clear_node_id:
        if args.set_node_id not in (None, 0):
            sys.exit("--clear-node-id and --set-node-id %d contradict each other" % args.set_node_id)
        args.set_node_id = 0
    if args.set_node_id is not None and not 0 <= args.set_node_id <= 127:
        sys.exit("--set-node-id must be 1 to 127 (or 0 to clear); DroneCAN does not allow other node IDs")
    probe = Probe(args)
    failures = []

    def step(text):
        print("-", text, flush=True)

    step("reading the board")
    before = probe.read_state()
    if before is None:
        sys.exit("FAIL: could not read the board over SWD (is the ST-LINK connected and the board powered?)")
    print("   " + describe(before))

    commands = ["catch {halt}"]
    if not args.check_only:
        if not args.elf or not os.path.exists(args.elf):
            sys.exit("FAIL: no firmware found; build it or pass --elf")
        step("programming %s" % os.path.relpath(args.elf))
        commands.append("program %s verify" % args.elf.replace("\\", "/"))
    id_file = None
    if args.set_node_id is not None:
        if args.set_node_id == 0:
            step("removing the stored node ID")
            commands.append("flash erase_sector 0 %d %d" % (NODE_ID_PAGE[0], NODE_ID_PAGE[0]))
        else:
            step("storing node ID %d" % args.set_node_id)
            import tempfile
            id_file = tempfile.NamedTemporaryFile(suffix=".bin", delete=False)
            id_file.write(bytes([args.set_node_id]) + bytes(7))   # one doubleword, like the firmware writes it
            id_file.close()
            commands.append("flash write_image erase %s 0x%08X bin" % (id_file.name.replace("\\", "/"), NODE_ID_PAGE[1]))
    if len(commands) > 1:
        # A brand-new STM32G0 keeps its "main flash empty" flag (FLASH_ACR.EMPTY) set until software clears it after the
        # first programming, and while it is set every reset boots ST's system bootloader instead of the firmware
        commands.append("mmw 0x40022000 0x0 0x00010000")
        commands.append("reset run")
        out = probe.run(commands, timeout=300)
        if id_file:
            os.unlink(id_file.name)
        if out is None or (not args.check_only and "Verified OK" not in out):
            print((out or "OpenOCD timed out")[-1500:])
            sys.exit("FAIL: programming or verification failed")
        if not args.check_only:
            print("   programmed and verified")
        step("waiting %.0f s for the board to start" % args.boot_wait)
        time.sleep(args.boot_wait)
    else:
        time.sleep(0.2)

    step("reading the board again")
    after = probe.read_state()
    if after is None:
        sys.exit("FAIL: could not read the board after programming")
    print("   " + describe(after))
    if args.set_node_id is not None:
        wanted = None if args.set_node_id == 0 else args.set_node_id
        if stored_byte(after["ID"]) != wanted:
            failures.append("the stored node ID is %s, not the requested %s" % (stored_byte(after["ID"]), wanted))
    else:
        if stored_byte(before["ID"]) != stored_byte(after["ID"]):
            failures.append("the stored node ID changed unexpectedly")
        if stored_byte(before["MODE"]) != stored_byte(after["MODE"]) or stored_byte(before["RATE"]) != stored_byte(after["RATE"]) \
                or stored_byte(before["ORIENT"]) != stored_byte(after["ORIENT"]):
            failures.append("the stored distance mode, period or beam orientation changed unexpectedly")
        if stored_byte(after["ID"]) is not None:
            print("   note: a node ID is stored in flash and overrides the jumpers (use --clear-node-id to remove it)")

    node, why = expected_node_id(after)
    print("   the board should be node %d (%s)" % (node, why))

    if args.can_device.lower() == "none":
        step("CAN check skipped")
    elif not os.path.exists(args.can_device):
        step("CAN check skipped: %s not found" % args.can_device)
    elif not os.access(args.can_device, os.R_OK | os.W_OK):
        failures.append("no permission to use %s: on Linux add your user to the dialout group (sudo usermod -aG dialout "
                        "$USER, then log in again) or use --can-device none to skip the CAN check" % args.can_device)
    elif subprocess.run([sys.executable, "-c", "import serial"], capture_output=True).returncode != 0:
        step("CAN check skipped: pyserial is not installed for %s (pip install pyserial, in a virtual environment)" %
             sys.executable)
    else:
        step("asking node %d for GetNodeInfo over CAN" % node)
        r = subprocess.run([sys.executable, os.path.join(HERE, "can_getnodeinfo.py"), args.can_device, str(node)],
                           capture_output=True, text=True, timeout=30)
        text = (r.stdout + r.stderr).strip()
        for line in text.splitlines():
            print("   " + line)
        if r.returncode != 0 or "CRC OK" not in text:
            failures.append("no valid GetNodeInfo reply from node %d" % node)
        elif "health OK" not in text:
            failures.append("the node reports a fault (see the health above; ERROR with vendor code 1 means the "
                            "VL53L1CB is not answering on I2C)")

    print()
    if failures:
        print("FAIL: " + "; ".join(failures))
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
