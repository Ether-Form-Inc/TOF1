# CAN time of flight sensor firmware

Firmware for Etherform's TOF1 time of flight sensor board (Rev B). An STM32G0B1KB reads an ST VL53L1CB
over I²C and publishes the distance on a DroneCAN bus, in the message that PX4 and ArduPilot expect from a
range finder.

## Hardware

From the board's schematic and PCB:

- **Power:** 5 V on J2 pin 2 (pins 3 and 4 are ground, pin 1 is not connected). A linear regulator
  (XC6222B281) makes the 2.8 V rail for the MCU, the sensor and the CAN transceiver's I/O.
- **CAN:** J2 pin 5 is CAN_P and pin 6 is CAN_N, through an MCP2562FD transceiver. Fitting jumper
  JP1 connects a 120 Ω termination resistor (R7) across the bus.
- **Programming:** J1 is the SWD header (pin 3 is 2.8 V, 4 is SWDIO, 6 is SWCLK, 12 is NRST, and 5, 7
  and 11 are ground).
- **Node ID jumpers:** J3, see below.
- **Sensor:** a VL53L1CB on I²C2 (PA6 SDA, PA7 SCL, address 0x29), with XSHUT on PB2 and its interrupt
  output on PB1 (the firmware polls and does not use the interrupt). The board has **no external I²C
  pull-ups**; the MCU's internal ones are used, which is why the bus runs at 100 kHz.

## What the board does

- Runs as a DroneCAN node: classic CAN, 29-bit IDs, **1 Mbit/s**, static node ID.
- Publishes

  | Message | Data type ID | Rate |
  |---|---|---|
  | `uavcan.equipment.range_sensor.Measurement` | 1050 | once per measurement, 10 Hz by default |
  | `uavcan.protocol.NodeStatus` | 341 | 1 Hz |

- Answers `uavcan.protocol.GetNodeInfo` (name `CAN ToF Sensor`, software version 1.0, hardware version 1.0, the chip's 96-bit unique ID).
- Has three parameters, set with the standard parameter services (`uavcan.protocol.param.GetSet` and
  `ExecuteOpcode`) or with `tools/can_param.py`; see Settings below. `uavcan.protocol.RestartNode`
  restarts the board.
- Reports the distance mode in use in `NodeStatus.sub_mode` (1 short, 2 long; in auto mode the one it has chosen).
- Reports its health in `NodeStatus`. A target out of range is not a fault; a missing or silent sensor is:

  | Health | Vendor status | Meaning |
  |---|---|---|
  | OK | 0 | The sensor is producing measurements. |
  | WARNING | 2 | The sensor answered once but has produced nothing for 3 s. |
  | ERROR | 1 | The sensor has never answered on I²C. |

  The node starts on CAN whether or not the sensor is present. It tries to reach the sensor every 500 ms,
  and resets it (XSHUT) and sets it up again if it falls silent for 3 s.
- Recovers by itself from CAN bus-off: it restarts the controller 100 ms after going bus-off and
  discards frames queued in the meantime.

### Message contents

`range_sensor.Measurement`:

| Field | Value |
|---|---|
| `sensor_id` | the node ID |
| `sensor_type` | `LIDAR` |
| `field_of_view` | 27° (0.471 rad), the sensor's default 16 × 16 SPAD region of interest |
| `beam_orientation_in_body_frame` | not defined (the board does not know how it is mounted) |
| `reading_type`, `range` | `VALID_RANGE` and the distance in metres when the sensor reports a good measurement; `TOO_FAR` and the longest range of the current mode (4 m long, 1.36 m short; fixed nominal values) when it saw no target or the signal was too weak; `TOO_CLOSE` and 0 when the target is below the sensor's minimum range; `UNDEFINED` and 0 for hardware errors and wrap-around |
| `timestamp` | always 0 |

### Distance modes

The VL53L1 has two distance modes, set with `DIST_MODE` (`short`, `long` or `auto`):

| Mode | Longest range (dark, white target) | Notes |
|---|---|---|
| long (default) | up to about 4 m, but only with a long timing budget (ST quotes 3.6 m at 100 ms and 4 m at 200 ms) | Less immune to ambient light: in sunlight the range is much shorter. |
| short | about 1.3 m | Best immunity to ambient light, so use it for outdoor ranging at low altitude. |

**Auto mode** (`DIST_MODE` 3) lets the firmware choose. It is not a third sensor mode and is not stored as one:
the choice is held in RAM, so switching never writes flash, and a restart begins in long mode again.

- It starts in long mode.
- Long to short: three readings in a row that are a valid distance below 900 mm, below the sensor's minimum range,
  or unusable.
- Short to long: three readings in a row that are out of range or unusable (a target that has moved beyond short
  range often shows as wrap-around), or a valid distance above 1200 mm.
- A reading that says nothing about the other mode (out of range in long mode, below minimum range in short mode)
  neither counts nor resets the count.
- The thresholds are 300 mm apart so a target near the changeover does not make it flip, and there are at least
  1000 ms between changes. A change reconfigures the sensor and the first reading after it is dropped, so the
  rate dips for one period at each change.
- A target that is beyond the short range but makes long mode report wrap-around would make auto mode cycle
  between the modes about once a second. Use a fixed mode if that matters.
- `Measurement` reports the same fields in either mode; the longest range given with `TOO_FAR` is that of the
  mode in use at the time.

The measurement period (`MEAS_PERIOD_MS`, 25 to 250 ms, default 100) also sets the **timing budget**, the time
the sensor integrates for each measurement: the largest of 20, 33, 50, 100 and 200 ms that is no more than 80% of
the period. The period has to be at least the budget, and the headroom matters: with a 100 ms budget at a 100 ms
period, long mode gave a failed reading between every pair of valid ones on the bench, and with the 20% headroom
every reading was valid. So the default 100 ms period runs a 50 ms budget, and a 100 ms budget needs a period of 125 ms
or more, and the 200 ms budget one of 250 ms. A longer budget gives a steadier reading and more range, so a faster rate trades
accuracy for update rate.
The range figures above are from ST's documentation, at their budgets (3.6 m at 100 ms, 4 m at 200 ms). With the
default 50 ms budget expect less than 3.6 m. For the full 4 m set `MEAS_PERIOD_MS` to 250.

## Node ID and settings

The node ID is **32 plus the number set on the four ID pins** on J3, GPIO1 to GPIO4 (J3 pins 1, 3,
5 and 7, with ground on the even pins; MCU pins PB5, PB4, PB3 and PA15). The pins have pull-ups, so **fitting a
jumper from a pin to ground sets its bit to 1** and an open pin reads 0. GPIO1 is the least significant bit, so the
jumpers select IDs 32 to 47, and with no jumpers fitted the ID is 32. (The GNSS board uses 16 to 31.)
The pins are read **only at startup**: after changing a jumper, reset or power-cycle the board.
Each fitted jumper draws about 0.28 mA, through the 10 kΩ pull-up (2.8 V / 10 kΩ).

| Jumpers fitted (rows) | Node ID |
|---|---|
| none | 32 |
| row 1 (pins 1 and 2) | 33 |
| row 2 (pins 3 and 4) | 34 |
| row 3 (pins 5 and 6) | 36 |
| row 4 (pins 7 and 8) | 40 |
| all four | 47 |

- A node ID stored in flash page 62 (byte at `0x0801F000`) overrides the jumpers and is used as is,
  without the offset of 32. It must be 1 to 127 (DroneCAN doesn't allow 0); an erased page or any other
  value is ignored and the jumpers are used.
- Flash page 63 (byte at `0x0801F800`) holds the measurement period in milliseconds (20 to 250, default 100).
- Flash page 61 (byte at `0x0801E800`) holds the distance mode (1 short, 2 long, 3 auto, default long).
- The linker script limits the application to the space below `0x0801E800`, so a build that grows into these
  pages fails to link. The image is about 87 KB, which leaves roughly 35 KB of headroom.

Erased flash (all `0xFF`) means "not set".

### Setting the parameters over CAN

The node has three integer parameters, each stored in the flash page above. 0 means "not set", which gives the
default. They can be changed with any DroneCAN tool that speaks the parameter services, or with
`tools/can_param.py`:

```bash
python3 -m venv .venv && .venv/bin/pip install -r tools/requirements.txt   # once
.venv/bin/python tools/can_param.py <node id> list
.venv/bin/python tools/can_param.py <node id> set DIST_MODE short          # short or long
.venv/bin/python tools/can_param.py <node id> set MEAS_PERIOD_MS 100       # 10 Hz
.venv/bin/python tools/can_param.py <node id> set NODE_ID 42               # store 42
.venv/bin/python tools/can_param.py <node id> restart                      # use the new node ID
.venv/bin/python tools/can_param.py <node id> set NODE_ID 0                # remove it, so the jumpers decide again
.venv/bin/python tools/can_param.py <node id> erase                        # every setting back to its default
```

- `DIST_MODE` and `MEAS_PERIOD_MS` take effect at once; `NODE_ID` is **used after the node restarts**
  (`RestartNode`, or a power cycle). A running node keeps its ID, so it is still reachable on the old one
  until then.
- Values out of range are ignored and the reply shows the value that is actually stored.
- Writing erases a flash page, which stops the processor for a few tens of milliseconds, so don't do it
  while the aircraft depends on the range data.
- `ExecuteOpcode` save is accepted (values are stored immediately) and erase clears all three settings.

## Building

You need CMake 3.15 or newer, `arm-none-eabi-gcc` (tested with the 12.3.rel1 toolchain that ships
with STM32CubeIDE 1.16), Python 3 with `venv`, git and network access to GitHub.

```bash
export PATH=/path/to/arm-none-eabi/bin:$PATH
mkdir build_stm && cd build_stm
cmake .. -DARCH=STM
make
```

The configure step fetches [libcanard](https://github.com/dronecan/libcanard), the DroneCAN DSDL
definitions and the `dronecan_dsdlc` generator, all pinned to specific commits
(`cmake/dronecan_c.cmake`), and generates the C message code in a Python virtualenv inside the
build directory. The firmware is `build_stm/stm_generated/can_tof_firmware.elf`.

## Flashing

With an ST-LINK, OpenOCD (the copy that ships with STM32CubeIDE is ST's fork):

```bash
openocd -s <openocd scripts> -f interface/stlink-dap.cfg -c "transport select dapdirect_swd" \
        -f target/stm32g0x.cfg -c "program can_tof_firmware.elf verify reset exit"
```

With a stock OpenOCD use `-f interface/stlink.cfg -f target/stm32g0x.cfg`. Programming the code
pages does not touch the settings pages.

### First flash of a new board

A factory-fresh STM32G0 has empty flash, and until software clears a flag after the first programming, every reset
starts ST's built-in bootloader instead of the application. `tools/flash_and_check.py` handles that, so a new board
needs only the ST-LINK on J1, 5 V on J2 and the steps below. The script has been tried on a board wiped with a mass
erase: it programmed it, the board started, and the CAN check passed.

```bash
python3 -m venv .venv && .venv/bin/pip install -r tools/requirements.txt     # once
mkdir build_stm && cd build_stm && cmake .. -DARCH=STM && make && cd ..       # once per firmware version
.venv/bin/python tools/flash_and_check.py                                      # program and check the board
.venv/bin/python tools/flash_and_check.py --set-node-id 42                     # same, and store a node ID
```

There is no prebuilt firmware in the repository, so the board needs a build first (see Building). Without a CAN
adapter use `--can-device none`: the board is then programmed and its flash read back, but not tested over CAN.

### Programming and checking boards with `tools/flash_and_check.py`

For one board at a time, this script programs the firmware over SWD with OpenOCD, resets the board and
checks that it works. It needs Python 3 and OpenOCD (set `OPENOCD` and `OPENOCD_SCRIPTS` if they are not
on the PATH; the copy in `/opt/st` is used if present). The CAN check also needs `pyserial` and a USB-CAN
adapter, as in the next section.

```bash
tools/flash_and_check.py                       # program the newest build, then check it over CAN
tools/flash_and_check.py --elf path/to/can_tof_firmware.elf
tools/flash_and_check.py --check-only          # do not program: just read and check the board
tools/flash_and_check.py --can-device none     # program only, no CAN check
tools/flash_and_check.py --set-node-id 42      # also store node ID 42 (1 to 127)
tools/flash_and_check.py --clear-node-id       # also remove a stored ID, so the jumpers decide again
```

What it does, in order:

1. Reads the board's stored node ID, distance mode and period, and the chip's unique ID.
2. Programs and verifies the firmware. The settings pages are not touched, so stored settings survive a
   firmware update. Use `--set-node-id` or `--clear-node-id` to change the node ID.
3. Resets the board and works out which node ID it should be using: the stored ID if there is one,
   otherwise 32 plus the jumpers.
4. Asks that node for `GetNodeInfo` over CAN and checks that the reply is valid and the health is OK.
5. Prints **PASS** or **FAIL** and exits with 0 only for PASS.

Notes:

- Other options are `--probe-serial` to choose one of several ST-LINKs, `--interface` and `--stock` for a
  stock OpenOCD, and `--boot-wait` to allow a slow start.
- A board that is not connected to the CAN bus fails the check even though it was programmed correctly:
  nothing acknowledges its frames, so it cannot answer. Check the CAN wiring (J2 pins 5 and 6 and ground) and
  the termination. Use `--can-device none` to skip the check when the board is not on a bus yet.
- A board with no working sensor reports health ERROR (vendor status 1) and fails the check.
- A new board has no stored node ID and starts as node 32 with no jumpers fitted. Two boards on one bus
  need different IDs, so set the ID with `--set-node-id`, the jumpers or `tools/can_param.py`.

## Watching the CAN traffic

`tools/can_monitor.py` decodes `Measurement` from a serial USB-CAN adapter (the 2 Mbaud
"USB-CAN analyzer" type), checks the multi-frame CRCs, reports the rate and shows the node's
`NodeStatus` (uptime, health and vendor status). `tools/can_range_live.py` prints every reading and the distance mode in use, continuously, which is the way to
watch auto mode working. `tools/can_getnodeinfo.py` sends a `GetNodeInfo`
request and decodes the reply:

```bash
pip install pyserial
python3 tools/can_monitor.py /dev/ttyUSB0 <node id> 30
python3 tools/can_range_live.py /dev/ttyUSB0 <node id>       # Ctrl-C to stop
python3 tools/can_getnodeinfo.py /dev/ttyUSB0 <node id>
```

![USB-CAN adapter](images/usbcan.png)

Notes from bring-up of the sibling boards:

- Plug the adapter straight into the computer. Behind a USB hub it stopped streaming after a
  second or two.
- The adapter only streams while DTR and RTS are asserted. The scripts do that.
- Reconfiguring the adapter briefly stops it acknowledging frames, which can push the board
  bus-off. The firmware recovers within about 100 ms.
- On Linux your user needs access to the serial device (the `dialout` group).

## Tests

The configuration rules (node ID selection, settings limits, timing budget, reading types) and the
message builder have unit tests that run on the PC. They need only CMake, a C/C++ compiler, Python 3 and network
access to GitHub (the same pinned dependencies as the firmware build, plus GoogleTest).

```bash
mkdir build_host && cd build_host
cmake .. -DARCH=HOST
make
./tests/tof_tests
```

## PX4

On the flight controller enable DroneCAN, set `UAVCAN_SUB_RNG` to receive range finder messages, and use a
node ID that doesn't clash with the flight controller's own.

## Status

Tested on a Rev B board with a USB-CAN adapter at 1 Mbit/s:

- `Measurement` at the configured rate (10 Hz by default, 20 Hz at a 50 ms period), `NodeStatus` at 1 Hz, and
  `GetNodeInfo`, all with correct CRCs and no errors over several minutes.
- The parameters: `DIST_MODE` and `MEAS_PERIOD_MS` apply at once and are stored in flash; out-of-range values are
  refused; `NODE_ID` is used after `RestartNode` and the old ID stops answering; `erase` returns every setting to its
  default and the flash pages read as erased.
- Short, long and auto modes, including auto switching from long to short for a near target and back to long
  when the target leaves.
- Programming and checking with `tools/flash_and_check.py`.
- Node ID selection from the J3 jumpers: with none fitted the ID is 32, and rows 1, 2, 3 and 4 on their own give 33,
  34, 36 and 40. Each time the board answered on that ID after a restart, the old ID stopped answering, and the pin
  states read over SWD matched.

## Layout

- `board/` firmware sources: the CAN node (`Canard_tof_node.h`), the FDCAN driver, the sensor wrapper
  (`Stm_tof.h`), the flash settings, and the board rules and message builder (`Tof_config.h`,
  `Tof_canard_messages.h`).
- `stm_generated/` CubeMX project: startup code, clock and pin setup, `main.cpp`.
- `third_party/vl53l1x_uld/` ST's VL53L1X driver with our platform layer.
- `cmake/` dependency fetching and message code generation.
- `tools/` host-side tools: flashing and checking a board, and CAN monitoring and configuration.
- `tests/` unit tests that run on the PC.
- `LICENSE`, `THIRD_PARTY_LICENSES.md` the project licence and the third-party notices.

## Licence

The firmware in this repository is licensed under the **MIT licence**; see `LICENSE`. Copyright (c) 2026
Ether Form Inc. Each source file carries an `SPDX-License-Identifier: MIT` line.

It includes and is built with third-party code under its own licences, listed with their notices in
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md):

- the ST HAL driver (BSD-3-Clause) and the CMSIS headers (Apache 2.0), included under `stm_generated/Drivers`,
  and the files generated by STM32CubeMX under `stm_generated/Core`, which keep ST's own headers;
- ST's VL53L1X driver, under `third_party/vl53l1x_uld`;
- libcanard and the DroneCAN DSDL definitions (MIT), fetched at build time;
- the C message code generated by `dronecan_dsdlc`. The generator is LGPL/GPL, but its licence lets you
  distribute a product that embeds the generated code under the MIT licence, which the firmware binary does.
  Include the notice in `THIRD_PARTY_LICENSES.md` when you distribute the firmware.
