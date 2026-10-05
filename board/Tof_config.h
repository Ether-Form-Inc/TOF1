// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// The board's configuration rules, with no hardware dependencies so they can be unit tested on the PC:
// how the node ID is chosen, the stored settings and their limits, and how a VL53L1 reading maps to a
// DroneCAN reading type.

#include <cstdint>

namespace Tof_config
{
// The ID jumpers on J3 select a node ID of node_id_offset plus the jumper value (0 to 15), so the
// IDs 32 to 47 are used. (The GNSS board uses 16 to 31.)
constexpr uint8_t node_id_offset = 32;

// A node ID stored in flash overrides the jumpers and is used as is. DroneCAN node IDs are 1 to 127.
// Anything else (erased flash, 0, 128 and up) is ignored.
inline uint8_t resolve_node_id(uint8_t jumpers, bool have_stored, uint8_t stored)
{
  if (have_stored && stored >= 1 && stored <= 127)
  {
    return stored;
  }
  return (uint8_t)(node_id_offset + (jumpers & 0x0F));
}

// VL53L1 distance mode, as the ST API numbers it
constexpr uint8_t mode_short = 1;
constexpr uint8_t mode_long = 2;
// Not a sensor mode: the firmware chooses between short and long itself (see Auto_mode)
constexpr uint8_t mode_auto = 3;
constexpr uint8_t default_mode = mode_long;

// Time between measurements in milliseconds. 25 ms is the fastest that leaves the headroom described at
// timing_budget_ms() below over the sensor's shortest timing budget of 20 ms.
constexpr uint16_t min_period_ms = 25;
constexpr uint16_t max_period_ms = 250;
constexpr uint16_t default_period_ms = 100;

inline uint8_t resolve_mode(bool have_stored, uint8_t stored)
{
  return (have_stored && stored >= mode_short && stored <= mode_auto) ? stored : default_mode;
}

inline uint16_t resolve_period_ms(bool have_stored, uint8_t stored)
{
  return (have_stored && stored >= min_period_ms && stored <= max_period_ms) ? stored : default_period_ms;
}

// The sensor's timing budget is how long it integrates for each measurement. Longer is more accurate and
// reaches further. The API only accepts 15 (short mode only), 20, 33, 50, 100, 200 and 500 ms, and the measurement
// period has to be at least the budget. Setting the two equal is not enough: on the bench, long mode then gave a
// failed reading between every pair of valid ones (a period of 100 ms with a 100 ms budget), and with 20% headroom
// every reading was valid. So use the largest allowed budget that is no more than 80% of the period.
inline uint16_t timing_budget_ms(uint16_t period_ms)
{
  static const uint16_t budgets[] = {200, 100, 50, 33, 20};
  const uint16_t limit = (uint16_t)(period_ms * 4 / 5);
  for (uint16_t b : budgets)
  {
    if (limit >= b)
    {
      return b;
    }
  }
  return 20;
}

// Longest distance the sensor can report in each mode, in millimetres (ST datasheet, dark, 100% reflectance;
// much less in sunlight or on a dark target)
inline uint16_t max_range_mm(uint8_t mode)
{
  return mode == mode_short ? 1360 : 4000;
}

// DroneCAN range sensor reading types (uavcan.equipment.range_sensor.Measurement)
enum class Reading
{
  undefined,
  valid,
  too_close,
  too_far
};

// The VL53L1 range status, as returned by the ST API after its translation (0 = valid, 3 = target below the
// minimum range). A target that is
// out of range shows up as a signal failure (2) or "out of bounds" (4); a sigma failure (1) means the
// measurement is too noisy to use, which usually also means there is nothing to see.
inline Reading reading_for(uint8_t range_status)
{
  switch (range_status)
  {
  case 0:
    return Reading::valid;
  case 3:
    return Reading::too_close;
  case 1:
  case 2:
  case 4:
    return Reading::too_far;
  default:
    return Reading::undefined; // hardware failure, wrap-around, or no update
  }
}
} // namespace Tof_config

namespace Tof_config
{
// Chooses between the short and long distance modes for DIST_MODE = auto. It starts in long mode, which reaches
// furthest, and moves to short mode when long mode cannot make sense of what is in front of the sensor: a near
// target (long mode reports wrap-around for a close one), or an unusable reading. It moves back to long mode when
// short mode runs out of range. A target that has moved beyond short mode's range shows up as a signal failure
// (too far) or, often, as wrap-around (undefined), so both count. A reading that says nothing about the other
// mode (too far in long mode, too close in short mode) neither counts nor resets the count. The two distance
// thresholds are 300 mm apart so that a target near the changeover does not make it flip back and forth, a mode
// change has to be asked for on 3 readings in a row, and there is at least a second between changes because each
// one reconfigures the sensor.
class Auto_mode
{
public:
  static constexpr uint16_t to_short_mm = 900;  // a valid long mode reading below this selects short mode
  static constexpr uint16_t to_long_mm = 1200;  // a valid short mode reading above this selects long mode
  static constexpr uint8_t readings_needed = 3;
  static constexpr uint32_t min_dwell_ms = 1000;

  uint8_t active() const
  {
    return active_mode;
  }

  // Goes back to long mode, as at start
  void reset(uint32_t now_ms)
  {
    active_mode = mode_long;
    votes = 0;
    last_change_ms = now_ms;
  }

  // Feed every reading taken in the active mode. Returns the mode to switch to, or 0 to stay.
  uint8_t update(uint8_t range_status, uint16_t distance_mm, uint32_t now_ms)
  {
    const Reading r = reading_for(range_status);
    bool wants_other;
    bool neutral;
    if (active_mode == mode_long)
    {
      wants_other = r == Reading::too_close || r == Reading::undefined || (r == Reading::valid && distance_mm < to_short_mm);
      neutral = r == Reading::too_far;
    }
    else
    {
      wants_other = r == Reading::too_far || r == Reading::undefined || (r == Reading::valid && distance_mm > to_long_mm);
      neutral = r == Reading::too_close;
    }
    if (wants_other)
    {
      votes = (uint8_t)(votes + 1 > readings_needed ? readings_needed : votes + 1);
    }
    else if (!neutral)
    {
      votes = 0;
    }
    if (votes < readings_needed || (uint32_t)(now_ms - last_change_ms) < min_dwell_ms)
    {
      return 0;
    }
    active_mode = active_mode == mode_long ? mode_short : mode_long;
    votes = 0;
    last_change_ms = now_ms;
    return active_mode;
  }

private:
  uint8_t active_mode{mode_long};
  uint8_t votes{0};
  uint32_t last_change_ms{0};
};
} // namespace Tof_config
