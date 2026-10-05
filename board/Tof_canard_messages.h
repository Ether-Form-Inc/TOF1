// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// Builds the C (libcanard / dronecan_dsdlc) range sensor message from a measurement.

#include <Tof_config.h>

extern "C"
{
#include <uavcan.equipment.range_sensor.Measurement.h>
}

#include <cstring>

namespace Tof_canard
{
// The sensor's field of view with the default 16x16 SPAD region of interest: 27 degrees (full angle)
constexpr float field_of_view_rad = 27.0f * 3.14159265f / 180.0f;

struct Sample
{
  uint8_t range_status; // VL53L1 range status, 0 = valid
  uint16_t distance_mm;
  uint8_t mode; // Tof_config::mode_short or mode_long
};

inline void make_range(uint8_t sensor_id, const Sample &s, const Tof_config::Beam &beam,
                       uavcan_equipment_range_sensor_Measurement &msg)
{
  memset(&msg, 0, sizeof(msg));

  // Network synchronised time is not available, so timestamp stays zero (unknown)
  msg.sensor_id = sensor_id;
  msg.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[0] = beam.roll;
  msg.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[1] = beam.pitch;
  msg.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[2] = beam.yaw;
  msg.beam_orientation_in_body_frame.orientation_defined = true;
  msg.field_of_view = field_of_view_rad;
  msg.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;

  switch (Tof_config::reading_for(s.range_status))
  {
  case Tof_config::Reading::valid:
    msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
    msg.range = (float)s.distance_mm / 1000.0f;
    break;
  case Tof_config::Reading::too_close:
    // The range field then holds the shortest range the sensor has, which the API does not give: 0
    msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;
    msg.range = 0.0f;
    break;
  case Tof_config::Reading::too_far:
    // The range field then holds the longest range the sensor has in its current mode
    msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
    msg.range = (float)Tof_config::max_range_mm(s.mode) / 1000.0f;
    break;
  default:
    msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
    msg.range = 0.0f;
    break;
  }
}
} // namespace Tof_canard
