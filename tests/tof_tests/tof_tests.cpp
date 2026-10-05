// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#include <Tof_canard_messages.h>
#include <Tof_config.h>

#include <gtest/gtest.h>

#include <cmath>

using namespace Tof_config;

TEST(Node_id, jumpers_add_to_the_offset)
{
  EXPECT_EQ(resolve_node_id(0, false, 0xFF), 32);
  EXPECT_EQ(resolve_node_id(1, false, 0xFF), 33);
  EXPECT_EQ(resolve_node_id(2, false, 0xFF), 34);
  EXPECT_EQ(resolve_node_id(4, false, 0xFF), 36);
  EXPECT_EQ(resolve_node_id(8, false, 0xFF), 40);
  EXPECT_EQ(resolve_node_id(15, false, 0xFF), 47);
}

TEST(Node_id, stored_id_overrides_the_jumpers_and_has_no_offset)
{
  EXPECT_EQ(resolve_node_id(0, true, 42), 42);
  EXPECT_EQ(resolve_node_id(15, true, 1), 1);
  EXPECT_EQ(resolve_node_id(0, true, 127), 127);
}

TEST(Node_id, invalid_stored_id_is_ignored)
{
  EXPECT_EQ(resolve_node_id(3, true, 0), 35);   // 0 is reserved in DroneCAN
  EXPECT_EQ(resolve_node_id(3, true, 128), 35);
  EXPECT_EQ(resolve_node_id(3, true, 0xFF), 35);
}

TEST(Settings, defaults_when_not_stored_or_invalid)
{
  EXPECT_EQ(resolve_mode(false, 0), mode_long);
  EXPECT_EQ(resolve_mode(true, 0), mode_long);
  EXPECT_EQ(resolve_mode(true, 3), mode_auto);
  EXPECT_EQ(resolve_mode(true, 4), mode_long);
  EXPECT_EQ(resolve_mode(true, 1), mode_short);
  EXPECT_EQ(resolve_mode(true, 2), mode_long);
  EXPECT_EQ(resolve_period_ms(false, 0), 100);
  EXPECT_EQ(resolve_period_ms(true, 24), 100);
  EXPECT_EQ(resolve_period_ms(true, 25), 25);
  EXPECT_EQ(resolve_period_ms(true, 250), 250);
}

TEST(Timing_budget, largest_allowed_value_within_80_percent_of_the_period)
{
  EXPECT_EQ(timing_budget_ms(25), 20);
  EXPECT_EQ(timing_budget_ms(41), 20);
  EXPECT_EQ(timing_budget_ms(42), 33);
  EXPECT_EQ(timing_budget_ms(62), 33);
  EXPECT_EQ(timing_budget_ms(63), 50);
  EXPECT_EQ(timing_budget_ms(100), 50);
  EXPECT_EQ(timing_budget_ms(125), 100);
  EXPECT_EQ(timing_budget_ms(124), 50);
  EXPECT_EQ(timing_budget_ms(249), 100);
  EXPECT_EQ(timing_budget_ms(250), 200);
}

TEST(Timing_budget, never_longer_than_the_period)
{
  for (uint16_t p = min_period_ms; p <= max_period_ms; p++)
  {
    EXPECT_LE(timing_budget_ms(p), p);
    if (p >= 25)
    {
      EXPECT_LE(timing_budget_ms(p) * 5u, p * 4u + (p < 42 ? 80u : 0u)); // 80%, except where only the 20 ms budget fits
    }
  }
}

TEST(Reading, range_status_mapping)
{
  EXPECT_EQ(reading_for(0), Reading::valid);
  EXPECT_EQ(reading_for(3), Reading::too_close);
  EXPECT_EQ(reading_for(1), Reading::too_far);
  EXPECT_EQ(reading_for(2), Reading::too_far);
  EXPECT_EQ(reading_for(4), Reading::too_far);
  EXPECT_EQ(reading_for(5), Reading::undefined);
  EXPECT_EQ(reading_for(7), Reading::undefined);
  EXPECT_EQ(reading_for(255), Reading::undefined);
}

TEST(Range_message, valid_reading)
{
  uavcan_equipment_range_sensor_Measurement msg;
  Tof_canard::make_range(35, {0, 1234, mode_long}, beam_for(orient_down), msg);
  EXPECT_EQ(msg.sensor_id, 35);
  EXPECT_EQ(msg.sensor_type, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR);
  EXPECT_EQ(msg.reading_type, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE);
  EXPECT_NEAR(msg.range, 1.234f, 1e-5);
  EXPECT_NEAR(msg.field_of_view, 0.4712f, 1e-3);
  EXPECT_TRUE(msg.beam_orientation_in_body_frame.orientation_defined);
  EXPECT_EQ(msg.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[1], -6); // pointing down
  EXPECT_EQ(msg.timestamp.usec, 0u);
}

TEST(Range_message, too_far_reports_the_longest_range_of_the_mode)
{
  uavcan_equipment_range_sensor_Measurement msg;
  Tof_canard::make_range(32, {2, 0, mode_long}, beam_for(orient_down), msg);
  EXPECT_EQ(msg.reading_type, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR);
  EXPECT_NEAR(msg.range, 4.0f, 1e-5);
  Tof_canard::make_range(32, {4, 0, mode_short}, beam_for(orient_down), msg);
  EXPECT_NEAR(msg.range, 1.36f, 1e-5);
}

TEST(Range_message, too_close_reading)
{
  uavcan_equipment_range_sensor_Measurement msg;
  Tof_canard::make_range(32, {3, 5, mode_long}, beam_for(orient_down), msg);
  EXPECT_EQ(msg.reading_type, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE);
  EXPECT_EQ(msg.range, 0.0f);
}

TEST(Range_message, undefined_reading_has_no_range)
{
  uavcan_equipment_range_sensor_Measurement msg;
  Tof_canard::make_range(32, {7, 999, mode_long}, beam_for(orient_down), msg);
  EXPECT_EQ(msg.reading_type, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED);
  EXPECT_EQ(msg.range, 0.0f);
}

// The generated encoder must link and produce a payload that fits the message
TEST(Range_message, encodes_within_maximum_size)
{
  uavcan_equipment_range_sensor_Measurement msg;
  Tof_canard::make_range(32, {0, 1000, mode_long}, beam_for(orient_down), msg);
  uint8_t buf[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE] = {};
  const uint32_t len = uavcan_equipment_range_sensor_Measurement_encode(&msg, buf);
  EXPECT_EQ(len, 15u);
  EXPECT_LE(len, (uint32_t)UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE);

  uavcan_equipment_range_sensor_Measurement back = {};
  CanardRxTransfer transfer = {};
  transfer.payload_head = buf;
  transfer.payload_len = (uint16_t)len;
  EXPECT_FALSE(uavcan_equipment_range_sensor_Measurement_decode(&transfer, &back));
  EXPECT_NEAR(back.range, 1.0f, 1e-3);
  EXPECT_EQ(back.reading_type, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE);
  EXPECT_EQ(back.sensor_type, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR);
}

TEST(Auto_mode, starts_in_long_and_stays_for_a_far_target)
{
  Auto_mode a;
  a.reset(0);
  EXPECT_EQ(a.active(), mode_long);
  for (uint32_t t = 100; t < 10000; t += 100)
  {
    EXPECT_EQ(a.update(0, 2500, t), 0);
  }
  EXPECT_EQ(a.active(), mode_long);
}

TEST(Auto_mode, near_target_or_wrap_around_selects_short_after_three_readings)
{
  Auto_mode a;
  a.reset(0);
  EXPECT_EQ(a.update(7, 0, 2000), 0); // wrap-around, as seen for a 20 cm target in long mode
  EXPECT_EQ(a.update(7, 0, 2100), 0);
  EXPECT_EQ(a.update(7, 0, 2200), mode_short);
  EXPECT_EQ(a.active(), mode_short);
}

TEST(Auto_mode, a_good_reading_resets_the_count)
{
  Auto_mode a;
  a.reset(0);
  EXPECT_EQ(a.update(0, 500, 2000), 0);
  EXPECT_EQ(a.update(0, 500, 2100), 0);
  EXPECT_EQ(a.update(0, 2000, 2200), 0); // not near: starts over
  EXPECT_EQ(a.update(0, 500, 2300), 0);
  EXPECT_EQ(a.update(0, 500, 2400), 0);
  EXPECT_EQ(a.update(0, 500, 2500), mode_short);
}

TEST(Auto_mode, short_goes_back_to_long_when_out_of_range)
{
  Auto_mode a;
  a.reset(0);
  a.update(3, 0, 2000);
  a.update(3, 0, 2100);
  ASSERT_EQ(a.update(3, 0, 2200), mode_short);
  EXPECT_EQ(a.update(2, 0, 4000), 0);
  EXPECT_EQ(a.update(2, 0, 4100), 0);
  EXPECT_EQ(a.update(2, 0, 4200), mode_long);
}

TEST(Auto_mode, hysteresis_band_between_the_thresholds_keeps_the_mode)
{
  Auto_mode a;
  a.reset(0);
  // A target at 1.0 m is between 900 and 1200 mm: long mode keeps it, and so would short mode
  for (uint32_t t = 2000; t < 8000; t += 100)
  {
    EXPECT_EQ(a.update(0, 1000, t), 0);
  }
  Auto_mode b;
  b.reset(0);
  b.update(7, 0, 2000);
  b.update(7, 0, 2100);
  ASSERT_EQ(b.update(7, 0, 2200), mode_short);
  for (uint32_t t = 4000; t < 9000; t += 100)
  {
    EXPECT_EQ(b.update(0, 1000, t), 0);
  }
}

TEST(Auto_mode, changes_are_at_least_a_second_apart)
{
  Auto_mode a;
  a.reset(0);
  a.update(7, 0, 100);
  a.update(7, 0, 200);
  EXPECT_EQ(a.update(7, 0, 300), 0); // three votes but only 300 ms since the start
  EXPECT_EQ(a.update(7, 0, 1000), mode_short);
  a.update(2, 0, 1100);
  a.update(2, 0, 1200);
  EXPECT_EQ(a.update(2, 0, 1300), 0); // too soon after the last change
  EXPECT_EQ(a.update(2, 0, 2000), mode_long);
}

TEST(Auto_mode, short_goes_back_to_long_when_the_target_leaves_with_wrap_around)
{
  // What was seen on the bench: valid, then undefined (wrap-around) readings once the target moved out of range
  Auto_mode a;
  a.reset(0);
  a.update(3, 0, 2000);
  a.update(3, 0, 2100);
  ASSERT_EQ(a.update(3, 0, 2200), mode_short);
  EXPECT_EQ(a.update(0, 400, 3500), 0);
  EXPECT_EQ(a.update(7, 300, 3600), 0);
  EXPECT_EQ(a.update(7, 300, 3700), 0);
  EXPECT_EQ(a.update(7, 300, 3800), mode_long);
}

TEST(Auto_mode, neutral_readings_do_not_reset_the_count)
{
  Auto_mode a;
  a.reset(0);
  EXPECT_EQ(a.update(0, 400, 2000), 0);
  EXPECT_EQ(a.update(2, 0, 2100), 0);   // too far in long mode says nothing about a near target
  EXPECT_EQ(a.update(0, 410, 2200), 0);
  EXPECT_EQ(a.update(2, 0, 2300), 0);
  EXPECT_EQ(a.update(0, 405, 2400), mode_short);
  // and too close in short mode says nothing about a far target
  EXPECT_EQ(a.update(2, 0, 4000), 0);
  EXPECT_EQ(a.update(3, 0, 4100), 0);
  EXPECT_EQ(a.update(2, 0, 4200), 0);
  EXPECT_EQ(a.update(3, 0, 4300), 0);
  EXPECT_EQ(a.update(2, 0, 4400), mode_long);
}

TEST(Orientation, default_is_down_and_invalid_values_fall_back_to_it)
{
  EXPECT_EQ(resolve_orientation(false, 0), orient_down);
  EXPECT_EQ(resolve_orientation(true, 0), orient_down);
  EXPECT_EQ(resolve_orientation(true, 11), orient_down);
  EXPECT_EQ(resolve_orientation(true, 2), orient_up);
  EXPECT_EQ(resolve_orientation(true, 10), 10);
}

TEST(Orientation, down_and_up_are_pitched_by_plus_or_minus_90_degrees)
{
  const Beam down = beam_for(orient_down);
  EXPECT_EQ(down.roll, 0);
  EXPECT_EQ(down.pitch, -6);
  EXPECT_EQ(down.yaw, 0);
  EXPECT_EQ(beam_for(orient_up).pitch, 6);
}

TEST(Orientation, horizontal_beams_turn_in_45_degree_steps_and_stay_in_range)
{
  const int8_t expected_yaw[8] = {0, 3, 6, 9, 12, 15, -6, -3}; // 0, 45, 90, 135, 180, 225, 270, 315 degrees
  for (int n = 0; n < 8; n++)
  {
    const Beam b = beam_for((uint8_t)(orient_forward + n));
    EXPECT_EQ(b.pitch, 0);
    EXPECT_EQ(b.yaw, expected_yaw[n]);
    EXPECT_GE(b.yaw, -16); // the coarse orientation holds 5 bit signed values
    EXPECT_LE(b.yaw, 15);
  }
}

// What PX4 does with the message: it multiplies by pi / 12 and checks sin(pitch)
TEST(Orientation, encoded_down_beam_is_what_px4_recognises_as_downward_facing)
{
  uavcan_equipment_range_sensor_Measurement msg;
  Tof_canard::make_range(32, {0, 1000, mode_long}, beam_for(orient_down), msg);
  uint8_t buf[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE] = {};
  const uint32_t len = uavcan_equipment_range_sensor_Measurement_encode(&msg, buf);
  uavcan_equipment_range_sensor_Measurement back = {};
  CanardRxTransfer transfer = {};
  transfer.payload_head = buf;
  transfer.payload_len = (uint16_t)len;
  ASSERT_FALSE(uavcan_equipment_range_sensor_Measurement_decode(&transfer, &back));
  EXPECT_TRUE(back.beam_orientation_in_body_frame.orientation_defined);
  const float pitch = back.beam_orientation_in_body_frame.fixed_axis_roll_pitch_yaw[1] * (3.14159265f / 12.0f);
  EXPECT_GT(-sinf(pitch), 1.0f - 1e-5f);
}
