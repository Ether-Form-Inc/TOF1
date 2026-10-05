// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ether Form Inc.
#pragma once

// DroneCAN time of flight node built on libcanard. Publishes range_sensor.Measurement, NodeStatus at 1 Hz,
// answers GetNodeInfo and offers its settings as parameters. All state is static; nothing is allocated
// outside the libcanard pool.

#include <Stm_fdcan.h>
#include <Stm_settings.h>
#include <Stm_tof.h>
#include <Tof_canard_messages.h>
#include <Tof_config.h>

extern "C"
{
#include <canard.h>
#include <uavcan.equipment.range_sensor.Measurement.h>
#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.RestartNode.h>
#include <uavcan.protocol.param.ExecuteOpcode.h>
#include <uavcan.protocol.param.GetSet.h>
}

#include <cstring>

class Canard_tof_node
{
public:
  Canard_tof_node(uint8_t node_id, uint8_t mode, uint16_t period_ms)
      : node_id(node_id), configured_mode(mode), period_ms(period_ms)
  {
    auto_mode.reset(0);
  }

  // Never returns
  void run()
  {
    fdcan.init();
    canardInit(&canard, pool, sizeof(pool), on_transfer_received, should_accept_transfer, this);
    canardSetLocalNodeID(&canard, node_id);
    tof.init(); // does not wait for the sensor; service_tof() connects to it

    for (;;)
    {
      const uint32_t now_ms = HAL_GetTick();
      service_bus_state(now_ms);
      service_restart(now_ms);
      service_rx(now_ms);
      service_tof(now_ms);
      service_node_status(now_ms);
      service_tx();
      HAL_Delay(1);
    }
  }

private:
  static constexpr uint32_t node_status_period_ms = 1000;
  static constexpr uint32_t cleanup_period_ms = 1000;
  static constexpr uint32_t bus_off_retry_ms = 100;
  static constexpr uint32_t restart_timeout_ms = 500;
  static constexpr uint32_t sensor_retry_ms = 500;
  static constexpr uint32_t sensor_stale_ms = 3000;

  // ---- receive ----
  void service_rx(uint32_t now_ms)
  {
    uint32_t id;
    uint8_t len;
    CanardCANFrame frame;
    // Bounded so a busy bus cannot starve everything else
    for (int i = 0; i < 16; i++)
    {
      memset(&frame, 0, sizeof(frame));
      if (!fdcan.receive(id, frame.data, len))
      {
        break;
      }
      frame.id = id | CANARD_CAN_FRAME_EFF;
      frame.data_len = len;
      canardHandleRxFrame(&canard, &frame, (uint64_t)now_ms * 1000ULL);
    }

    if ((uint32_t)(now_ms - last_cleanup_ms) >= cleanup_period_ms)
    {
      last_cleanup_ms = now_ms;
      canardCleanupStaleTransfers(&canard, (uint64_t)now_ms * 1000ULL);
    }
  }

  static bool should_accept_transfer(const CanardInstance *, uint64_t *out_signature, uint16_t data_type_id,
                                     CanardTransferType transfer_type, uint8_t)
  {
    if (transfer_type != CanardTransferTypeRequest)
    {
      return false;
    }
    switch (data_type_id)
    {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      *out_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      *out_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
      *out_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
      return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      *out_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
      return true;
    default:
      return false;
    }
  }

  static void on_transfer_received(CanardInstance *ins, CanardRxTransfer *transfer)
  {
    Canard_tof_node *self = static_cast<Canard_tof_node *>(canardGetUserReference(ins));
    if (transfer->transfer_type != CanardTransferTypeRequest)
    {
      return;
    }
    switch (transfer->data_type_id)
    {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
      self->send_node_info(transfer);
      break;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
      self->handle_param_getset(transfer);
      break;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
      self->handle_param_execute_opcode(transfer);
      break;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
      self->handle_restart_node(transfer);
      break;
    default:
      break;
    }
  }

  // ---- parameters ----
  //
  // Three integer parameters, each stored in its own flash page (see Stm_settings.h). For all of them 0 means
  // "not set": the default applies (for NODE_ID that is the ID selected by the jumpers).
  //   NODE_ID         0 to 127. Used from the next start.
  //   DIST_MODE       0 (default, long), 1 short, 2 long, 3 auto (see Tof_config::Auto_mode). Applied at once.
  //   MEAS_PERIOD_MS  0 (default, 100) or 25 to 250. Applied at once.
  struct Param
  {
    const char *name;
    const Stm_settings::Setting &setting;
    int64_t max;
    bool (*valid)(int64_t);
  };

  static bool valid_node_id(int64_t v)
  {
    return v >= 0 && v <= 127;
  }
  static bool valid_mode(int64_t v)
  {
    return v >= 0 && v <= Tof_config::mode_auto;
  }
  static bool valid_period(int64_t v)
  {
    return v == 0 || (v >= Tof_config::min_period_ms && v <= Tof_config::max_period_ms);
  }

  static constexpr int param_count = 3;
  static const Param &param(int i)
  {
    static const Param params[param_count] = {
        {"NODE_ID", Stm_settings::node_id, 127, valid_node_id},
        {"DIST_MODE", Stm_settings::distance_mode, Tof_config::mode_auto, valid_mode},
        {"MEAS_PERIOD_MS", Stm_settings::update_rate, Tof_config::max_period_ms, valid_period},
    };
    return params[i];
  }

  static int find_param(const uint8_t *name, uint8_t len)
  {
    for (int i = 0; i < param_count; i++)
    {
      if (len == strlen(param(i).name) && memcmp(name, param(i).name, len) == 0)
      {
        return i;
      }
    }
    return -1;
  }

  static int64_t stored(const Param &p)
  {
    uint8_t v;
    return Stm_settings::read(p.setting, v) ? v : 0;
  }

  void send_service_response(CanardRxTransfer *request, uint64_t signature, uint8_t type_id, const uint8_t *buf,
                             uint32_t len)
  {
    canardRequestOrRespond(&canard, request->source_node_id, signature, type_id, &request->transfer_id,
                           request->priority, CanardResponse, buf, (uint16_t)len);
  }

  void handle_param_getset(CanardRxTransfer *request)
  {
    static uavcan_protocol_param_GetSetRequest req;
    static uavcan_protocol_param_GetSetResponse res;
    static uint8_t buf[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];

    memset(&req, 0, sizeof(req));
    const bool decode_failed = uavcan_protocol_param_GetSetRequest_decode(request, &req);
    canardReleaseRxTransferPayload(&canard, request); // give the pool blocks back before replying
    if (decode_failed)
    {
      return;
    }

    // Find the parameter by name if one is given, otherwise by index
    const int found = req.name.len > 0 ? find_param(req.name.data, req.name.len)
                                       : (req.index < param_count ? (int)req.index : -1);

    memset(&res, 0, sizeof(res));
    if (found >= 0)
    {
      const Param &p = param(found);
      if (req.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE)
      {
        const int64_t wanted = req.value.integer_value;
        // An invalid value is ignored; the response shows what is actually stored
        if (p.valid(wanted) && wanted != stored(p))
        {
          if (wanted == 0)
          {
            Stm_settings::erase(p.setting);
          }
          else
          {
            Stm_settings::write(p.setting, (uint8_t)wanted);
          }
          if (found != 0)
          {
            reconfigure_pending = true; // the sensor settings take effect at once
          }
        }
      }
      res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
      res.value.integer_value = stored(p);
      res.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
      res.default_value.integer_value = 0;
      res.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
      res.max_value.integer_value = p.max;
      res.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
      res.min_value.integer_value = 0;
      res.name.len = (uint8_t)strlen(p.name);
      memcpy(res.name.data, p.name, res.name.len);
    }
    // A parameter that does not exist gets an empty response, which ends a listing by index

    memset(buf, 0, sizeof(buf));
    const uint32_t len = uavcan_protocol_param_GetSetResponse_encode(&res, buf);
    send_service_response(request, UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE, UAVCAN_PROTOCOL_PARAM_GETSET_ID, buf, len);
  }

  void handle_param_execute_opcode(CanardRxTransfer *request)
  {
    uavcan_protocol_param_ExecuteOpcodeRequest req = {};
    const bool decode_failed = uavcan_protocol_param_ExecuteOpcodeRequest_decode(request, &req);
    canardReleaseRxTransferPayload(&canard, request);
    if (decode_failed)
    {
      return;
    }

    uavcan_protocol_param_ExecuteOpcodeResponse res = {};
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
    {
      res.ok = true; // GetSet already stored the values in flash
    }
    else if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
    {
      // Back to the defaults: the jumper selected ID, long mode, 10 Hz
      res.ok = true;
      for (int i = 0; i < param_count; i++)
      {
        res.ok = Stm_settings::erase(param(i).setting) && res.ok;
      }
      reconfigure_pending = true;
    }

    uint8_t buf[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE] = {};
    const uint32_t len = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&res, buf);
    send_service_response(request, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                          UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID, buf, len);
  }

  void handle_restart_node(CanardRxTransfer *request)
  {
    uavcan_protocol_RestartNodeRequest req = {};
    const bool decode_failed = uavcan_protocol_RestartNodeRequest_decode(request, &req);
    canardReleaseRxTransferPayload(&canard, request);
    if (decode_failed)
    {
      return;
    }

    uavcan_protocol_RestartNodeResponse res = {};
    res.ok = req.magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;
    if (res.ok)
    {
      restart_requested = true; // service_restart() resets once the reply has been sent
      restart_requested_ms = HAL_GetTick();
    }

    uint8_t buf[UAVCAN_PROTOCOL_RESTARTNODE_RESPONSE_MAX_SIZE] = {};
    const uint32_t len = uavcan_protocol_RestartNodeResponse_encode(&res, buf);
    send_service_response(request, UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE, UAVCAN_PROTOCOL_RESTARTNODE_ID, buf, len);
  }

  // Resets the MCU after a RestartNode request, once the reply has left the CAN controller
  void service_restart(uint32_t now_ms)
  {
    if (!restart_requested)
    {
      return;
    }
    const bool sent = canardPeekTxQueue(&canard) == nullptr && fdcan.tx_idle();
    if (sent || (uint32_t)(now_ms - restart_requested_ms) > restart_timeout_ms)
    {
      HAL_Delay(5); // let the last frame finish on the bus
      NVIC_SystemReset();
    }
  }

  // Vendor specific status codes in NodeStatus
  static constexpr uint16_t status_ok = 0;
  static constexpr uint16_t status_sensor_not_found = 1; // the VL53L1CB never answered on I2C
  static constexpr uint16_t status_no_data = 2;          // the sensor has stopped producing measurements

  void fill_node_status(uavcan_protocol_NodeStatus &status)
  {
    const uint32_t now_ms = HAL_GetTick();
    status.uptime_sec = now_ms / 1000;
    status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    status.sub_mode = sensor_mode(); // the distance mode in use: 1 short, 2 long (in auto mode, the one it chose)

    // An out of range target is not a fault. A sensor that is missing, or silent, is.
    if (!sensor_found)
    {
      status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR;
      status.vendor_specific_status_code = status_sensor_not_found;
    }
    else if ((uint32_t)(now_ms - last_measurement_ms) > sensor_stale_ms)
    {
      status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_WARNING;
      status.vendor_specific_status_code = status_no_data;
    }
    else
    {
      status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
      status.vendor_specific_status_code = status_ok;
    }
  }

  void send_node_info(CanardRxTransfer *request)
  {
    static uavcan_protocol_GetNodeInfoResponse res;
    static uint8_t buf[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    memset(&res, 0, sizeof(res));
    memset(buf, 0, sizeof(buf));

    fill_node_status(res.status);
    res.software_version.major = 1;
    res.software_version.minor = 0;
    res.hardware_version.major = 1;
    res.hardware_version.minor = 0;

    // 96 bit STM32 unique ID, zero padded to the 128 bits DroneCAN wants
    const uint32_t *uid = reinterpret_cast<const uint32_t *>(0x1FFF7590UL);
    memcpy(res.hardware_version.unique_id, uid, 12);

    static const char name[] = "CAN ToF Sensor";
    res.name.len = sizeof(name) - 1;
    memcpy(res.name.data, name, res.name.len);

    const uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&res, buf
#if CANARD_ENABLE_TAO_OPTION
                                                                   , false
#endif
    );
    canardRequestOrRespond(&canard, request->source_node_id, UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID, &request->transfer_id, request->priority,
                           CanardResponse, buf, (uint16_t)len);
  }

  // ---- transmit ----

  // Recovers from bus-off. Frames queued while the controller cannot transmit are dropped:
  // replaying old measurements in a burst once the bus returns would be worse than losing them.
  void service_bus_state(uint32_t now_ms)
  {
    if (!fdcan.bus_off())
    {
      bus_off_seen = false;
      return;
    }

    while (canardPeekTxQueue(&canard) != nullptr)
    {
      canardPopTxQueue(&canard);
    }

    if (!bus_off_seen)
    {
      bus_off_seen = true;
      last_bus_off_action_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - last_bus_off_action_ms) >= bus_off_retry_ms)
    {
      fdcan.restart_after_bus_off();
      last_bus_off_action_ms = now_ms;
    }
  }

  void service_tx()
  {
    // The hardware FIFO only holds 3 frames, so feed it as it drains
    while (fdcan.tx_has_room())
    {
      const CanardCANFrame *frame = canardPeekTxQueue(&canard);
      if (frame == nullptr)
      {
        break;
      }
      if (!fdcan.send(frame->id, frame->data, frame->data_len))
      {
        break;
      }
      canardPopTxQueue(&canard);
    }
  }

  void service_node_status(uint32_t now_ms)
  {
    if ((uint32_t)(now_ms - last_node_status_ms) < node_status_period_ms)
    {
      return;
    }
    last_node_status_ms = now_ms;

    uavcan_protocol_NodeStatus status;
    uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    memset(buf, 0, sizeof(buf));
    fill_node_status(status);
    const uint32_t len = uavcan_protocol_NodeStatus_encode(&status, buf
#if CANARD_ENABLE_TAO_OPTION
                                                          , false
#endif
    );
    canardBroadcast(&canard, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &node_status_transfer_id, CANARD_TRANSFER_PRIORITY_LOW, buf, (uint16_t)len);
  }

  // ---- sensor ----

  // Reads the stored settings again (after a parameter change)
  void reload_settings()
  {
    uint8_t v;
    configured_mode = Tof_config::resolve_mode(Stm_settings::read(Stm_settings::distance_mode, v), v);
    period_ms = Tof_config::resolve_period_ms(Stm_settings::read(Stm_settings::update_rate, v), v);
    auto_mode.reset(HAL_GetTick());
  }

  // The mode the sensor runs in: the configured one, or whichever of short and long auto mode has chosen
  uint8_t sensor_mode() const
  {
    return configured_mode == Tof_config::mode_auto ? auto_mode.active() : configured_mode;
  }

  void service_tof(uint32_t now_ms)
  {
    // The sensor may be slow to start, or absent; keep trying without blocking the CAN node for long
    if (!sensor_found)
    {
      if ((uint32_t)(now_ms - last_connect_try_ms) >= sensor_retry_ms)
      {
        last_connect_try_ms = now_ms;
        if (tof.connect(sensor_mode(), period_ms))
        {
          sensor_found = true;
          last_measurement_ms = HAL_GetTick();
          skip_readings = 2; // the first readings after a start are unreliable
        }
      }
      return;
    }

    if (reconfigure_pending)
    {
      reconfigure_pending = false;
      reload_settings();
      if (!tof.reconfigure(sensor_mode(), period_ms))
      {
        sensor_found = false; // start over with a reset of the sensor
      }
      last_measurement_ms = HAL_GetTick();
      return;
    }

    // A sensor that stops measuring is reset and set up again
    if ((uint32_t)(now_ms - last_measurement_ms) > sensor_stale_ms)
    {
      sensor_found = false;
      last_connect_try_ms = now_ms - sensor_retry_ms;
      return;
    }

    if ((uint32_t)(now_ms - last_poll_ms) < poll_period_ms)
    {
      return;
    }
    last_poll_ms = now_ms;

    Tof_canard::Sample sample;
    if (!tof.read(sample))
    {
      return;
    }
    last_measurement_ms = now_ms;

    // The first readings after a start or a change of mode are not trusted, and not sent
    if (skip_readings > 0)
    {
      skip_readings--;
      return;
    }
    const uint8_t new_mode = configured_mode == Tof_config::mode_auto
                                 ? auto_mode.update(sample.range_status, sample.distance_mm, now_ms)
                                 : 0;

    static uavcan_equipment_range_sensor_Measurement msg;
    static uint8_t buf[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE];
    Tof_canard::make_range(node_id, sample, msg);
    memset(buf, 0, sizeof(buf));
    const uint32_t len = uavcan_equipment_range_sensor_Measurement_encode(&msg, buf
#if CANARD_ENABLE_TAO_OPTION
                                                                         , false
#endif
    );
    canardBroadcast(&canard, UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                    UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID, &range_transfer_id, CANARD_TRANSFER_PRIORITY_MEDIUM,
                    buf, (uint16_t)len);

    if (new_mode != 0)
    {
      if (tof.reconfigure(new_mode, period_ms))
      {
        skip_readings = 1;
      }
      else
      {
        sensor_found = false; // start over with a reset of the sensor
      }
      last_measurement_ms = HAL_GetTick();
    }
  }

  static constexpr uint32_t poll_period_ms = 2;

  uint8_t node_id;
  uint8_t configured_mode; // DIST_MODE as stored: 1 short, 2 long, 3 auto
  Tof_config::Auto_mode auto_mode;
  uint8_t skip_readings{0};
  uint16_t period_ms;
  Stm_fdcan fdcan;
  Stm_tof tof;

  CanardInstance canard;
  alignas(8) uint8_t pool[1024];

  uint8_t node_status_transfer_id{0};
  uint8_t range_transfer_id{0};

  uint32_t last_poll_ms{0};
  bool sensor_found{false};
  uint32_t last_measurement_ms{0};
  uint32_t last_connect_try_ms{0};
  bool reconfigure_pending{false};
  uint32_t last_node_status_ms{0};
  uint32_t last_cleanup_ms{0};
  uint32_t last_bus_off_action_ms{0};
  bool bus_off_seen{false};
  bool restart_requested{false};
  uint32_t restart_requested_ms{0};
};
