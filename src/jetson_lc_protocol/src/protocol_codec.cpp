#include "jetson_lc_protocol/protocol.hpp"

#include <stdexcept>
#include <tuple>

namespace jetson_lc_protocol
{
namespace
{
std::uint16_t be16(const std::vector<std::uint8_t> & bytes, const std::size_t offset)
{
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(bytes[offset]) << 8U) |
         bytes[offset + 1U]);
}
}

bool Configuration::operator==(const Configuration & other) const
{
  return std::tie(rated_capacity_kg, car_width_mm, distance_mm, door_width_mm,
    has_second_door, second_door_position, service_floor_configuration) ==
    std::tie(other.rated_capacity_kg, other.car_width_mm, other.distance_mm,
    other.door_width_mm, other.has_second_door, other.second_door_position,
    other.service_floor_configuration);
}

bool validate_command(const CommandState & command, std::string * const reason)
{
  auto fail = [reason](const char * text) {if (reason) {*reason = text;} return false;};
  if (command.destination_floor != 0U) {
    return fail("nonzero destination floors are not supported in Phase 1");
  }
  if (command.manual_motion != ManualMotion::Stop &&
    command.manual_motion != ManualMotion::Up && command.manual_motion != ManualMotion::Down)
  {
    return fail("invalid manual motion");
  }
  if (command.maintenance_request != MaintenanceRequest::None &&
    command.maintenance_request != MaintenanceRequest::StartHold &&
    command.maintenance_request != MaintenanceRequest::Abort)
  {
    return fail("invalid maintenance request");
  }
  if (command.manual_motion != ManualMotion::Stop &&
    command.maintenance_request != MaintenanceRequest::StartHold)
  {
    return fail("manual motion requires REQUEST_START_HOLD");
  }
  return true;
}

std::vector<std::uint8_t> encode_command_data(const CommandState & command)
{
  std::string reason;
  if (!validate_command(command, &reason)) {throw std::invalid_argument(reason);}
  std::uint8_t bits = command.hall_call_disable ? 0x04U : 0U;
  if (command.manual_motion == ManualMotion::Up) {bits |= 0x08U;}
  if (command.manual_motion == ManualMotion::Down) {bits |= 0x10U;}
  std::uint8_t type = 0;
  if (command.maintenance_request == MaintenanceRequest::StartHold) {bits |= 0x20U; type = 1;}
  if (command.maintenance_request == MaintenanceRequest::Abort) {bits |= 0x20U; type = 2;}
  return {command.destination_floor, bits, type};
}

std::vector<std::uint8_t> encode_command_request(
  const std::uint8_t lift_id, const CommandState & command)
{
  return encode_frame(Frame{lift_id, kFunctionCommand, encode_command_data(command)});
}

std::optional<Configuration> decode_configuration(const Frame & frame)
{
  if (frame.function_code != kFunctionConfiguration || frame.data.size() != kConfigurationDataSize) {
    return std::nullopt;
  }
  Configuration result;
  result.rated_capacity_kg = be16(frame.data, 0);
  result.car_width_mm = be16(frame.data, 2);
  result.distance_mm = be16(frame.data, 4);
  result.door_width_mm = be16(frame.data, 6);
  result.has_second_door = frame.data[8] != 0U;
  result.second_door_position = frame.data[9];
  result.service_floor_configuration.assign(frame.data.begin() + 10, frame.data.begin() + 26);
  return result;
}

std::optional<Status> decode_status(const Frame & frame)
{
  if (frame.function_code != kFunctionStatus || frame.data.size() != kStatusDataSize) {
    return std::nullopt;
  }
  const auto bits = frame.data[1];
  return Status{frame.data[0], (bits & 0x01U) != 0U, (bits & 0x02U) != 0U,
    (bits & 0x10U) != 0U, (bits & 0x20U) != 0U,
    static_cast<SessionState>(frame.data[2]), static_cast<AbnormalReason>(frame.data[3])};
}

bool is_command_ack(const Frame & frame)
{
  return frame.function_code == kFunctionCommand && frame.data == std::vector<std::uint8_t>{0};
}

bool matches_response(
  const Frame & frame, const std::uint8_t lift_id, const std::uint8_t function_code)
{
  return frame.lift_id == lift_id && frame.function_code == function_code;
}
}
