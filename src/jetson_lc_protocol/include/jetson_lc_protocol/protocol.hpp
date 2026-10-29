#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace jetson_lc_protocol
{
constexpr std::uint8_t kHeaderByte = 0xA5;
constexpr std::uint8_t kFunctionConfiguration = 0x10;
constexpr std::uint8_t kFunctionStatus = 0x11;
constexpr std::uint8_t kFunctionCommand = 0x12;
constexpr std::size_t kConfigurationDataSize = 32;
constexpr std::size_t kStatusDataSize = 4;

struct Frame
{
  std::uint8_t lift_id{};
  std::uint8_t function_code{};
  std::vector<std::uint8_t> data;
};

enum class ManualMotion : std::uint8_t { Stop = 0, Up = 1, Down = 2 };
enum class MaintenanceRequest : std::uint8_t { None = 0, StartHold = 1, Abort = 2 };

enum class SessionState : std::uint8_t
{
  Idle = 0x00, PreparationRequestReceived = 0x01, CheckingReadiness = 0x02,
  NotAvailable = 0x03, EnteringRobotMaintenanceMode = 0x04,
  MovingToTopServiceFloor = 0x05, ReadyForJetsonControl = 0x06,
  CompletionRequestReceived = 0x07, ReturningToAutoMode = 0x08,
  ServiceComplete = 0x09, Failed = 0x0A, Aborted = 0x0B
};

enum class AbnormalReason : std::uint8_t
{
  None = 0x00, LiftNotAvailable = 0x01, DoorNotClosed = 0x02,
  PassengerServiceConflict = 0x03, LiftAbnormalMode = 0x04,
  FailedToEnterRobotMaintenanceMode = 0x05, FailedToReturnToAutoMode = 0x06,
  Timeout = 0x07, SafetyInterlock = 0x08, InternalLcError = 0x09,
  ManualTechnicianIntervention = 0x0A, AbortRequestedByJetson = 0x0B,
  JetsonCommunicationLost = 0x0C
};

struct CommandState
{
  std::uint8_t destination_floor{0};
  bool hall_call_disable{false};
  ManualMotion manual_motion{ManualMotion::Stop};
  MaintenanceRequest maintenance_request{MaintenanceRequest::None};
};

struct Configuration
{
  std::uint16_t rated_capacity_kg{};
  std::uint16_t car_width_mm{};
  std::uint16_t distance_mm{};
  std::uint16_t door_width_mm{};
  bool has_second_door{};
  std::uint8_t second_door_position{};
  std::vector<std::uint8_t> service_floor_configuration;

  bool operator==(const Configuration & other) const;
  bool operator!=(const Configuration & other) const {return !(*this == other);}
};

struct Status
{
  std::uint8_t current_floor{};
  bool jetson_operation_permitted{};
  bool arrived_at_destination{};
  bool robot_maintenance_mode{};
  bool lift_abnormal_mode{};
  SessionState session_state{SessionState::Idle};
  AbnormalReason abnormal_reason{AbnormalReason::None};
};

std::vector<std::uint8_t> encode_frame(const Frame & frame);
std::vector<std::uint8_t> encode_poll_request(std::uint8_t lift_id, std::uint8_t function_code);
bool validate_command(const CommandState & command, std::string * reason = nullptr);
std::vector<std::uint8_t> encode_command_data(const CommandState & command);
std::vector<std::uint8_t> encode_command_request(std::uint8_t lift_id, const CommandState & command);
std::optional<Configuration> decode_configuration(const Frame & frame);
std::optional<Status> decode_status(const Frame & frame);
bool is_command_ack(const Frame & frame);
bool matches_response(const Frame & frame, std::uint8_t lift_id, std::uint8_t function_code);

struct ParseResult
{
  std::vector<Frame> frames;
  std::uint32_t crc_errors{};
  std::uint32_t malformed_frames{};
};

class FrameParser
{
public:
  ParseResult append(const std::uint8_t * data, std::size_t size);
  ParseResult append(const std::vector<std::uint8_t> & bytes)
  {return append(bytes.data(), bytes.size());}
  void clear() {buffer_.clear();}

private:
  std::vector<std::uint8_t> buffer_;
};
}
