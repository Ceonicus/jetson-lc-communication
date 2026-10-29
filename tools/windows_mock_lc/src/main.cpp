#include "jetson_lc_protocol/protocol.hpp"
#include "jetson_lc_tools/mock_lc_logic.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace
{
constexpr std::uint8_t kLiftId = 1;
constexpr DWORD kIdleReadTimeoutMs = 20;
std::atomic_bool g_running{true};
using SteadyClock = std::chrono::steady_clock;

struct Options
{
  std::string port;
  DWORD baud{38400};
  unsigned int delay_ms{};
  unsigned int drop_every{};
  unsigned int bad_crc_every{};
};

class SerialHandle
{
public:
  explicit SerialHandle(HANDLE handle) : handle_(handle) {}
  ~SerialHandle()
  {
    if (handle_ != INVALID_HANDLE_VALUE) {
      CloseHandle(handle_);
    }
  }
  SerialHandle(const SerialHandle &) = delete;
  SerialHandle & operator=(const SerialHandle &) = delete;
  SerialHandle(SerialHandle && other) noexcept : handle_(other.handle_)
  {
    other.handle_ = INVALID_HANDLE_VALUE;
  }
  SerialHandle & operator=(SerialHandle &&) = delete;
  HANDLE get() const noexcept {return handle_;}

private:
  HANDLE handle_{INVALID_HANDLE_VALUE};
};

std::string hex_bytes(const std::vector<std::uint8_t> & bytes)
{
  std::ostringstream output;
  output << std::hex << std::uppercase << std::setfill('0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    if (index != 0U) {
      output << ' ';
    }
    output << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  return output.str();
}

std::string hex_byte(const std::uint8_t value)
{
  std::ostringstream output;
  output << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
         << static_cast<unsigned int>(value);
  return output.str();
}

const char * on_off(const bool value)
{
  return value ? "ON" : "OFF";
}

const char * session_state_name(const jetson_lc_protocol::SessionState state)
{
  using jetson_lc_protocol::SessionState;
  switch (state) {
    case SessionState::Idle: return "Idle";
    case SessionState::PreparationRequestReceived: return "Preparation Request Received";
    case SessionState::CheckingReadiness: return "Checking Readiness";
    case SessionState::NotAvailable: return "Not Available";
    case SessionState::EnteringRobotMaintenanceMode: return "Entering Robot Maintenance Mode";
    case SessionState::MovingToTopServiceFloor: return "Moving to Top Service Floor";
    case SessionState::ReadyForJetsonControl: return "Ready for Jetson/Robot Control";
    case SessionState::CompletionRequestReceived: return "Completion Request Received";
    case SessionState::ReturningToAutoMode: return "Returning to Auto Mode";
    case SessionState::ServiceComplete: return "Service Complete / Normal Operation Restored";
    case SessionState::Failed: return "Failed";
    case SessionState::Aborted: return "Aborted";
  }
  return "Unknown";
}

const char * abnormal_reason_name(const jetson_lc_protocol::AbnormalReason reason)
{
  using jetson_lc_protocol::AbnormalReason;
  switch (reason) {
    case AbnormalReason::None: return "No Abnormal Condition";
    case AbnormalReason::LiftNotAvailable: return "Lift Not Available";
    case AbnormalReason::DoorNotClosed: return "Door Not Closed";
    case AbnormalReason::PassengerServiceConflict: return "Passenger / Service Conflict";
    case AbnormalReason::LiftAbnormalMode: return "Lift Abnormal Mode Detected";
    case AbnormalReason::FailedToEnterRobotMaintenanceMode:
      return "Failed to Enter Robot Maintenance Mode";
    case AbnormalReason::FailedToReturnToAutoMode: return "Failed to Return to Auto Mode";
    case AbnormalReason::Timeout: return "Timeout";
    case AbnormalReason::SafetyInterlock: return "Safety / Interlock Condition";
    case AbnormalReason::InternalLcError: return "Internal LC Error";
    case AbnormalReason::ManualTechnicianIntervention:
      return "Manual Technician Intervention / Manual Service Takeover";
    case AbnormalReason::AbortRequestedByJetson: return "Abort Requested by Jetson";
    case AbnormalReason::JetsonCommunicationLost: return "Jetson Communication Lost";
  }
  return "Unknown";
}

const char * request_type_name(const bool automatic_request, const std::uint8_t type)
{
  if (!automatic_request && type == 0x00U) {
    return "REQUEST_CLEAR";
  }
  if (automatic_request && type == 0x01U) {
    return "START_HOLD";
  }
  if (automatic_request && type == 0x02U) {
    return "ABORT";
  }
  return "INVALID";
}

bool is_start_hold(const jetson_lc_protocol::Frame & frame)
{
  return frame.data.size() == 3U && (frame.data[1] & 0x20U) != 0U &&
         frame.data[2] == 0x01U;
}

bool is_request_clear(const jetson_lc_protocol::Frame & frame)
{
  return frame.data.size() == 3U && (frame.data[1] & 0x20U) == 0U &&
         frame.data[2] == 0x00U;
}

bool is_abort(const jetson_lc_protocol::Frame & frame)
{
  return frame.data.size() == 3U && (frame.data[1] & 0x20U) != 0U &&
         frame.data[2] == 0x02U;
}

void log_received_frame(std::ostream & output, const jetson_lc_protocol::Frame & frame)
{
  output << "\nRX: " << hex_bytes(jetson_lc_protocol::encode_frame(frame)) << '\n'
         << "Function: " << hex_byte(frame.function_code) << '\n'
         << "Lift ID: " << static_cast<unsigned int>(frame.lift_id) << '\n'
         << "CRC: VALID\n";
}

void log_command(
  std::ostream & output, const jetson_lc_protocol::Frame & frame,
  const bool held_request_active)
{
  if (frame.data.size() != 3U) {
    output << "Decoded command: invalid data size (" << frame.data.size()
           << ", expected 3)\n";
    return;
  }

  const std::uint8_t bits = frame.data[1];
  const bool automatic_request = (bits & 0x20U) != 0U;
  output << "Decoded command:\n"
         << "  Destination Floor: " << static_cast<unsigned int>(frame.data[0]) << '\n'
         << "  Hall Call Disable: " << on_off((bits & 0x04U) != 0U) << '\n'
         << "  Manual Up: " << on_off((bits & 0x08U) != 0U) << '\n'
         << "  Manual Down: " << on_off((bits & 0x10U) != 0U) << '\n'
         << "  Automatic Service Request: " << on_off(automatic_request) << '\n'
         << "  Request Type: " << request_type_name(automatic_request, frame.data[2]) << '\n'
         << "  Reserved Door Bits: " << on_off((bits & 0x03U) != 0U) << '\n'
         << "  Spare Bits: " << on_off((bits & 0xC0U) != 0U) << '\n';
  if (is_start_hold(frame)) {
    output << "  Held Request: "
           << (held_request_active ? "HOLD EXISTING ACTIVE REQUEST" : "START ACTIVE REQUEST")
           << '\n';
  }
}

void log_configuration_summary(
  std::ostream & output, const jetson_lc_protocol::Frame & response)
{
  const auto configuration = jetson_lc_protocol::decode_configuration(response);
  if (!configuration) {
    return;
  }
  output << "0x10 reply: capacity=" << configuration->rated_capacity_kg
         << "kg, car_width=" << configuration->car_width_mm
         << "mm, distance=" << configuration->distance_mm
         << "mm, door_width=" << configuration->door_width_mm
         << "mm, second_door=" << (configuration->has_second_door ? "yes" : "no") << '\n';
}

void log_status_summary(std::ostream & output, const jetson_lc_protocol::Frame & response)
{
  const auto status = jetson_lc_protocol::decode_status(response);
  if (!status) {
    return;
  }
  output << "0x11 reply: floor=" << static_cast<unsigned int>(status->current_floor)
         << ", arrived=" << on_off(status->arrived_at_destination)
         << ", permitted=" << on_off(status->jetson_operation_permitted)
         << ", maintenance=" << on_off(status->robot_maintenance_mode)
         << ", abnormal=" << on_off(status->lift_abnormal_mode) << '\n'
         << "  Session: " << hex_byte(static_cast<std::uint8_t>(status->session_state))
         << ' ' << session_state_name(status->session_state) << '\n'
         << "  Abnormal Reason: "
         << hex_byte(static_cast<std::uint8_t>(status->abnormal_reason)) << ' '
         << abnormal_reason_name(status->abnormal_reason) << '\n';
}

double elapsed_ms(const SteadyClock::time_point start, const SteadyClock::time_point end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void log_timing(
  std::ostream & output, const std::uint8_t function_code,
  const SteadyClock::time_point first_byte_detected,
  const SteadyClock::time_point frame_parsed,
  const SteadyClock::time_point reply_prepared,
  const SteadyClock::time_point write_started,
  const SteadyClock::time_point write_completed,
  const unsigned int injected_delay_ms)
{
  output << std::fixed << std::setprecision(3)
         << "Request " << hex_byte(function_code) << " timing:\n"
         << "  frame parsed after: " << elapsed_ms(first_byte_detected, frame_parsed) << " ms\n"
         << "  reply prepared after: " << elapsed_ms(first_byte_detected, reply_prepared)
         << " ms\n"
         << "  reply write started after: " << elapsed_ms(first_byte_detected, write_started)
         << " ms\n"
         << "  reply write completed after: " << elapsed_ms(first_byte_detected, write_completed)
         << " ms\n";
  if (injected_delay_ms != 0U) {
    output << "  explicit --delay-ms: " << injected_delay_ms << " ms\n";
  }
}

unsigned long parse_unsigned(const std::string & text, const char * option)
{
  if (text.empty() || text.front() == '-') {
    throw std::invalid_argument(std::string(option) + " requires a non-negative integer");
  }
  std::size_t consumed = 0;
  unsigned long value = 0;
  try {
    value = std::stoul(text, &consumed, 10);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string(option) + " requires a non-negative integer");
  }
  if (consumed != text.size()) {
    throw std::invalid_argument(std::string(option) + " requires a non-negative integer");
  }
  return value;
}

void print_usage(const char * executable)
{
  std::cout << "Usage: " << executable
            << " COM5 [--baud 38400] [--delay-ms N] [--drop-every N]"
               " [--bad-crc-every N]\n";
}

Options parse_options(const int argc, char ** argv)
{
  if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
    print_usage(argv[0]);
    std::exit(argc < 2 ? 2 : 0);
  }

  Options options;
  options.port = argv[1];
  for (int index = 2; index < argc; index += 2) {
    if (index + 1 >= argc) {
      throw std::invalid_argument(std::string("missing value for ") + argv[index]);
    }
    const std::string option = argv[index];
    const auto value = parse_unsigned(argv[index + 1], option.c_str());
    if (option == "--baud") {
      if (value == 0UL || value > std::numeric_limits<DWORD>::max()) {
        throw std::invalid_argument("--baud is outside the supported range");
      }
      options.baud = static_cast<DWORD>(value);
    } else if (option == "--delay-ms") {
      if (value > std::numeric_limits<unsigned int>::max()) {
        throw std::invalid_argument("--delay-ms is outside the supported range");
      }
      options.delay_ms = static_cast<unsigned int>(value);
    } else if (option == "--drop-every") {
      if (value > std::numeric_limits<unsigned int>::max()) {
        throw std::invalid_argument("--drop-every is outside the supported range");
      }
      options.drop_every = static_cast<unsigned int>(value);
    } else if (option == "--bad-crc-every") {
      if (value > std::numeric_limits<unsigned int>::max()) {
        throw std::invalid_argument("--bad-crc-every is outside the supported range");
      }
      options.bad_crc_every = static_cast<unsigned int>(value);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  return options;
}

std::string windows_device_name(const std::string & port)
{
  if (port.rfind("\\\\.\\", 0U) == 0U) {
    return port;
  }
  return "\\\\.\\" + port;
}

SerialHandle open_serial(const Options & options)
{
  const auto device = windows_device_name(options.port);
  SerialHandle serial(CreateFileA(device.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
      OPEN_EXISTING, 0, nullptr));
  if (serial.get() == INVALID_HANDLE_VALUE) {
    throw std::system_error(static_cast<int>(GetLastError()), std::system_category(),
            "could not open " + device);
  }

  if (!SetupComm(serial.get(), 4096, 4096)) {
    throw std::system_error(static_cast<int>(GetLastError()), std::system_category(), "SetupComm");
  }

  DCB settings{};
  settings.DCBlength = sizeof(settings);
  if (!GetCommState(serial.get(), &settings)) {
    throw std::system_error(
            static_cast<int>(GetLastError()), std::system_category(), "GetCommState");
  }
  settings.BaudRate = options.baud;
  settings.ByteSize = 8;
  settings.Parity = NOPARITY;
  settings.StopBits = ONESTOPBIT;
  settings.fBinary = TRUE;
  settings.fParity = FALSE;
  settings.fOutxCtsFlow = FALSE;
  settings.fOutxDsrFlow = FALSE;
  settings.fDtrControl = DTR_CONTROL_DISABLE;
  settings.fDsrSensitivity = FALSE;
  settings.fTXContinueOnXoff = TRUE;
  settings.fOutX = FALSE;
  settings.fInX = FALSE;
  settings.fErrorChar = FALSE;
  settings.fNull = FALSE;
  settings.fRtsControl = RTS_CONTROL_DISABLE;
  settings.fAbortOnError = FALSE;
  if (!SetCommState(serial.get(), &settings)) {
    throw std::system_error(
            static_cast<int>(GetLastError()), std::system_category(), "SetCommState");
  }

  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = MAXDWORD;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = kIdleReadTimeoutMs;
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 1000;
  if (!SetCommTimeouts(serial.get(), &timeouts)) {
    throw std::system_error(
            static_cast<int>(GetLastError()), std::system_category(), "SetCommTimeouts");
  }
  return serial;
}

struct SerialRead
{
  std::size_t size{};
  SteadyClock::time_point first_byte_detected{};
};

SerialRead read_available(const HANDLE serial, std::array<std::uint8_t, 512> & buffer)
{
  // Request one byte so ReadFile can only wait for the first byte, never for a large buffer.
  DWORD bytes_read = 0;
  if (!ReadFile(serial, buffer.data(), 1, &bytes_read, nullptr)) {
    throw std::system_error(static_cast<int>(GetLastError()), std::system_category(), "ReadFile");
  }
  if (bytes_read == 0U) {
    return {};
  }
  const auto first_byte_detected = SteadyClock::now();

  // Drain only bytes that the driver reports are already queued. If a frame is split, the
  // next one-byte read wakes as soon as its next byte arrives and the streaming parser keeps
  // the partial frame.
  DWORD errors = 0;
  COMSTAT status{};
  if (!ClearCommError(serial, &errors, &status)) {
    throw std::system_error(
            static_cast<int>(GetLastError()), std::system_category(), "ClearCommError");
  }
  const auto queued = (std::min)(
    static_cast<std::size_t>(status.cbInQue), buffer.size() - 1U);
  if (queued == 0U) {
    return {1U, first_byte_detected};
  }

  DWORD drained = 0;
  if (!ReadFile(serial, buffer.data() + 1, static_cast<DWORD>(queued), &drained, nullptr)) {
    throw std::system_error(static_cast<int>(GetLastError()), std::system_category(), "ReadFile");
  }
  return {1U + static_cast<std::size_t>(drained), first_byte_detected};
}

void write_all(const HANDLE serial, const std::vector<std::uint8_t> & bytes)
{
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    DWORD written = 0;
    const auto remaining = static_cast<DWORD>(bytes.size() - offset);
    if (!WriteFile(serial, bytes.data() + offset, remaining, &written, nullptr)) {
      throw std::system_error(static_cast<int>(GetLastError()), std::system_category(), "WriteFile");
    }
    if (written == 0U) {
      throw std::runtime_error("serial write completed without writing data");
    }
    offset += written;
  }
}

BOOL WINAPI console_control_handler(const DWORD event)
{
  if (event == CTRL_C_EVENT || event == CTRL_BREAK_EVENT || event == CTRL_CLOSE_EVENT ||
    event == CTRL_LOGOFF_EVENT || event == CTRL_SHUTDOWN_EVENT)
  {
    g_running = false;
    return TRUE;
  }
  return FALSE;
}

void serve(const HANDLE serial, const Options & options)
{
  jetson_lc_protocol::FrameParser parser;
  jetson_lc_tools::MockLcLogic logic(kLiftId);
  std::array<std::uint8_t, 512> buffer{};
  std::uint32_t request_count = 0;
  bool held_request_active = false;
  std::optional<SteadyClock::time_point> first_byte_detected;
  std::string pending_output;

  while (g_running) {
    const auto serial_read = read_available(serial, buffer);
    if (serial_read.size == 0U) {
      if (!pending_output.empty()) {
        std::cout << pending_output;
        pending_output.clear();
      }
      continue;
    }
    if (!first_byte_detected) {
      first_byte_detected = serial_read.first_byte_detected;
    }

    const auto parsed = parser.append(buffer.data(), serial_read.size);
    const auto frame_parsed = SteadyClock::now();
    if (parsed.crc_errors != 0U) {
      pending_output += "\nRX rejected: " + std::to_string(parsed.crc_errors) +
        " invalid CRC frame(s)\n";
    }
    if (parsed.malformed_frames != 0U) {
      pending_output += "\nRX rejected: " + std::to_string(parsed.malformed_frames) +
        " malformed Data Length frame(s)\n";
    }

    for (const auto & request : parsed.frames) {
      ++request_count;
      const auto request_first_byte = first_byte_detected.value_or(frame_parsed);
      const bool was_held_request_active = held_request_active;

      if (options.drop_every != 0U && request_count % options.drop_every == 0U) {
        std::ostringstream output;
        log_received_frame(output, request);
        if (request.function_code == jetson_lc_protocol::kFunctionCommand) {
          log_command(output, request, was_held_request_active);
        }
        output << "FAULT: dropping response for request #" << request_count << '\n';
        pending_output += output.str();
        continue;
      }

      const auto response = logic.reply(request);
      if (!response) {
        std::ostringstream output;
        log_received_frame(output, request);
        if (request.function_code == jetson_lc_protocol::kFunctionCommand) {
          log_command(output, request, was_held_request_active);
        }
        output << "No response: unsupported or invalid request\n";
        pending_output += output.str();
        continue;
      }

      if (is_start_hold(request)) {
        held_request_active = true;
      } else if (is_request_clear(request) || is_abort(request)) {
        held_request_active = false;
      }

      auto bytes = jetson_lc_protocol::encode_frame(*response);
      if (options.bad_crc_every != 0U && request_count % options.bad_crc_every == 0U) {
        bytes.back() ^= 0xFFU;
      }
      const auto reply_prepared = SteadyClock::now();

      if (options.delay_ms != 0U) {
        std::this_thread::sleep_for(std::chrono::milliseconds(options.delay_ms));
      }

      const auto write_started = SteadyClock::now();
      write_all(serial, bytes);
      const auto write_completed = SteadyClock::now();

      // Console output is intentionally after WriteFile so it is outside this reply's latency
      // path. Build one block and avoid std::endl's forced flush.
      std::ostringstream output;
      log_received_frame(output, request);
      if (request.function_code == jetson_lc_protocol::kFunctionConfiguration) {
        output << "0x10 request: configuration and distance\n";
      } else if (request.function_code == jetson_lc_protocol::kFunctionStatus) {
        output << "0x11 request: current LC status\n";
      } else if (request.function_code == jetson_lc_protocol::kFunctionCommand) {
        log_command(output, request, was_held_request_active);
      }
      if (options.delay_ms != 0U) {
        output << "FAULT: delayed response by " << options.delay_ms << " ms\n";
      }
      if (options.bad_crc_every != 0U && request_count % options.bad_crc_every == 0U) {
        output << "FAULT: corrupted response CRC for request #" << request_count << '\n';
      }
      if (response->function_code == jetson_lc_protocol::kFunctionConfiguration) {
        log_configuration_summary(output, *response);
      } else if (response->function_code == jetson_lc_protocol::kFunctionStatus) {
        log_status_summary(output, *response);
      }
      const char * label = response->function_code == jetson_lc_protocol::kFunctionCommand ?
        "TX ACK" : "TX";
      output << label << ": " << hex_bytes(bytes) << '\n';
      log_timing(output, request.function_code, request_first_byte, frame_parsed,
        reply_prepared, write_started, write_completed, options.delay_ms);
      pending_output += output.str();
    }
    if (!parsed.frames.empty()) {
      first_byte_detected.reset();
    }
  }
  if (!pending_output.empty()) {
    std::cout << pending_output;
  }
}
}

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    if (!SetConsoleCtrlHandler(console_control_handler, TRUE)) {
      throw std::system_error(
              static_cast<int>(GetLastError()), std::system_category(), "SetConsoleCtrlHandler");
    }
    auto serial = open_serial(options);
    std::cout << "Standalone mock LC listening on " << options.port << " at " << options.baud
              << " baud, 8N1, no flow control\n"
              << "Lift ID: " << static_cast<unsigned int>(kLiftId) << '\n'
              << "Responder only; press Ctrl+C to stop.\n";
    serve(serial.get(), options);
    std::cout << "Stopped.\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "mock_lc_serial: " << error.what() << '\n';
    return 1;
  }
}

#else

int main()
{
  std::cerr << "mock_lc_serial uses the Windows COM API and must be run on Windows.\n";
  return 1;
}

#endif
