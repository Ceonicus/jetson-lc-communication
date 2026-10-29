#pragma once

#include "jetson_lc_protocol/protocol.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace jetson_lc_driver
{
struct DriverConfiguration
{
  std::string serial_device;
  unsigned int baud_rate{};
  std::uint8_t lift_id{};
  std::chrono::milliseconds response_timeout;
  std::chrono::milliseconds reconnect_period;
  std::chrono::milliseconds status_poll_period;
  std::chrono::milliseconds distance_poll_period;
  std::chrono::milliseconds configuration_refresh_period;
  std::chrono::milliseconds hold_frame_period;
  std::chrono::milliseconds request_retry_period;
  bool raw_frame_logging{};
};

struct CommunicationHealth
{
  bool serial_connected{};
  bool receiving_valid_frames{};
  bool status_stale{true};
  std::uint32_t consecutive_timeouts{};
  std::uint32_t crc_error_count{};
  std::uint32_t malformed_frame_count{};
  std::uint8_t last_function_code{};
  std::string last_error;
};

struct RuntimeCallbacks
{
  std::function<void(const jetson_lc_protocol::Status &)> status;
  std::function<void(const jetson_lc_protocol::Configuration &, bool)> configuration;
  std::function<void(const CommunicationHealth &)> health;
  std::function<void(const std::string &)> debug;
};

class CommunicationRuntime
{
public:
  CommunicationRuntime(DriverConfiguration configuration, RuntimeCallbacks callbacks);
  ~CommunicationRuntime();
  CommunicationRuntime(const CommunicationRuntime &) = delete;
  CommunicationRuntime & operator=(const CommunicationRuntime &) = delete;

  void start();
  void shutdown();
  void queue_command(const jetson_lc_protocol::CommandState & command);
  void refresh_configuration();
  bool manual_motion_allowed() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
