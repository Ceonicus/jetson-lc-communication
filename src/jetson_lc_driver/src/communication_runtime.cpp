#include "jetson_lc_driver/communication_runtime.hpp"

#include "jetson_lc_driver/serial_transport.hpp"
#include "jetson_lc_protocol/protocol.hpp"

#include <boost/asio.hpp>

#include <algorithm>
#include <future>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>

namespace jetson_lc_driver
{
using jetson_lc_protocol::CommandState;
using jetson_lc_protocol::Frame;
using jetson_lc_protocol::MaintenanceRequest;
using jetson_lc_protocol::ManualMotion;
using jetson_lc_protocol::SessionState;

namespace
{
enum class Transition {AckOnly, StartHold, Clear, Abort};
enum class TransactionKind {Status, Configuration, CommandChange, Hold};

std::string hexadecimal(const std::vector<std::uint8_t> & bytes)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : bytes) {stream << std::setw(2) << static_cast<unsigned>(byte) << ' ';}
  return stream.str();
}

bool same_configuration(
  const jetson_lc_protocol::Configuration & left,
  const jetson_lc_protocol::Configuration & right)
{
  return left.rated_capacity_kg == right.rated_capacity_kg &&
         left.car_width_mm == right.car_width_mm && left.door_width_mm == right.door_width_mm &&
         left.has_second_door == right.has_second_door &&
         left.second_door_position == right.second_door_position &&
         left.service_floor_configuration == right.service_floor_configuration;
}
}

class CommunicationRuntime::Impl
{
public:
  Impl(DriverConfiguration configuration, RuntimeCallbacks callbacks)
  : configuration_(std::move(configuration)), callbacks_(std::move(callbacks)), transport_(io_),
    response_timer_(io_), reconnect_timer_(io_), status_timer_(io_), distance_timer_(io_),
    configuration_timer_(io_), hold_timer_(io_), retry_timer_(io_), stale_timer_(io_),
    shutdown_timer_(io_)
  {}

  ~Impl() {shutdown();}

  void start()
  {
    if (worker_.joinable()) {return;}
    work_ = std::make_unique<Work>(boost::asio::make_work_guard(io_));
    worker_ = std::thread([this] {io_.run();});
    boost::asio::post(io_, [this] {
      arm_periodic_timers();
      open_transport();
    });
  }

  void shutdown()
  {
    if (!worker_.joinable()) {return;}
    boost::asio::post(io_, [this] {begin_shutdown();});
    worker_.join();
  }

  void queue_command(const CommandState & command)
  {
    boost::asio::post(io_, [this, command] {
      if (shutting_down_) {return;}
      if (command.manual_motion != ManualMotion::Stop && !manual_motion_allowed()) {
        force_manual_stop();
        schedule();
        return;
      }
      const auto previous_request = desired_.maintenance_request;
      desired_ = command;
      ++desired_generation_;
      pending_generation_ = desired_generation_;
      pending_sent_generation_ = 0;
      pending_command_ = command;
      pending_due_ = true;
      if (command.maintenance_request == MaintenanceRequest::Abort &&
        previous_request != MaintenanceRequest::Abort)
      {
        pending_transition_ = Transition::Abort;
      } else if (command.maintenance_request == MaintenanceRequest::None &&
        previous_request != MaintenanceRequest::None)
      {
        pending_transition_ = Transition::Clear;
        completion_mode_ = true;
      } else if (command.maintenance_request == MaintenanceRequest::StartHold &&
        previous_request != MaintenanceRequest::StartHold)
      {
        pending_transition_ = Transition::StartHold;
      } else {
        pending_transition_ = Transition::AckOnly;
      }
      schedule();
    });
  }

  void refresh_configuration()
  {
    boost::asio::post(io_, [this] {
      force_configuration_publish_ = true;
      distance_due_ = true;
      schedule();
    });
  }

  bool manual_motion_allowed() const
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!health_.serial_connected || health_.status_stale || !last_status_) {return false;}
    const auto age = std::chrono::steady_clock::now() - last_status_time_;
    const auto & status = *last_status_;
    return age <= configuration_.status_poll_period + configuration_.response_timeout &&
      status.session_state == SessionState::ReadyForJetsonControl &&
      status.jetson_operation_permitted && status.robot_maintenance_mode &&
      !status.lift_abnormal_mode;
  }

private:
  using Work = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;

  struct Transaction
  {
    TransactionKind kind;
    std::uint8_t expected_function;
    std::uint64_t generation{};
  };

  void arm_periodic_timers()
  {
    arm_status_timer();
    arm_distance_timer();
    arm_configuration_timer();
    arm_hold_timer();
  }

  void arm_status_timer()
  {
    status_timer_.expires_after(configuration_.status_poll_period);
    status_timer_.async_wait([this](const boost::system::error_code & error) {
      if (!error && !shutting_down_) {status_due_ = true; schedule(); arm_status_timer();}
    });
  }

  void arm_distance_timer()
  {
    distance_timer_.expires_after(configuration_.distance_poll_period);
    distance_timer_.async_wait([this](const boost::system::error_code & error) {
      if (!error && !shutting_down_) {distance_due_ = true; schedule(); arm_distance_timer();}
    });
  }

  void arm_configuration_timer()
  {
    configuration_timer_.expires_after(configuration_.configuration_refresh_period);
    configuration_timer_.async_wait([this](const boost::system::error_code & error) {
      if (!error && !shutting_down_) {
        force_configuration_publish_ = true;
        arm_configuration_timer();
      }
    });
  }

  void arm_hold_timer()
  {
    hold_timer_.expires_after(configuration_.hold_frame_period);
    hold_timer_.async_wait([this](const boost::system::error_code & error) {
      if (!error && !shutting_down_) {hold_due_ = true; schedule(); arm_hold_timer();}
    });
  }

  void open_transport()
  {
    if (shutting_down_) {return;}
    try {
      transport_.open(configuration_.serial_device, configuration_.baud_rate);
      parser_.clear();
      set_connected(true, "");
      transport_.start_read([this](const boost::system::error_code & error,
          const std::uint8_t * bytes, const std::size_t size) {on_read(error, bytes, size);});
      status_due_ = true;
      distance_due_ = true;
      schedule();
    } catch (const std::exception & error) {
      transport_.close();
      set_connected(false, std::string("serial open failed: ") + error.what());
      arm_reconnect();
    }
  }

  void arm_reconnect()
  {
    reconnect_timer_.expires_after(configuration_.reconnect_period);
    reconnect_timer_.async_wait([this](const boost::system::error_code & error) {
      if (!error && !shutting_down_) {open_transport();}
    });
  }

  void set_connected(const bool connected, const std::string & error)
  {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      health_.serial_connected = connected;
      if (!connected) {
        health_.receiving_valid_frames = false;
        health_.status_stale = true;
      }
      health_.last_error = error;
    }
    if (!connected) {force_manual_stop();}
    publish_health();
  }

  void disconnect(const std::string & reason)
  {
    response_timer_.cancel();
    outstanding_.reset();
    write_in_progress_ = false;
    transport_.close();
    set_connected(false, reason);
    arm_reconnect();
  }

  void schedule()
  {
    if (shutting_down_ || outstanding_ || !transport_.is_open()) {return;}
    if (pending_due_ && pending_command_) {
      pending_due_ = false;
      send(TransactionKind::CommandChange,
        jetson_lc_protocol::encode_command_request(configuration_.lift_id, *pending_command_),
        jetson_lc_protocol::kFunctionCommand, pending_generation_);
      return;
    }
    if (!completion_mode_ && hold_due_ && desired_.maintenance_request == MaintenanceRequest::StartHold) {
      hold_due_ = false;
      send(TransactionKind::Hold,
        jetson_lc_protocol::encode_command_request(configuration_.lift_id, desired_),
        jetson_lc_protocol::kFunctionCommand, 0);
      return;
    }
    if (status_due_) {
      status_due_ = false;
      send(TransactionKind::Status,
        jetson_lc_protocol::encode_poll_request(configuration_.lift_id,
        jetson_lc_protocol::kFunctionStatus), jetson_lc_protocol::kFunctionStatus, 0);
      return;
    }
    if (!completion_mode_ && distance_due_) {
      distance_due_ = false;
      send(TransactionKind::Configuration,
        jetson_lc_protocol::encode_poll_request(configuration_.lift_id,
        jetson_lc_protocol::kFunctionConfiguration),
        jetson_lc_protocol::kFunctionConfiguration, 0);
    }
  }

  void send(const TransactionKind kind, std::vector<std::uint8_t> bytes,
    const std::uint8_t expected_function, const std::uint64_t generation)
  {
    if (configuration_.raw_frame_logging && callbacks_.debug) {
      callbacks_.debug("TX " + hexadecimal(bytes));
    }
    outstanding_ = Transaction{kind, expected_function, generation};
    if (kind == TransactionKind::CommandChange) {pending_sent_generation_ = generation;}
    write_in_progress_ = true;
    transport_.async_write(std::move(bytes),
      [this](const boost::system::error_code & error, const std::size_t) {
        write_in_progress_ = false;
        if (error) {disconnect("serial write failed: " + error.message()); return;}
        response_timer_.expires_after(configuration_.response_timeout);
        response_timer_.async_wait([this](const boost::system::error_code & timer_error) {
          if (!timer_error) {on_timeout();}
        });
        if (outstanding_ && outstanding_->kind == TransactionKind::CommandChange) {
          arm_request_retry(outstanding_->generation);
        }
      });
  }

  void arm_request_retry(const std::uint64_t generation)
  {
    retry_timer_.expires_after(configuration_.request_retry_period);
    retry_timer_.async_wait([this, generation](const boost::system::error_code & error) {
      if (!error && pending_command_ && pending_generation_ == generation) {
        pending_due_ = true;
        schedule();
      }
    });
  }

  void on_timeout()
  {
    if (!outstanding_) {return;}
    const bool command_change = outstanding_->kind == TransactionKind::CommandChange;
    outstanding_.reset();
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      ++health_.consecutive_timeouts;
      health_.receiving_valid_frames = false;
      health_.status_stale = true;
      health_.last_error = "LC response timeout";
    }
    if (command_change) {status_due_ = true;}
    force_manual_stop();
    publish_health();
    schedule();
  }

  void on_read(const boost::system::error_code & error, const std::uint8_t * bytes,
    const std::size_t size)
  {
    if (error) {
      if (!shutting_down_ && error != boost::asio::error::operation_aborted) {
        disconnect("serial read failed: " + error.message());
      }
      return;
    }
    if (configuration_.raw_frame_logging && callbacks_.debug) {
      callbacks_.debug("RX " + hexadecimal(std::vector<std::uint8_t>(bytes, bytes + size)));
    }
    auto result = parser_.append(bytes, size);
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      health_.crc_error_count += result.crc_errors;
      health_.malformed_frame_count += result.malformed_frames;
      if (result.crc_errors != 0U) {health_.last_error = "invalid CRC";}
      if (result.malformed_frames != 0U) {health_.last_error = "malformed frame";}
    }
    for (const auto & frame : result.frames) {on_frame(frame);}
    if (result.crc_errors != 0U || result.malformed_frames != 0U) {publish_health();}
  }

  void on_frame(const Frame & frame)
  {
    if (!outstanding_ || !jetson_lc_protocol::matches_response(
        frame, configuration_.lift_id, outstanding_->expected_function))
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      ++health_.malformed_frame_count;
      health_.last_error = frame.lift_id != configuration_.lift_id ?
        "wrong Lift ID" : "unexpected Function Code";
      publish_health_unlocked();
      return;
    }

    bool valid = false;
    const auto transaction = *outstanding_;
    if (transaction.kind == TransactionKind::Status) {
      const auto status = jetson_lc_protocol::decode_status(frame);
      if (status) {process_status(*status); valid = true;}
    } else if (transaction.kind == TransactionKind::Configuration) {
      const auto configuration = jetson_lc_protocol::decode_configuration(frame);
      if (configuration) {process_configuration(*configuration); valid = true;}
    } else {
      valid = jetson_lc_protocol::is_command_ack(frame);
      if (valid && transaction.kind == TransactionKind::CommandChange && pending_command_ &&
        transaction.generation == pending_generation_)
      {
        clear_pending();
      }
    }
    if (!valid) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      ++health_.malformed_frame_count;
      health_.last_error = "invalid reply data length or ACK";
      publish_health_unlocked();
      return;
    }
    response_timer_.cancel();
    outstanding_.reset();
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      health_.receiving_valid_frames = true;
      health_.consecutive_timeouts = 0;
      health_.last_function_code = frame.function_code;
      health_.last_error.clear();
    }
    publish_health();
    schedule();
  }

  void process_status(const jetson_lc_protocol::Status & status)
  {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_status_ = status;
      last_status_time_ = std::chrono::steady_clock::now();
      health_.status_stale = false;
    }
    stale_timer_.expires_after(configuration_.status_poll_period + configuration_.response_timeout);
    stale_timer_.async_wait([this](const boost::system::error_code & error) {
      if (error || shutting_down_) {return;}
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        health_.status_stale = true;
        health_.receiving_valid_frames = false;
        health_.last_error = "LC status stale";
      }
      force_manual_stop();
      publish_health();
      schedule();
    });
    if (pending_command_ && pending_sent_generation_ == pending_generation_ &&
      transition_observed(status.session_state))
    {
      clear_pending();
    }
    if (completion_mode_ && status.session_state == SessionState::ServiceComplete) {
      completion_mode_ = false;
    }
    const bool safe = status.session_state == SessionState::ReadyForJetsonControl &&
      status.jetson_operation_permitted && status.robot_maintenance_mode &&
      !status.lift_abnormal_mode;
    if (!safe) {force_manual_stop();}
    if (callbacks_.status) {callbacks_.status(status);}
  }

  bool transition_observed(const SessionState state) const
  {
    const auto value = static_cast<std::uint8_t>(state);
    switch (pending_transition_) {
      case Transition::StartHold: return value >= 0x01U && value <= 0x06U;
      case Transition::Clear: return value >= 0x07U && value <= 0x09U;
      case Transition::Abort: return state == SessionState::Aborted;
      case Transition::AckOnly: return false;
    }
    return false;
  }

  void process_configuration(const jetson_lc_protocol::Configuration & configuration)
  {
    const bool changed = !last_configuration_ || !same_configuration(*last_configuration_, configuration);
    const bool publish_configuration = changed || force_configuration_publish_;
    last_configuration_ = configuration;
    force_configuration_publish_ = false;
    if (callbacks_.configuration) {callbacks_.configuration(configuration, publish_configuration);}
  }

  void clear_pending()
  {
    pending_command_.reset();
    pending_due_ = false;
    pending_sent_generation_ = 0;
    retry_timer_.cancel();
  }

  void force_manual_stop()
  {
    if (desired_.manual_motion == ManualMotion::Stop) {return;}
    desired_.manual_motion = ManualMotion::Stop;
    ++desired_generation_;
    pending_generation_ = desired_generation_;
    pending_sent_generation_ = 0;
    pending_command_ = desired_;
    pending_transition_ = Transition::AckOnly;
    pending_due_ = true;
  }

  void publish_health()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    publish_health_unlocked();
  }

  void publish_health_unlocked()
  {
    if (callbacks_.health) {callbacks_.health(health_);}
  }

  void begin_shutdown()
  {
    if (shutting_down_) {return;}
    shutting_down_ = true;
    status_timer_.cancel(); distance_timer_.cancel(); configuration_timer_.cancel();
    hold_timer_.cancel(); retry_timer_.cancel(); reconnect_timer_.cancel(); response_timer_.cancel();
    stale_timer_.cancel();
    outstanding_.reset();
    if (desired_.manual_motion != ManualMotion::Stop && transport_.is_open() && !write_in_progress_) {
      desired_.manual_motion = ManualMotion::Stop;
      transport_.async_write(
        jetson_lc_protocol::encode_command_request(configuration_.lift_id, desired_),
        [this](const boost::system::error_code &, const std::size_t) {finish_shutdown();});
      shutdown_timer_.expires_after(std::min(configuration_.response_timeout,
          std::chrono::milliseconds(100)));
      shutdown_timer_.async_wait([this](const boost::system::error_code & error) {
        if (!error) {finish_shutdown();}
      });
    } else {
      finish_shutdown();
    }
  }

  void finish_shutdown()
  {
    if (!work_) {return;}
    shutdown_timer_.cancel();
    transport_.close();
    work_.reset();
    io_.stop();
  }

  DriverConfiguration configuration_;
  RuntimeCallbacks callbacks_;
  boost::asio::io_context io_;
  SerialTransport transport_;
  boost::asio::steady_timer response_timer_;
  boost::asio::steady_timer reconnect_timer_;
  boost::asio::steady_timer status_timer_;
  boost::asio::steady_timer distance_timer_;
  boost::asio::steady_timer configuration_timer_;
  boost::asio::steady_timer hold_timer_;
  boost::asio::steady_timer retry_timer_;
  boost::asio::steady_timer stale_timer_;
  boost::asio::steady_timer shutdown_timer_;
  std::unique_ptr<Work> work_;
  std::thread worker_;
  jetson_lc_protocol::FrameParser parser_;
  std::optional<Transaction> outstanding_;
  bool write_in_progress_{false};
  bool shutting_down_{false};
  bool status_due_{true};
  bool distance_due_{true};
  bool hold_due_{false};
  bool pending_due_{false};
  bool completion_mode_{false};
  bool force_configuration_publish_{true};
  CommandState desired_;
  std::uint64_t desired_generation_{};
  std::optional<CommandState> pending_command_;
  std::uint64_t pending_generation_{};
  std::uint64_t pending_sent_generation_{};
  Transition pending_transition_{Transition::AckOnly};
  std::optional<jetson_lc_protocol::Configuration> last_configuration_;
  mutable std::mutex status_mutex_;
  std::optional<jetson_lc_protocol::Status> last_status_;
  std::chrono::steady_clock::time_point last_status_time_{};
  CommunicationHealth health_;
};

CommunicationRuntime::CommunicationRuntime(
  DriverConfiguration configuration, RuntimeCallbacks callbacks)
: impl_(std::make_unique<Impl>(std::move(configuration), std::move(callbacks))) {}

CommunicationRuntime::~CommunicationRuntime() {impl_->shutdown();}
void CommunicationRuntime::start() {impl_->start();}
void CommunicationRuntime::shutdown() {impl_->shutdown();}
void CommunicationRuntime::queue_command(const CommandState & command) {impl_->queue_command(command);}
void CommunicationRuntime::refresh_configuration() {impl_->refresh_configuration();}
bool CommunicationRuntime::manual_motion_allowed() const {return impl_->manual_motion_allowed();}
}
