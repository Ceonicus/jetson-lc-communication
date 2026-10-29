#include "jetson_lc_driver/lc_driver_node.hpp"

#include "jetson_lc_protocol/protocol.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

namespace jetson_lc_driver
{
namespace
{
std::chrono::milliseconds positive_period(rclcpp::Node & node, const char * name, const int value)
{
  if (value <= 0) {throw std::invalid_argument(std::string(name) + " must be positive");}
  (void)node;
  return std::chrono::milliseconds(value);
}
}

LcDriverNode::LcDriverNode() : Node("lc_driver")
{
  const auto serial_device = declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
  const auto baud_rate = declare_parameter<int>("baud_rate", 38400);
  const auto lift_id = declare_parameter<int>("lift_id", 1);
  const auto response_timeout = declare_parameter<int>("response_timeout_ms", 100);
  const auto reconnect_period = declare_parameter<int>("reconnect_period_ms", 1000);
  const auto status_period = declare_parameter<int>("status_poll_period_ms", 200);
  const auto distance_period = declare_parameter<int>("distance_poll_period_ms", 200);
  const auto configuration_period = declare_parameter<int>("configuration_refresh_period_ms", 5000);
  const auto hold_period = declare_parameter<int>("hold_frame_period_ms", 500);
  const auto retry_period = declare_parameter<int>("request_retry_period_ms", 250);
  const auto raw_logging = declare_parameter<bool>("raw_frame_logging", false);
  if (baud_rate <= 0 || lift_id <= 0 || lift_id > 255) {
    throw std::invalid_argument("baud_rate and Lift ID are invalid");
  }
  if (response_timeout != 100) {
    throw std::invalid_argument("response_timeout_ms must be the approved 100 ms");
  }
  if (retry_period <= response_timeout) {
    throw std::invalid_argument("request_retry_period_ms must exceed response_timeout_ms");
  }

  status_publisher_ = create_publisher<jetson_lc_interfaces::msg::LcStatus>("/lc/status", 10);
  distance_publisher_ = create_publisher<jetson_lc_interfaces::msg::LcDistance>("/lc/distance", 10);
  configuration_publisher_ = create_publisher<jetson_lc_interfaces::msg::LcConfiguration>(
    "/lc/configuration", rclcpp::QoS(1).transient_local().reliable());
  health_publisher_ = create_publisher<jetson_lc_interfaces::msg::LcCommunicationStatus>(
    "/lc/communication_status", 10);

  DriverConfiguration configuration{serial_device, static_cast<unsigned int>(baud_rate),
    static_cast<std::uint8_t>(lift_id), positive_period(*this, "response_timeout_ms", response_timeout),
    positive_period(*this, "reconnect_period_ms", reconnect_period),
    positive_period(*this, "status_poll_period_ms", status_period),
    positive_period(*this, "distance_poll_period_ms", distance_period),
    positive_period(*this, "configuration_refresh_period_ms", configuration_period),
    positive_period(*this, "hold_frame_period_ms", hold_period),
    positive_period(*this, "request_retry_period_ms", retry_period), raw_logging};

  RuntimeCallbacks callbacks;
  callbacks.status = [this, lift_id](const jetson_lc_protocol::Status & status) {
      jetson_lc_interfaces::msg::LcStatus message;
      message.stamp = now();
      message.lift_id = static_cast<std::uint8_t>(lift_id);
      message.current_floor = status.current_floor;
      message.arrived_at_destination = status.arrived_at_destination;
      message.jetson_operation_permitted = status.jetson_operation_permitted;
      message.robot_maintenance_mode = status.robot_maintenance_mode;
      message.lift_abnormal_mode = status.lift_abnormal_mode;
      message.automatic_service_session_state = static_cast<std::uint8_t>(status.session_state);
      message.automatic_service_abnormal_reason = static_cast<std::uint8_t>(status.abnormal_reason);
      status_publisher_->publish(message);
    };
  callbacks.configuration = [this](const jetson_lc_protocol::Configuration & value,
      const bool publish_configuration) {
      const auto stamp = now();
      jetson_lc_interfaces::msg::LcDistance distance;
      distance.stamp = stamp;
      distance.distance_from_car_top_to_current_floor_sill_mm = value.distance_mm;
      distance.valid = true;
      distance_publisher_->publish(distance);
      if (publish_configuration) {
        jetson_lc_interfaces::msg::LcConfiguration message;
        message.stamp = stamp;
        message.rated_capacity_kg = value.rated_capacity_kg;
        message.car_width_mm = value.car_width_mm;
        message.door_width_mm = value.door_width_mm;
        message.has_second_door = value.has_second_door;
        message.second_door_position = value.second_door_position;
        message.service_floor_configuration = value.service_floor_configuration;
        configuration_publisher_->publish(message);
      }
    };
  callbacks.health = [this](const CommunicationHealth & value) {
      jetson_lc_interfaces::msg::LcCommunicationStatus message;
      message.stamp = now();
      message.serial_connected = value.serial_connected;
      message.receiving_valid_frames = value.receiving_valid_frames;
      message.status_stale = value.status_stale;
      message.consecutive_timeouts = value.consecutive_timeouts;
      message.crc_error_count = value.crc_error_count;
      message.malformed_frame_count = value.malformed_frame_count;
      message.last_function_code = value.last_function_code;
      message.last_error = value.last_error;
      health_publisher_->publish(message);
    };
  callbacks.debug = [this](const std::string & text) {RCLCPP_DEBUG(get_logger(), "%s", text.c_str());};
  runtime_ = std::make_unique<CommunicationRuntime>(std::move(configuration), std::move(callbacks));

  command_service_ = create_service<jetson_lc_interfaces::srv::SetLcCommandState>(
    "/lc/set_command_state",
    [this](const std::shared_ptr<jetson_lc_interfaces::srv::SetLcCommandState::Request> request,
      std::shared_ptr<jetson_lc_interfaces::srv::SetLcCommandState::Response> response) {
      jetson_lc_protocol::CommandState command{request->destination_floor,
        request->hall_call_disable,
        static_cast<jetson_lc_protocol::ManualMotion>(request->manual_motion),
        static_cast<jetson_lc_protocol::MaintenanceRequest>(request->maintenance_request)};
      std::string reason;
      if (!jetson_lc_protocol::validate_command(command, &reason)) {
        response->accepted = false;
        response->message = reason;
        return;
      }
      if (command.manual_motion != jetson_lc_protocol::ManualMotion::Stop &&
        !runtime_->manual_motion_allowed())
      {
        response->accepted = false;
        response->message = "manual motion requires fresh, safe Ready status";
        return;
      }
      runtime_->queue_command(command);
      response->accepted = true;
      response->message = "validated and queued; LC completion is not implied";
    });

  refresh_service_ = create_service<jetson_lc_interfaces::srv::RefreshLcConfiguration>(
    "/lc/refresh_configuration",
    [this](const std::shared_ptr<jetson_lc_interfaces::srv::RefreshLcConfiguration::Request>,
      std::shared_ptr<jetson_lc_interfaces::srv::RefreshLcConfiguration::Response> response) {
      runtime_->refresh_configuration();
      response->accepted = true;
      response->message = "configuration refresh queued";
    });
  runtime_->start();
}

LcDriverNode::~LcDriverNode()
{
  if (runtime_) {runtime_->shutdown(); runtime_.reset();}
}
}
