#pragma once

#include "jetson_lc_driver/communication_runtime.hpp"

#include <jetson_lc_interfaces/msg/lc_communication_status.hpp>
#include <jetson_lc_interfaces/msg/lc_configuration.hpp>
#include <jetson_lc_interfaces/msg/lc_distance.hpp>
#include <jetson_lc_interfaces/msg/lc_status.hpp>
#include <jetson_lc_interfaces/srv/refresh_lc_configuration.hpp>
#include <jetson_lc_interfaces/srv/set_lc_command_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace jetson_lc_driver
{
class LcDriverNode : public rclcpp::Node
{
public:
  LcDriverNode();
  ~LcDriverNode() override;

private:
  rclcpp::Publisher<jetson_lc_interfaces::msg::LcStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<jetson_lc_interfaces::msg::LcDistance>::SharedPtr distance_publisher_;
  rclcpp::Publisher<jetson_lc_interfaces::msg::LcConfiguration>::SharedPtr configuration_publisher_;
  rclcpp::Publisher<jetson_lc_interfaces::msg::LcCommunicationStatus>::SharedPtr health_publisher_;
  rclcpp::Service<jetson_lc_interfaces::srv::SetLcCommandState>::SharedPtr command_service_;
  rclcpp::Service<jetson_lc_interfaces::srv::RefreshLcConfiguration>::SharedPtr refresh_service_;
  std::unique_ptr<CommunicationRuntime> runtime_;
};
}
