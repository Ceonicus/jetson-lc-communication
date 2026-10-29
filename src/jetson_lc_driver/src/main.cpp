#include "jetson_lc_driver/lc_driver_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetson_lc_driver::LcDriverNode>());
  rclcpp::shutdown();
  return 0;
}
