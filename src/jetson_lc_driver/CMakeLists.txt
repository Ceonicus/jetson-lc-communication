cmake_minimum_required(VERSION 3.8)
project(jetson_lc_driver)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(jetson_lc_interfaces REQUIRED)
find_package(jetson_lc_protocol REQUIRED)

add_executable(lc_driver_node
  src/main.cpp
  src/lc_driver_node.cpp
  src/communication_runtime.cpp
  src/serial_transport.cpp)
target_compile_features(lc_driver_node PRIVATE cxx_std_17)
target_compile_options(lc_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(lc_driver_node PRIVATE include)
ament_target_dependencies(lc_driver_node rclcpp jetson_lc_interfaces jetson_lc_protocol)
target_link_libraries(lc_driver_node Boost::system)

install(TARGETS lc_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
