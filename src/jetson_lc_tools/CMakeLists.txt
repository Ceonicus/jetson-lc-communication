cmake_minimum_required(VERSION 3.8)
project(jetson_lc_tools)

find_package(ament_cmake REQUIRED)
find_package(jetson_lc_protocol REQUIRED)

add_library(mock_lc_logic src/mock_lc_logic.cpp)
target_compile_features(mock_lc_logic PUBLIC cxx_std_17)
target_compile_options(mock_lc_logic PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(mock_lc_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mock_lc_logic jetson_lc_protocol)

add_executable(mock_lc src/mock_lc.cpp)
target_compile_features(mock_lc PRIVATE cxx_std_17)
target_compile_options(mock_lc PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mock_lc mock_lc_logic util pthread)
ament_target_dependencies(mock_lc jetson_lc_protocol)

install(TARGETS mock_lc DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mock_lc test/test_mock_lc.cpp)
  target_link_libraries(test_mock_lc mock_lc_logic)
  ament_target_dependencies(test_mock_lc jetson_lc_protocol)
  add_test(NAME mock_lc_pty_smoke COMMAND mock_lc --self-test)
endif()

ament_package()
