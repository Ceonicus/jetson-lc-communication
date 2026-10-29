cmake_minimum_required(VERSION 3.16)
project(mock_lc_serial LANGUAGES CXX)

get_filename_component(REPOSITORY_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

add_executable(mock_lc_serial
  src/main.cpp
  ${REPOSITORY_ROOT}/src/jetson_lc_protocol/src/crc16_modbus.cpp
  ${REPOSITORY_ROOT}/src/jetson_lc_protocol/src/frame_codec.cpp
  ${REPOSITORY_ROOT}/src/jetson_lc_protocol/src/protocol_codec.cpp
  ${REPOSITORY_ROOT}/src/jetson_lc_tools/src/mock_lc_logic.cpp)

target_compile_features(mock_lc_serial PRIVATE cxx_std_17)
target_include_directories(mock_lc_serial PRIVATE
  ${REPOSITORY_ROOT}/src/jetson_lc_protocol/include
  ${REPOSITORY_ROOT}/src/jetson_lc_tools/include)

if(MSVC)
  target_compile_options(mock_lc_serial PRIVATE /W4 /permissive-)
else()
  target_compile_options(mock_lc_serial PRIVATE -Wall -Wextra -Wpedantic)
endif()
