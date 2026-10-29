#include "jetson_lc_tools/mock_lc_logic.hpp"

#include "jetson_lc_protocol/protocol.hpp"

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
void make_raw(const int fd)
{
  termios settings{};
  if (tcgetattr(fd, &settings) != 0) {throw std::runtime_error("tcgetattr failed");}
  cfmakeraw(&settings);
  if (tcsetattr(fd, TCSANOW, &settings) != 0) {throw std::runtime_error("tcsetattr failed");}
}

void serve(const int master, std::atomic_bool & running, const int delay_ms,
  const int drop_every, const int bad_crc_every)
{
  jetson_lc_protocol::FrameParser parser;
  jetson_lc_tools::MockLcLogic logic;
  std::uint32_t request_count = 0;
  std::uint8_t buffer[256];
  while (running) {
    pollfd descriptor{master, POLLIN, 0};
    if (poll(&descriptor, 1, 100) <= 0) {continue;}
    const auto size = read(master, buffer, sizeof(buffer));
    if (size <= 0) {break;}
    auto parsed = parser.append(buffer, static_cast<std::size_t>(size));
    for (const auto & frame : parsed.frames) {
      ++request_count;
      if (drop_every > 0 && request_count % static_cast<unsigned>(drop_every) == 0U) {continue;}
      const auto response = logic.reply(frame);
      if (!response) {continue;}
      if (delay_ms > 0) {std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));}
      auto bytes = jetson_lc_protocol::encode_frame(*response);
      if (bad_crc_every > 0 && request_count % static_cast<unsigned>(bad_crc_every) == 0U) {
        bytes.back() ^= 0xFFU;
      }
      (void)write(master, bytes.data(), bytes.size());
    }
  }
}

bool exchange(const int slave, const std::vector<std::uint8_t> & request,
  const std::uint8_t expected_function)
{
  if (write(slave, request.data(), request.size()) != static_cast<ssize_t>(request.size())) {return false;}
  pollfd descriptor{slave, POLLIN, 0};
  if (poll(&descriptor, 1, 200) <= 0) {return false;}
  std::uint8_t bytes[128];
  const auto size = read(slave, bytes, sizeof(bytes));
  if (size <= 0) {return false;}
  jetson_lc_protocol::FrameParser parser;
  const auto result = parser.append(bytes, static_cast<std::size_t>(size));
  return result.frames.size() == 1U && result.frames.front().function_code == expected_function;
}

int self_test()
{
  int master = -1;
  int slave = -1;
  if (openpty(&master, &slave, nullptr, nullptr, nullptr) != 0) {return 1;}
  make_raw(slave);
  std::atomic_bool running{true};
  std::thread server(serve, master, std::ref(running), 0, 4, 0);
  const bool status = exchange(slave, jetson_lc_protocol::encode_poll_request(1, 0x11), 0x11);
  const bool configuration = exchange(slave,
      jetson_lc_protocol::encode_poll_request(1, 0x10), 0x10);
  const bool command = exchange(slave, jetson_lc_protocol::encode_command_request(1,
      {0, true, jetson_lc_protocol::ManualMotion::Stop,
       jetson_lc_protocol::MaintenanceRequest::StartHold}), 0x12);
  const bool dropped = !exchange(slave, jetson_lc_protocol::encode_poll_request(1, 0x11), 0x11);
  running = false;
  close(slave);
  close(master);
  server.join();
  if (status && configuration && command && dropped) {
    std::cout << "mock LC PTY self-test passed\n";
    return 0;
  }
  return 1;
}
}

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--self-test") {return self_test();}
  int delay_ms = 0;
  int drop_every = 0;
  int bad_crc_every = 0;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string option(argv[i]);
    if (option == "--delay-ms") {delay_ms = std::atoi(argv[i + 1]);}
    else if (option == "--drop-every") {drop_every = std::atoi(argv[i + 1]);}
    else if (option == "--bad-crc-every") {bad_crc_every = std::atoi(argv[i + 1]);}
    else {std::cerr << "unknown option: " << option << '\n'; return 2;}
  }
  int master = -1;
  int slave = -1;
  char name[128]{};
  if (openpty(&master, &slave, name, nullptr, nullptr) != 0) {
    std::cerr << std::strerror(errno) << '\n';
    return 1;
  }
  make_raw(slave);
  std::cout << name << std::endl;
  std::atomic_bool running{true};
  serve(master, running, delay_ms, drop_every, bad_crc_every);
  close(slave);
  close(master);
  return 0;
}
