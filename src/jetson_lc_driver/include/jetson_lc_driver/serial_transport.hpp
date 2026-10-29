#pragma once

#include <boost/asio.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace jetson_lc_driver
{
class SerialTransport
{
public:
  using ReadHandler = std::function<void(const boost::system::error_code &,
      const std::uint8_t *, std::size_t)>;
  using WriteHandler = std::function<void(const boost::system::error_code &, std::size_t)>;

  explicit SerialTransport(boost::asio::io_context & io);
  void open(const std::string & device, unsigned int baud_rate);
  void close();
  bool is_open() const noexcept;
  void start_read(ReadHandler handler);
  void async_write(std::vector<std::uint8_t> bytes, WriteHandler handler);

private:
  void read_next();
  boost::asio::serial_port port_;
  std::array<std::uint8_t, 256> read_buffer_{};
  std::vector<std::uint8_t> write_buffer_;
  ReadHandler read_handler_;
};
}
