#include "jetson_lc_driver/serial_transport.hpp"

namespace jetson_lc_driver
{
SerialTransport::SerialTransport(boost::asio::io_context & io) : port_(io) {}

void SerialTransport::open(const std::string & device, const unsigned int baud_rate)
{
  port_.open(device);
  port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  port_.set_option(boost::asio::serial_port_base::character_size(8));
  port_.set_option(boost::asio::serial_port_base::parity(
      boost::asio::serial_port_base::parity::none));
  port_.set_option(boost::asio::serial_port_base::stop_bits(
      boost::asio::serial_port_base::stop_bits::one));
  port_.set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
}

void SerialTransport::close()
{
  boost::system::error_code ignored;
  port_.cancel(ignored);
  port_.close(ignored);
}

bool SerialTransport::is_open() const noexcept {return port_.is_open();}

void SerialTransport::start_read(ReadHandler handler)
{
  read_handler_ = std::move(handler);
  read_next();
}

void SerialTransport::read_next()
{
  port_.async_read_some(boost::asio::buffer(read_buffer_),
    [this](const boost::system::error_code & error, const std::size_t size) {
      read_handler_(error, read_buffer_.data(), size);
      if (!error && port_.is_open()) {read_next();}
    });
}

void SerialTransport::async_write(std::vector<std::uint8_t> bytes, WriteHandler handler)
{
  write_buffer_ = std::move(bytes);
  boost::asio::async_write(port_, boost::asio::buffer(write_buffer_), std::move(handler));
}
}
