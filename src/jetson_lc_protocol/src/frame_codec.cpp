#include "jetson_lc_protocol/protocol.hpp"

#include "jetson_lc_protocol/crc16_modbus.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

namespace jetson_lc_protocol
{
std::vector<std::uint8_t> encode_frame(const Frame & frame)
{
  if (frame.data.size() > 253U) {
    throw std::invalid_argument("frame data is too large");
  }
  std::vector<std::uint8_t> bytes{kHeaderByte, kHeaderByte,
    static_cast<std::uint8_t>(frame.data.size() + 2U), frame.lift_id, frame.function_code};
  bytes.insert(bytes.end(), frame.data.begin(), frame.data.end());
  const auto crc = crc16_modbus(bytes.data() + 2, bytes.size() - 2);
  bytes.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  bytes.push_back(static_cast<std::uint8_t>(crc >> 8U));
  return bytes;
}

std::vector<std::uint8_t> encode_poll_request(
  const std::uint8_t lift_id, const std::uint8_t function_code)
{
  if (function_code != kFunctionConfiguration && function_code != kFunctionStatus) {
    throw std::invalid_argument("unsupported poll function code");
  }
  return encode_frame(Frame{lift_id, function_code, {0x00}});
}

ParseResult FrameParser::append(const std::uint8_t * data, const std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
  ParseResult result;
  static constexpr std::array<std::uint8_t, 2> header_bytes{kHeaderByte, kHeaderByte};
  while (true) {
    auto header = std::search(buffer_.begin(), buffer_.end(),
        header_bytes.begin(), header_bytes.end());
    if (header == buffer_.end()) {
      const bool keep_a5 = !buffer_.empty() && buffer_.back() == kHeaderByte;
      buffer_.assign(keep_a5 ? 1U : 0U, kHeaderByte);
      break;
    }
    buffer_.erase(buffer_.begin(), header);
    if (buffer_.size() < 3U) {break;}
    const std::size_t data_length = buffer_[2];
    if (data_length < 2U || data_length > 34U) {
      ++result.malformed_frames;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t frame_size = 2U + 1U + data_length + 2U;
    if (buffer_.size() < frame_size) {break;}
    const auto expected_crc = crc16_modbus(buffer_.data() + 2, 1U + data_length);
    const auto received_crc = static_cast<std::uint16_t>(buffer_[frame_size - 2U]) |
      static_cast<std::uint16_t>(buffer_[frame_size - 1U] << 8U);
    if (expected_crc != received_crc) {
      ++result.crc_errors;
      buffer_.erase(buffer_.begin());
      continue;
    }
    Frame frame{buffer_[3], buffer_[4], {}};
    frame.data.assign(buffer_.begin() + 5, buffer_.begin() + 3 + data_length);
    result.frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }
  return result;
}
}
