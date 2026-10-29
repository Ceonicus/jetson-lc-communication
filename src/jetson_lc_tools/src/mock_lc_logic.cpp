#include "jetson_lc_tools/mock_lc_logic.hpp"

#include <vector>

namespace jetson_lc_tools
{
std::optional<jetson_lc_protocol::Frame> MockLcLogic::reply(
  const jetson_lc_protocol::Frame & request)
{
  if (drop_next_) {drop_next_ = false; return std::nullopt;}
  if (request.lift_id != lift_id_) {return std::nullopt;}
  if (request.function_code == jetson_lc_protocol::kFunctionConfiguration &&
    request.data == std::vector<std::uint8_t>{0})
  {
    std::vector<std::uint8_t> data(32, 0);
    data[0] = 0x03; data[1] = 0x84; data[2] = 0x06; data[3] = 0x40;
    data[4] = 0x05; data[5] = 0x78; data[6] = 0x03; data[7] = 0xE8;
    data[10] = 0xEF; data[11] = 0xFF;
    return jetson_lc_protocol::Frame{lift_id_, request.function_code, std::move(data)};
  }
  if (request.function_code == jetson_lc_protocol::kFunctionStatus &&
    request.data == std::vector<std::uint8_t>{0})
  {
    const std::uint8_t flags = session_state_ == 0x06U ? 0x13U : 0x02U;
    auto response = jetson_lc_protocol::Frame{lift_id_, request.function_code,
      {16, flags, session_state_, 0}};
    if (session_state_ == 0x07U || session_state_ == 0x08U) {++session_state_;}
    return response;
  }
  if (request.function_code == jetson_lc_protocol::kFunctionCommand && request.data.size() == 3U) {
    const bool automatic_request = (request.data[1] & 0x20U) != 0U;
    if (!automatic_request && request.data[2] == 0U) {
      session_state_ = 0x07;
    } else if (automatic_request && request.data[2] == 0x02U) {
      session_state_ = 0x0B;
    } else if (automatic_request && request.data[2] == 0x01U) {
      ++hold_count_;
      if (session_state_ < 0x06U) {
        static constexpr std::uint8_t progression[]{0x01, 0x02, 0x04, 0x05, 0x06};
        session_state_ = progression[(hold_count_ - 1U) < 5U ? hold_count_ - 1U : 4U];
      }
    } else {
      return std::nullopt;
    }
    return jetson_lc_protocol::Frame{lift_id_, request.function_code, {0}};
  }
  return std::nullopt;
}
}
