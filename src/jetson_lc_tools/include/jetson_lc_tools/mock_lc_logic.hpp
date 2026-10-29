#pragma once

#include "jetson_lc_protocol/protocol.hpp"

#include <cstdint>
#include <optional>

namespace jetson_lc_tools
{
class MockLcLogic
{
public:
  explicit MockLcLogic(std::uint8_t lift_id = 1) : lift_id_(lift_id) {}
  std::optional<jetson_lc_protocol::Frame> reply(const jetson_lc_protocol::Frame & request);
  std::uint32_t hold_count() const noexcept {return hold_count_;}
  void drop_next() noexcept {drop_next_ = true;}

private:
  std::uint8_t lift_id_;
  std::uint8_t session_state_{};
  std::uint32_t hold_count_{};
  bool drop_next_{};
};
}
