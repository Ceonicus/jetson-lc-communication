#include "jetson_lc_protocol/crc16_modbus.hpp"

namespace jetson_lc_protocol
{
std::uint16_t crc16_modbus(const std::uint8_t * data, const std::size_t size) noexcept
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<std::uint16_t>((crc >> 1U) ^ 0xA001U) :
        static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}
}
