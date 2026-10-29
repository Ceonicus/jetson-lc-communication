#pragma once

#include <cstddef>
#include <cstdint>

namespace jetson_lc_protocol
{
std::uint16_t crc16_modbus(const std::uint8_t * data, std::size_t size) noexcept;
}
