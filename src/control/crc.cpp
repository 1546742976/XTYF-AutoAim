#include "autoaim/control/crc.hpp"
#include <stdexcept>

namespace autoaim::control {
std::uint16_t crc16(const std::uint8_t* data, std::size_t size) {
  if (!data && size != 0) throw std::invalid_argument("Null CRC input");
  std::uint16_t crc = 0xffff;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit)
      crc = static_cast<std::uint16_t>((crc >> 1) ^ ((crc & 1) ? 0x8408 : 0));
  }
  return crc;
}
bool check_crc16(const std::vector<std::uint8_t>& packet) {
  if (packet.size() < 2) return false;
  const auto size = packet.size();
  const auto expected = static_cast<std::uint16_t>(packet[size - 2] | (packet[size - 1] << 8));
  return crc16(packet.data(), size - 2) == expected;
}
void append_crc16(std::vector<std::uint8_t>& packet) {
  const auto crc = crc16(packet.data(), packet.size());
  packet.push_back(static_cast<std::uint8_t>(crc));
  packet.push_back(static_cast<std::uint8_t>(crc >> 8));
}
}  // namespace autoaim::control
