#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoaim::control {
// 与旧 tools::get_crc16 兼容：CCITT 反射 0x8408，init=ffff，xorout=0。
// 线协议 CRC 低字节在前；不能替换成非反射的 0x1021 版本。
std::uint16_t crc16(const std::uint8_t* data, std::size_t size);
bool check_crc16(const std::vector<std::uint8_t>& packet);
void append_crc16(std::vector<std::uint8_t>& packet);
}  // namespace autoaim::control
