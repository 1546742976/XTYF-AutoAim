#include "autoaim/control/protocol.hpp"
#include "autoaim/control/crc.hpp"
#include <algorithm>
#include <cstring>
#include <limits>

namespace autoaim::control {
namespace {
void append_u16(std::vector<std::uint8_t>& bytes, std::uint16_t value, bool big_endian = false) {
  bytes.push_back(static_cast<std::uint8_t>(big_endian ? value >> 8 : value));
  bytes.push_back(static_cast<std::uint8_t>(big_endian ? value : value >> 8));
}
void append_float(std::vector<std::uint8_t>& bytes, double value) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
    throw std::invalid_argument("Wire float out of range");
  const float narrowed = static_cast<float>(value);
  std::uint32_t bits;
  std::memcpy(&bits, &narrowed, 4);
  for (int shift = 0; shift < 32; shift += 8) bytes.push_back(static_cast<std::uint8_t>(bits >> shift));
}
double read_float(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  std::uint32_t bits = 0;
  for (int index = 0; index < 4; ++index) bits |= static_cast<std::uint32_t>(bytes.at(offset + index)) << (8 * index);
  float value;
  std::memcpy(&value, &bits, 4);
  return value;
}
}  // namespace

core::Result<std::vector<std::uint8_t>> encode(const Command& command, ProtocolKind protocol) {
  using Result = core::Result<std::vector<std::uint8_t>>;
  if (command.control_enabled && command.space == core::CommandSpace::relative)
    return Result::failure(core::ErrorCode::unavailable, "Relative-control wire mapping is unverified");
  try {
    std::vector<std::uint8_t> bytes;
    const auto& p = command.pointing;
    const bool enabled = command.control_enabled;
    if (protocol == ProtocolKind::gimbal_ab) {
      bytes = {'A', 'B', static_cast<std::uint8_t>(enabled ? (command.shoot ? 2 : 1) : 0)};
      for (const double value : {p.yaw.value(), p.yaw_rate_radps, p.yaw_acceleration_radps2,
                                 p.pitch.value(), p.pitch_rate_radps, p.pitch_acceleration_radps2})
        append_float(bytes, enabled ? value : 0);
      append_crc16(bytes);
    } else if (protocol == ProtocolKind::cboard_uart || protocol == ProtocolKind::cboard_uart_v2 ||
               protocol == ProtocolKind::cboard_can) {
      if (enabled && (std::abs(p.yaw.value()) > 3.2767 || std::abs(p.pitch.value()) > 3.2767))
        return Result::failure(core::ErrorCode::invalid_input, "Fixed-point angle out of range");
      const bool can = protocol == ProtocolKind::cboard_can;
      const bool v2 = protocol == ProtocolKind::cboard_uart_v2;
      if (!can) bytes.push_back(0xa5);
      if (v2) bytes.push_back(14);
      bytes.push_back(enabled ? 1 : 0);
      bytes.push_back(command.shoot && enabled ? 1 : 0);
      // 距离饱和及截断取整沿用旧 CBoard 协议；CAN 大端，UART 小端。
      for (double value : {p.yaw.value(), p.pitch.value(), std::clamp(command.horizontal_distance.value(), 0.0, 3.2767)}) {
        const auto fixed = static_cast<std::int16_t>(enabled ? value * 1e4 : 0);
        append_u16(bytes, static_cast<std::uint16_t>(fixed), can);
      }
      if (!can) append_crc16(bytes);
      if (v2) append_u16(bytes, 0x7891);
    } else {
      return Result::failure(core::ErrorCode::invalid_input, "Unknown wire protocol");
    }
    return Result::success(std::move(bytes));
  } catch (const std::invalid_argument& error) {
    return Result::failure(core::ErrorCode::invalid_input, error.what());
  }
}

core::Result<GimbalPacket> decode_gimbal(const std::vector<std::uint8_t>& bytes) {
  using Result = core::Result<GimbalPacket>;
  if (bytes.size() != 43 || bytes[0] != 'A' || bytes[1] != 'B' || bytes[2] > 3 || !check_crc16(bytes))
    return Result::failure(core::ErrorCode::invalid_input, "Invalid gimbal packet");
  GimbalPacket packet{bytes[2], {read_float(bytes, 3), read_float(bytes, 7), read_float(bytes, 11),
    read_float(bytes, 15)}, read_float(bytes, 19), read_float(bytes, 23), read_float(bytes, 27),
    read_float(bytes, 31), read_float(bytes, 35), static_cast<std::uint16_t>(bytes[39] | (bytes[40] << 8))};
  // 这里只解字节；四元数坐标语义与弹速/反馈有效性由适配层分别核对。
  return Result::success(packet);
}

std::vector<GimbalPacket> GimbalStreamParser::feed(const std::vector<std::uint8_t>& bytes) {
  std::vector<GimbalPacket> packets;
  for (auto byte : bytes) {
    buffer_.push_back(byte);
    while (buffer_.size() >= 2 && (buffer_[0] != 'A' || buffer_[1] != 'B')) buffer_.erase(buffer_.begin());
    if (buffer_.size() < 43) continue;
    auto decoded = decode_gimbal(buffer_);
    if (decoded) { packets.push_back(decoded.value()); buffer_.clear(); }
    else { ++rejected_; buffer_.erase(buffer_.begin()); }
  }
  return packets;
}
namespace {
double signed_big_endian(const std::vector<std::uint8_t>& bytes, std::size_t offset, double scale) {
  const auto raw = static_cast<unsigned>(bytes[offset]) * 256 + bytes[offset + 1];
  const int value = raw >= 32768 ? static_cast<int>(raw) - 65536 : static_cast<int>(raw);
  return value / scale;
}
}  // namespace
core::Result<std::array<double, 4>> decode_can_quaternion(const std::vector<std::uint8_t>& bytes) {
  using Result = core::Result<std::array<double, 4>>;
  if (bytes.size() != 8) return Result::failure(core::ErrorCode::invalid_input, "CAN quaternion requires 8 bytes");
  // 线序 x,y,z,w；公共 DTO 为 w,x,y,z。只解码，不宣称坐标约定已实测。
  return Result::success({signed_big_endian(bytes, 6, 1e4), signed_big_endian(bytes, 0, 1e4),
    signed_big_endian(bytes, 2, 1e4), signed_big_endian(bytes, 4, 1e4)});
}
core::Result<CanStatusPacket> decode_can_status(const std::vector<std::uint8_t>& bytes) {
  using Result = core::Result<CanStatusPacket>;
  if (bytes.size() < 6 || bytes.size() > 8 || bytes[2] > 4 || bytes[3] > 2)
    return Result::failure(core::ErrorCode::invalid_input, "Invalid CAN status payload/mode");
  return Result::success({signed_big_endian(bytes, 0, 1e2), bytes[2], bytes[3], signed_big_endian(bytes, 4, 1e4)});
}
}  // namespace autoaim::control
