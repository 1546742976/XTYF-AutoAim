#pragma once

#include "autoaim/control/command.hpp"
#include "autoaim/core/result.hpp"
#include <array>
#include <vector>

namespace autoaim::control {
enum class ProtocolKind { gimbal_ab, cboard_uart, cboard_uart_v2, cboard_can };
core::Result<std::vector<std::uint8_t>> encode(const Command& command, ProtocolKind protocol);

struct GimbalPacket {
  std::uint8_t mode;
  std::array<double, 4> quaternion_wxyz;
  double yaw_rad;
  double yaw_rate_radps;
  double pitch_rad;
  double pitch_rate_radps;
  double bullet_speed_mps;
  std::uint16_t bullet_count;
};
core::Result<GimbalPacket> decode_gimbal(const std::vector<std::uint8_t>& packet);
struct CanStatusPacket {
  double bullet_speed_mps;
  std::uint8_t mode;
  std::uint8_t shoot_mode;
  double ft_angle_rad;
};
// 旧 CAN 两类反馈 payload 独立解码；适配层必须保留各自采样时间，不因另一类刷新而续期。
core::Result<std::array<double, 4>> decode_can_quaternion(const std::vector<std::uint8_t>& payload);
core::Result<CanStatusPacket> decode_can_status(const std::vector<std::uint8_t>& payload);

// 固定 43 字节反馈帧；坏 CRC 后逐字节重同步，跨调用保留半包，缓存不超过 43 字节。
class GimbalStreamParser {
public:
  std::vector<GimbalPacket> feed(const std::vector<std::uint8_t>& bytes);
  std::size_t buffered() const noexcept { return buffer_.size(); }
  std::uint64_t rejected() const noexcept { return rejected_; }
private:
  std::vector<std::uint8_t> buffer_;
  std::uint64_t rejected_ = 0;
};
}  // namespace autoaim::control
