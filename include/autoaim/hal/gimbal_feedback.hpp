#pragma once

#include "autoaim/core/evidence.hpp"
#include "autoaim/core/types.hpp"
#include <array>
#include <optional>

namespace autoaim::hal {
struct OperatorInput {
  core::TimePoint sampled_at;
  bool valid;
  bool intervening;
  std::uint64_t enable_event;  // 单调事件号；按住按钮不能反复产生重新启用事件。
  core::Evidence source_evidence;
};
struct GimbalFeedback {
  const core::TimePoint sampled_at;
  const core::Generation generation;
  const std::array<double, 4> quaternion_wxyz;
  const double yaw_rad;
  const double pitch_rad;
  const double bullet_speed_mps;
  const bool pose_valid;
  const bool status_valid;
  const std::optional<OperatorInput> operator_input;
};
}  // namespace autoaim::hal
