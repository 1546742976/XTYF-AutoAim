#pragma once

#include "autoaim/core/types.hpp"
#include <Eigen/Core>
#include <optional>
#include <string>

namespace autoaim::decision {
// 瞄准参考系为 x 前、y 左、z 上；单位 rad。不是相机光轴坐标。
struct AimAngles { core::Radians yaw; core::Radians pitch; };
AimAngles direction_to_angles(const Eigen::Vector3d& direction_reference);
struct MeasuredPointing {
  core::TimePoint sampled_at;
  core::Generation generation;
  AimAngles angles;
  bool valid;
};
struct AimKey {
  std::uint64_t target;
  std::string profile;
  std::optional<std::size_t> plate;
};
// 由安全配置装配，不能由轨迹生产者逐次放宽。
struct AimSafetyLimits {
  core::Radians yaw_tolerance;
  core::Radians pitch_tolerance;
  std::size_t minimum_observations;
  core::Seconds minimum_duration;
  core::Seconds maximum_gap;
  core::Seconds source_age;
  core::Seconds feedback_age;
};
struct AimAdequacyReport {
  const core::Stamp source;
  bool feedback_fresh;
  double yaw_error_rad;
  double pitch_error_rad;
  std::size_t supporting_observations;
  bool settled;
};
// 单估计线程调用。只输出证据，不产生开火许可；重复发送不增加支持帧数。
class AimAdequacy {
public:
  explicit AimAdequacy(AimSafetyLimits limits);
  AimAdequacyReport evaluate(const core::Stamp& source, const AimKey& key,
      AimAngles desired, const MeasuredPointing& measured, core::TimePoint now);
  void reset();
private:
  void clear_support();
  AimSafetyLimits limits_;
  std::optional<core::Stamp> last_;
  std::optional<AimKey> key_;
  std::optional<core::TimePoint> first_;
  std::size_t count_ = 0;
};
}  // namespace autoaim::decision
