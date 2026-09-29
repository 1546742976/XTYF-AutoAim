#pragma once

#include "autoaim/core/time.hpp"
#include <optional>

namespace autoaim::control {
struct SmootherOptions {
  core::Radians maximum_correction;
  double maximum_rate_radps;
  core::Seconds time_constant;
  core::Radians enter_threshold;
  core::Radians leave_threshold;
  core::Seconds minimum_dwell;
};

// 每实例一个相对修正轴，由单写者调用。正常退出可平滑归零；紧急撤销必须旁路。
class CorrectionSmoother {
public:
  explicit CorrectionSmoother(SmootherOptions options);
  core::Radians update(core::Radians requested, core::TimePoint now, bool enabled);
  core::Radians emergency_stop(core::TimePoint now);
private:
  SmootherOptions options_;
  double value_ = 0;
  bool active_ = false;
  std::optional<core::TimePoint> previous_;
  std::optional<core::TimePoint> transition_;
};
}  // namespace autoaim::control
