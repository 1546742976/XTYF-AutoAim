#include "autoaim/decision/aim_adequacy.hpp"
#include "autoaim/math/angle.hpp"
#include <cmath>
#include <stdexcept>

namespace autoaim::decision {
AimAngles direction_to_angles(const Eigen::Vector3d& direction) {
  if (!direction.allFinite() || direction.norm() < 1e-12)
    throw std::invalid_argument("Invalid aim direction");
  return {core::Radians(std::atan2(direction.y(), direction.x())),
    core::Radians(std::atan2(direction.z(), std::hypot(direction.x(), direction.y())))};
}
AimAdequacy::AimAdequacy(AimSafetyLimits limits) : limits_(limits) {
  if (limits.yaw_tolerance.value() <= 0 || limits.pitch_tolerance.value() <= 0 ||
      limits.minimum_observations == 0 || limits.minimum_duration.value() < 0 ||
      limits.maximum_gap.value() <= 0 || limits.source_age.value() <= 0 || limits.feedback_age.value() <= 0)
    throw std::invalid_argument("Invalid aim safety limits");
}
void AimAdequacy::clear_support() { first_.reset(); count_ = 0; }
void AimAdequacy::reset() { clear_support(); last_.reset(); key_.reset(); }
AimAdequacyReport AimAdequacy::evaluate(const core::Stamp& source, const AimKey& key,
    AimAngles desired, const MeasuredPointing& measured, core::TimePoint now) {
  const bool feedback_ok = measured.valid && measured.generation == source.generation &&
    core::fresh(measured.sampled_at, now, limits_.feedback_age);
  const double yaw = std::atan2(std::sin(desired.yaw.value() - measured.angles.yaw.value()),
    std::cos(desired.yaw.value() - measured.angles.yaw.value()));
  const double pitch = desired.pitch.value() - measured.angles.pitch.value();
  const bool same_key = key_ && key_->target == key.target && key_->profile == key.profile && key_->plate == key.plate;
  const bool same_epoch = last_ && last_->generation == source.generation && last_->exposure.domain() == source.exposure.domain();
  const bool duplicate = same_epoch && source.frame_id == last_->frame_id && core::same_time(source.exposure, last_->exposure);
  const bool newer = !last_ || (same_epoch && source.frame_id > last_->frame_id &&
    source.exposure.nanoseconds() > last_->exposure.nanoseconds());
  const bool valid = feedback_ok && core::fresh(source.exposure, now, limits_.source_age) &&
    std::abs(yaw) <= limits_.yaw_tolerance.value() && std::abs(pitch) <= limits_.pitch_tolerance.value();
  if (!same_key || !valid || (!newer && !duplicate)) clear_support();
  if (newer) {
    if (last_ && !core::fresh(last_->exposure, source.exposure, limits_.maximum_gap)) clear_support();
    last_.emplace(source);
    key_ = key;
    if (valid) {
      if (!first_) first_ = source.exposure;
      ++count_;
    }
  }
  // 不清除 last_：同帧坏反馈后再变好，也不能重新积累或恢复许可证据。
  const bool settled = valid && same_key && (newer || duplicate) && first_ &&
    count_ >= limits_.minimum_observations &&
    core::elapsed(source.exposure, *first_).value() >= limits_.minimum_duration.value();
  return {source, feedback_ok, yaw, pitch, count_, settled};
}
}  // namespace autoaim::decision
