#pragma once

#include "autoaim/estimation/target_snapshot.hpp"

namespace autoaim::estimation {
enum class TrackLifecycle { acquiring, tracking, coasting, lost };

struct TrackingLimits {
  std::size_t minimum_observations;
  core::Seconds minimum_convergence_time;
  core::Seconds maximum_observation_gap;
  core::Seconds lost_after;
  double phase_variance_enter;
  double phase_variance_exit;
};

struct QualityFacts {
  bool update_accepted;
  bool pose_reliable;
  bool identity_known;
  bool geometry_supported;
  bool fault;
  double phase_variance;
};

class TrackingStateMachine {
public:
  explicit TrackingStateMachine(TrackingLimits limits);
  bool observe(const core::Stamp& source, const QualityFacts& facts);

  // 无观测时也调用；不刷新源时刻，超时立即撤销可靠性。
  void tick(core::TimePoint now);

  TrackLifecycle lifecycle() const noexcept {
    return lifecycle_;
  }

  TrackingQuality quality() const noexcept {
    return quality_;
  }

private:
  void interrupt_quality();
  TrackingLimits limits_;
  TrackLifecycle lifecycle_ = TrackLifecycle::lost;
  TrackingQuality quality_ = TrackingQuality::unconverged;
  std::optional<core::Stamp> last_processed_;
  std::optional<core::TimePoint> last_seen_;
  std::optional<core::TimePoint> good_since_;
  std::size_t hits_ = 0;
  std::size_t good_hits_ = 0;
};

enum class MotionRegime { slow, steady_rotation, uncertain };

struct MotionSelectionLimits {
  double rotation_threshold_radps;
  double acceleration_enter_radps2;
  double acceleration_exit_radps2;
  std::size_t minimum_observations;
  core::Seconds minimum_dwell;
  core::Seconds maximum_gap;
};

class MotionSelector {
public:
  explicit MotionSelector(MotionSelectionLimits limits);
  bool observe(const core::Stamp& source, double estimated_omega_radps, bool healthy);

  MotionKind kind() const noexcept {
    return kind_;
  }

  MotionRegime regime() const noexcept {
    return regime_;
  }

private:
  MotionSelectionLimits limits_;
  MotionKind kind_ = MotionKind::constant_velocity;
  MotionRegime regime_ = MotionRegime::uncertain;
  std::optional<core::Stamp> previous_;
  std::optional<core::TimePoint> switched_at_;
  double previous_omega_ = 0;
  std::size_t support_ = 0;
};
} // namespace autoaim::estimation
