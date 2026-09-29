#include "autoaim/estimation/state_machine.hpp"

namespace autoaim::estimation {
TrackingStateMachine::TrackingStateMachine(TrackingLimits limits) : limits_(limits) {
  if (limits.minimum_observations < 2 || limits.minimum_convergence_time.value() < 0 ||
      limits.maximum_observation_gap.value() <= 0 ||
      limits.lost_after.value() <= limits.maximum_observation_gap.value() ||
      !std::isfinite(limits.phase_variance_enter) || limits.phase_variance_enter <= 0 ||
      !std::isfinite(limits.phase_variance_exit) ||
      limits.phase_variance_exit < limits.phase_variance_enter)
    throw std::invalid_argument("Invalid tracking lifecycle limits");
}

void TrackingStateMachine::interrupt_quality() {
  quality_ = TrackingQuality::degraded;
  good_hits_ = 0;
  good_since_.reset();
}

void TrackingStateMachine::tick(core::TimePoint now) {
  if (!last_seen_ || !core::fresh(*last_seen_, now, limits_.lost_after)) {
    lifecycle_ = TrackLifecycle::lost;
    hits_ = 0;
    interrupt_quality();
  } else if (!core::fresh(*last_seen_, now, limits_.maximum_observation_gap)) {
    lifecycle_ = TrackLifecycle::coasting;
    hits_ = 0;
    interrupt_quality();
  }
}

bool TrackingStateMachine::observe(const core::Stamp& source, const QualityFacts& facts) {
  if (last_processed_ && (source.generation != last_processed_->generation ||
                          source.frame_id <= last_processed_->frame_id ||
                          source.exposure.domain() != last_processed_->exposure.domain() ||
                          core::elapsed(source.exposure, last_processed_->exposure).value() <= 0))
    return false;

  tick(source.exposure);
  last_processed_.emplace(source);

  if (facts.fault) {
    lifecycle_ = TrackLifecycle::lost;
    last_seen_.reset();
    hits_ = 0;
    interrupt_quality();

    return true;
  }

  if (!facts.update_accepted) {
    if (lifecycle_ != TrackLifecycle::lost)
      lifecycle_ = TrackLifecycle::coasting;

    hits_ = 0;
    interrupt_quality();

    return true;
  }

  last_seen_ = source.exposure;

  if (hits_ < limits_.minimum_observations)
    ++hits_;

  lifecycle_ =
      hits_ >= limits_.minimum_observations ? TrackLifecycle::tracking : TrackLifecycle::acquiring;

  const double phase_limit = quality_ == TrackingQuality::converged ? limits_.phase_variance_exit
                                                                    : limits_.phase_variance_enter;

  const bool good = facts.pose_reliable && facts.identity_known && facts.geometry_supported &&
                    std::isfinite(facts.phase_variance) && facts.phase_variance >= 0 &&
                    facts.phase_variance <= phase_limit;

  if (!good) {
    interrupt_quality();

    return true;
  }

  if (!good_since_)
    good_since_ = source.exposure;

  if (good_hits_ < limits_.minimum_observations)
    ++good_hits_;

  if (good_hits_ >= limits_.minimum_observations &&
      core::elapsed(source.exposure, *good_since_).value() >=
          limits_.minimum_convergence_time.value())
    quality_ = TrackingQuality::converged;
  else
    quality_ = TrackingQuality::unconverged;

  return true;
}

MotionSelector::MotionSelector(MotionSelectionLimits limits) : limits_(limits) {
  if (!std::isfinite(limits.rotation_threshold_radps) || limits.rotation_threshold_radps <= 0 ||
      !std::isfinite(limits.acceleration_enter_radps2) ||
      !std::isfinite(limits.acceleration_exit_radps2) || limits.acceleration_exit_radps2 < 0 ||
      limits.acceleration_enter_radps2 <= limits.acceleration_exit_radps2 ||
      limits.minimum_observations < 2 || limits.minimum_dwell.value() < 0 ||
      limits.maximum_gap.value() <= 0)
    throw std::invalid_argument("Invalid motion selection hysteresis");
}

bool MotionSelector::observe(const core::Stamp& source, double omega, bool healthy) {
  if (previous_ &&
      (source.generation != previous_->generation || source.frame_id <= previous_->frame_id ||
       source.exposure.domain() != previous_->exposure.domain() ||
       core::elapsed(source.exposure, previous_->exposure).value() <= 0))
    return false;

  if (!switched_at_)
    switched_at_ = source.exposure;

  if (!healthy || !std::isfinite(omega) || !previous_ ||
      !core::fresh(previous_->exposure, source.exposure, limits_.maximum_gap)) {
    regime_ = MotionRegime::uncertain;
    support_ = 0;
  } else {
    const double acceleration = std::abs(omega - previous_omega_) /
                                core::elapsed(source.exposure, previous_->exposure).value();

    regime_ = acceleration > limits_.acceleration_enter_radps2     ? MotionRegime::uncertain
              : std::abs(omega) < limits_.rotation_threshold_radps ? MotionRegime::slow
                                                                   : MotionRegime::steady_rotation;

    const bool switch_requested = kind_ == MotionKind::constant_velocity
                                      ? acceleration > limits_.acceleration_enter_radps2
                                      : acceleration < limits_.acceleration_exit_radps2;

    if (switch_requested) {
      if (support_ < limits_.minimum_observations)
        ++support_;
    } else
      support_ = 0;

    if (support_ >= limits_.minimum_observations &&
        core::elapsed(source.exposure, *switched_at_).value() >= limits_.minimum_dwell.value()) {
      kind_ = kind_ == MotionKind::constant_velocity ? MotionKind::bounded_acceleration
                                                     : MotionKind::constant_velocity;

      support_ = 0;
      switched_at_ = source.exposure;
    }
  }

  // 非健康样本中断差分，下一帧不能跨故障计算一个伪加速度。
  if (healthy && std::isfinite(omega)) {
    previous_.emplace(source);
    previous_omega_ = omega;
  } else
    previous_.reset();

  return true;
}
} // namespace autoaim::estimation
