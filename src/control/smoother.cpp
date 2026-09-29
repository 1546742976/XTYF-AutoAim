#include "autoaim/control/smoother.hpp"
#include <algorithm>

namespace autoaim::control {
CorrectionSmoother::CorrectionSmoother(SmootherOptions options) : options_(options) {
  if (options.maximum_correction.value() <= 0 || !std::isfinite(options.maximum_rate_radps) ||
      options.maximum_rate_radps <= 0 || options.time_constant.value() < 0 ||
      options.leave_threshold.value() < 0 ||
      options.enter_threshold.value() < options.leave_threshold.value() ||
      options.enter_threshold.value() > options.maximum_correction.value() ||
      options.minimum_dwell.value() < 0)
    throw std::invalid_argument("Invalid smoother limits");
}

core::Radians CorrectionSmoother::update(core::Radians requested, core::TimePoint now,
                                         bool enabled) {
  if (!previous_) {
    previous_ = now;
    transition_ = now;
  }

  const double dt = core::elapsed(now, *previous_).value();

  if (dt < 0)
    throw std::invalid_argument("Smoother time moved backwards");

  previous_ = now;
  const double magnitude = std::abs(requested.value());
  const bool desired = enabled && (active_ ? magnitude > options_.leave_threshold.value()
                                           : magnitude >= options_.enter_threshold.value());

  if (desired != active_ &&
      core::elapsed(now, *transition_).value() >= options_.minimum_dwell.value()) {
    active_ = desired;
    transition_ = now;
  }

  const double limit = options_.maximum_correction.value();
  const double target = enabled && active_ ? std::clamp(requested.value(), -limit, limit) : 0;
  const double gain =
      options_.time_constant.value() == 0 ? 1 : -std::expm1(-dt / options_.time_constant.value());

  const double allowed_delta = options_.maximum_rate_radps * dt;
  value_ += std::clamp(gain * (target - value_), -allowed_delta, allowed_delta);

  return core::Radians(value_);
}

core::Radians CorrectionSmoother::emergency_stop(core::TimePoint now) {
  value_ = 0;
  active_ = false;
  previous_ = now;
  transition_ = now;

  return core::Radians(0);
}
} // namespace autoaim::control
