#include "autoaim/control/watchdog.hpp"

namespace autoaim::control {
Watchdog::Watchdog(TimingLimits limits) : limits_(limits) {
  if (limits.fire_age.value() <= 0 || limits.control_age.value() < limits.fire_age.value() ||
      limits.feedback_age.value() <= 0)
    throw std::invalid_argument("Invalid timing limits");
}

TimeVerdict Watchdog::evaluate(const ControlIntent& intent, core::TimePoint now) const {
  if (now.domain() != intent.created_at.domain() ||
      now.nanoseconds() < intent.created_at.nanoseconds() ||
      now.nanoseconds() >= intent.deadline.nanoseconds())
    return {false, false};

  return {core::fresh(intent.source.exposure, now, limits_.control_age),
          core::fresh(intent.source.exposure, now, limits_.fire_age)};
}

bool Watchdog::feedback_fresh(core::TimePoint sample, core::TimePoint now,
                              core::TimePoint since) const {
  return sample.domain() == since.domain() && sample.nanoseconds() > since.nanoseconds() &&
         core::fresh(sample, now, limits_.feedback_age);
}
} // namespace autoaim::control
