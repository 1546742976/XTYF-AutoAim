#include "autoaim/hal/clock.hpp"

namespace autoaim::hal {
core::TimePoint MonotonicClock::now() const {
  return core::monotonic_now();
}

ReplayClock::ReplayClock(core::TimePoint start) : time_(start) {
  if (start.domain() != core::ClockDomain::replay)
    throw std::invalid_argument("ReplayClock requires replay domain");
}

core::TimePoint ReplayClock::now() const {
  std::lock_guard<std::mutex> lock(mutex_);

  return time_;
}

void ReplayClock::advance(core::Seconds delta) {
  if (delta.value() < 0)
    throw std::invalid_argument("Replay clock cannot rewind");

  std::lock_guard<std::mutex> lock(mutex_);
  time_ = core::advance(time_, delta);
}

void ReplayClock::advance_to(core::TimePoint time) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (time.domain() != core::ClockDomain::replay || time.nanoseconds() < time_.nanoseconds())
    throw std::invalid_argument("Replay clock cannot rewind or change domain");

  time_ = time;
}
} // namespace autoaim::hal
