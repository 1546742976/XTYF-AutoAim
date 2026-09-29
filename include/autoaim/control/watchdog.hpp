#pragma once

#include "autoaim/control/control_intent.hpp"

namespace autoaim::control {
struct TimingLimits {
  core::Seconds control_age;
  core::Seconds fire_age;
  core::Seconds feedback_age;
};
struct TimeVerdict { bool control_fresh; bool fire_fresh; };

// 无线程的主机时效判据，不声称实现下位机独立通信看门狗。
class Watchdog {
public:
  explicit Watchdog(TimingLimits limits);
  TimeVerdict evaluate(const ControlIntent& intent, core::TimePoint now) const;
  bool feedback_fresh(core::TimePoint sample, core::TimePoint now, core::TimePoint mode_since) const;
private:
  TimingLimits limits_;
};
}  // namespace autoaim::control
