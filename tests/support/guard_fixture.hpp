#pragma once
#include "autoaim/control/command_guard.hpp"
#include "support/intents.hpp"

namespace test {
inline autoaim::control::CommandGuard guard() {
  using namespace autoaim;

  return control::CommandGuard({{core::Seconds(0.2), core::Seconds(0.1), core::Seconds(0.05)},
                                "sim",
                                "v1",
                                core::Radians(0.02),
                                core::Radians(0.02)});
}

inline autoaim::control::GuardContext
context(autoaim::core::TimePoint now,
        autoaim::core::ControlMode mode = autoaim::core::ControlMode::automatic,
        bool intervening = false, autoaim::core::Generation generation = 0) {
  using namespace autoaim;
  const auto sample = intent(1, generation, now, mode);
  hal::GimbalFeedback feedback{
      now,  generation, {1, 0, 0, 0},
      0,    0,          25,
      true, true,       hal::OperatorInput{now, true, intervening, 1, sample.timing_evidence}};

  return {{core::Role::infantry, core::Task::armor, mode, sample.space, true,
           mode == core::ControlMode::automatic},
          generation,
          core::TimePoint(0, now.domain()),
          feedback,
          sample.timing_evidence,
          false};
}
} // namespace test
