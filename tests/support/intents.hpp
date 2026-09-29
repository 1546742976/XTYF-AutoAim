#pragma once
#include "autoaim/control/control_intent.hpp"

namespace test {
inline autoaim::control::ControlIntent
intent(autoaim::core::FrameId id, autoaim::core::Generation generation,
       autoaim::core::TimePoint source,
       autoaim::core::ControlMode mode = autoaim::core::ControlMode::automatic) {
  using namespace autoaim;
  auto evidence = core::Evidence::from_report(core::MeasurementReport::evaluate(
      {"sim", "v1", "2026-09-29", "synthetic fixture"}, {0, 0}, 0, core::ClockDomain::replay));

  return control::ControlIntent(
      core::Stamp(id, generation, source), core::Role::infantry, core::Task::armor, mode,
      mode == core::ControlMode::assist ? core::CommandSpace::relative
                                        : core::CommandSpace::absolute,
      true, true, {core::Radians(0), core::Radians(0), 0, 0, 0, 0}, source,
      core::advance(source, core::Seconds(1)),
      {core::Stamp(id, generation, source), source, true, true, true, true, true, true}, evidence,
      core::Metres(3));
}
} // namespace test
