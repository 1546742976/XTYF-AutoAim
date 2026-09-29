#include "autoaim/mission/sentry/sentry_mission.hpp"

namespace autoaim::mission {
MissionRequest sentry_request(const AimSolution& solution, core::Generation generation,
    core::TimePoint mode_since, bool program_fire_requested) {
  const bool enabled = solution.valid && solution.source.generation == generation &&
    solution.source.exposure.domain() == mode_since.domain() &&
    solution.source.exposure.nanoseconds() > mode_since.nanoseconds();
  return {solution.source, sentry_authority(), solution.absolute_angles, solution.distance,
    enabled, enabled && program_fire_requested};
}
}  // namespace autoaim::mission
