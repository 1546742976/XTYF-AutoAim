#include "autoaim/mission/sentry/sentry_mission.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const core::TimePoint start(0, core::ClockDomain::replay);
    const mission::AimSolution aim{core::Stamp(1, 1, core::advance(start, core::Seconds(0.01))),
      {core::Radians(0), core::Radians(0)}, core::Metres(2), true};
    CHECK(mission::sentry_request(aim, 1, start, true).fire_requested);
    CHECK(!mission::sentry_request(aim, 1, start, false).fire_requested);
    CHECK(!mission::sentry_request(aim, 2, start, true).control_requested);
    CHECK(!mission::sentry_request(aim, 1, aim.source.exposure, true).control_requested);
  });
}
