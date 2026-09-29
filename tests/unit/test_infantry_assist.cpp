#include "autoaim/mission/infantry/infantry_mission.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const core::Stamp source(1, 2, core::TimePoint(100, core::ClockDomain::replay));
    const mission::AimSolution aim{
        source, {core::Radians(-3.13), core::Radians(0.1)}, core::Metres(3), true};

    decision::MeasuredPointing measured{
        source.exposure, 2, {core::Radians(3.13), core::Radians(0.04)}, true};

    const auto request = mission::infantry_assist(aim, measured);
    CHECK(!request.fire_requested && request.control_requested);
    CHECK(request.authority.space == core::CommandSpace::relative);
    CHECK_NEAR(request.angles.yaw.value(), 6.283185307179586 - 6.26, 1e-10);
    CHECK_NEAR(request.angles.pitch.value(), 0.06, 1e-10);
    measured.generation = 3;
    CHECK(!mission::infantry_assist(aim, measured).control_requested);
  });
}
