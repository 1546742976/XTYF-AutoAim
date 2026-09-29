#include "support/guard_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    auto guard = test::guard();
    const core::TimePoint time(1000000000, core::ClockDomain::replay);
    auto automatic = guard.admit(test::intent(1, 0, time), test::context(time), time);
    CHECK(automatic.fire_allowed());
    CHECK(guard.recheck(automatic, test::context(time), time).shoot);
    CHECK_NEAR(guard.recheck(automatic, test::context(time), time).horizontal_distance.value(), 3, 0);
    CHECK(!guard.recheck(automatic, test::context(time, core::ControlMode::automatic, true), time).control_enabled);
    CHECK(!guard.recheck(automatic, test::context(time), time).shoot);
    auto fresh = guard.admit(test::intent(2, 0, time), test::context(time), time);
    CHECK(fresh.fire_allowed());
    CHECK(!guard.recheck(fresh, test::context(time, core::ControlMode::assist, false, 1), time).shoot);
    CHECK(!guard.admit(test::intent(3, 0, time, core::ControlMode::assist),
      test::context(time, core::ControlMode::assist), time).fire_allowed());
    auto expired = guard.admit(test::intent(4, 0, time), test::context(time), time);
    const auto late = core::advance(time, core::Seconds(0.1));
    const auto tracking = guard.recheck(expired, test::context(late), late);
    CHECK(tracking.control_enabled && !tracking.shoot);
    CHECK(!guard.recheck(expired, test::context(time), time).shoot);
    CHECK(core::same_time(tracking.source->exposure, time));
    auto fault = test::context(time); fault.fault_latched = true;
    CHECK(!guard.admit(test::intent(5, 0, time), fault, time).control_allowed());
    auto absent = test::context(time); absent.feedback.reset();
    CHECK(!guard.admit(test::intent(6, 0, time), absent, time).control_allowed());
    auto declaration = test::context(time); declaration.control_channel = core::Evidence::declared();
    CHECK(!guard.admit(test::intent(7, 0, time), declaration, time).fire_allowed());
    auto wrapped = test::context(time);
    const auto input = wrapped.feedback->operator_input;
    wrapped.feedback.emplace(hal::GimbalFeedback{time, 0, {1, 0, 0, 0}, 6.28318530717958647692, 0, 25, true, true, input});
    CHECK(guard.admit(test::intent(8, 0, time), wrapped, time).fire_allowed());
    wrapped.feedback.emplace(hal::GimbalFeedback{time, 0, {1, 0, 0, 0}, 6.28318530717958647692,
      6.28318530717958647692, 25, true, true, input});
    CHECK(!guard.admit(test::intent(9, 0, time), wrapped, time).fire_allowed());
  });
}
