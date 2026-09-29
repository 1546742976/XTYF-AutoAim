#include "autoaim/estimation/state_machine.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    estimation::MotionSelector selector({0.1, 2, 0.5, 2, core::Seconds(0.02), core::Seconds(0.05)});
    const auto stamp = [](std::uint64_t n) {
      return core::Stamp(n, 1, core::TimePoint(n * 10000000, core::ClockDomain::replay));
    };

    CHECK(selector.observe(stamp(1), 0, true));
    CHECK(selector.observe(stamp(2), 0.1, true));
    CHECK(selector.kind() == estimation::MotionKind::constant_velocity);
    CHECK(selector.observe(stamp(3), 0.2, true));
    CHECK(selector.kind() == estimation::MotionKind::bounded_acceleration);
    CHECK(!selector.observe(stamp(3), 0.2, true));
    CHECK(selector.observe(stamp(4), 0.2, true));
    CHECK(selector.kind() == estimation::MotionKind::bounded_acceleration);
    CHECK(selector.observe(stamp(5), 0.2, true));
    CHECK(selector.kind() == estimation::MotionKind::constant_velocity);
    CHECK(selector.regime() == estimation::MotionRegime::steady_rotation);
    CHECK(selector.observe(stamp(6), 0.2, false));
    CHECK(selector.regime() == estimation::MotionRegime::uncertain);
  });
}
