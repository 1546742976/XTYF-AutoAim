#include "autoaim/control/smoother.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const core::TimePoint start(0, core::ClockDomain::replay);
    const auto at = [&](double seconds) {
      return core::advance(start, core::Seconds(seconds));
    };

    control::CorrectionSmoother smoother({core::Radians(0.2), 1, core::Seconds(0.02),
                                          core::Radians(0.02), core::Radians(0.01),
                                          core::Seconds(0.05)});

    CHECK_NEAR(smoother.update(core::Radians(0.3), at(0), true).value(), 0, 0);
    const auto value = smoother.update(core::Radians(0.3), at(0.1), true).value();
    CHECK(value > 0 && value <= 0.1);
    CHECK(smoother.update(core::Radians(0.3), at(0.2), true).value() <= 0.2);
    CHECK_NEAR(smoother.emergency_stop(at(0.201)).value(), 0, 0);
    CHECK_NEAR(smoother.update(core::Radians(0.005), at(0.3), true).value(), 0, 0);
    CHECK_THROWS(std::invalid_argument, smoother.update(core::Radians(0), at(0.2), true));
    control::CorrectionSmoother normal({core::Radians(0.2), 1, core::Seconds(0),
                                        core::Radians(0.02), core::Radians(0.01),
                                        core::Seconds(0)});

    normal.update(core::Radians(0.2), at(0), true);
    CHECK_NEAR(normal.update(core::Radians(0.2), at(0.2), true).value(), 0.2, 1e-12);
    CHECK_NEAR(normal.update(core::Radians(0), at(0.21), false).value(), 0.19, 1e-12);
  });
}
