#include "autoaim/decision/aim_adequacy.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    decision::AimAdequacy aim({core::Radians(0.02), core::Radians(0.02), 3, core::Seconds(0.02),
                               core::Seconds(0.05), core::Seconds(0.1), core::Seconds(0.03)});

    const auto time = [](int ms) {
      return core::TimePoint(ms * 1000000, core::ClockDomain::replay);
    };

    decision::AimKey key{1, "p", 0};
    const decision::AimAngles desired{core::Radians(3.14), core::Radians(0)};
    const auto check = [&](int id, int ms, bool valid = true) {
      return aim.evaluate(core::Stamp(id, 1, time(ms)), key, desired,
                          {time(ms), 1, {core::Radians(-3.14), core::Radians(0)}, valid}, time(ms));
    };

    CHECK(!check(1, 0).settled);
    CHECK(!check(1, 0).settled);
    CHECK(!check(2, 10).settled);
    CHECK(check(3, 20).settled);
    CHECK(check(3, 20).supporting_observations == 3);
    CHECK(!check(3, 20, false).settled);
    CHECK(!check(3, 20).settled);
    CHECK(check(4, 30).supporting_observations == 1);
    CHECK(check(5, 100).supporting_observations == 1);
    key.target = 2;
    CHECK(!check(6, 110).settled);
    CHECK_NEAR(decision::direction_to_angles(Eigen::Vector3d::UnitY()).yaw.value(),
               1.5707963267948966, 1e-10);

    CHECK_THROWS(std::invalid_argument, decision::direction_to_angles(Eigen::Vector3d::Zero()));
    CHECK(!aim.evaluate(core::Stamp(7, 1, time(120)), key, desired, {time(80), 1, desired, true},
                        time(120))
               .feedback_fresh);
  });
}
