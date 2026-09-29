#include "autoaim/decision/ballistic.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const decision::BallisticModel model({0, 0, -9.81}, core::Seconds(2), core::Metres(0.01));
    const math::Point3<math::WorldFrame> origin({0, 0, 0});

    for (const Eigen::Vector3d& point : {Eigen::Vector3d(10, 0, 0), Eigen::Vector3d(3, 2, 1),
                                         Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, -1)}) {
      const math::Point3<math::WorldFrame> target(point);
      const auto solution = model.solve(origin, target, core::MetresPerSecond(20));
      CHECK(solution && solution.value().flight_time.value() > 0);
      CHECK_NEAR(solution.value().initial_velocity_world_mps.norm(), 20, 1e-9);
      CHECK((model
                 .position(origin, solution.value().initial_velocity_world_mps,
                           solution.value().flight_time)
                 .metres() -
             point)
                .norm() < 1e-9);
    }

    CHECK(!model.solve(origin, math::Point3<math::WorldFrame>({100, 0, 100}),
                       core::MetresPerSecond(1)));

    CHECK(
        !model.solve(origin, math::Point3<math::WorldFrame>({3, 0, 0}), core::MetresPerSecond(0)));

    CHECK(!model.solve(origin, origin, core::MetresPerSecond(20)));
    const decision::BallisticModel straight({0, 0, 0}, core::Seconds(2), core::Metres(0.01));
    CHECK_NEAR(
        straight
            .solve(origin, math::Point3<math::WorldFrame>({10, 0, 0}), core::MetresPerSecond(20))
            .value()
            .flight_time.value(),
        0.5, 0);

    CHECK_THROWS(std::invalid_argument,
                 core::MetresPerSecond(std::numeric_limits<double>::quiet_NaN()));
  });
}
