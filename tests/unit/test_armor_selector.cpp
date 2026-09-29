#include "autoaim/decision/armor_selector.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    decision::ArmorSelector selector({0.1, 1, 0, 0, 0.05, core::Seconds(0.1)});
    const auto stamp = [](std::uint64_t n, std::int64_t ms) { return core::Stamp(n, 1, core::TimePoint(ms * 1000000, core::ClockDomain::replay)); };
    const auto candidate = [](const core::Stamp& source, std::size_t plate, double yaw, bool facing = true) {
      const Eigen::Quaterniond orientation(Eigen::AngleAxisd(facing ? -1.5707963267948966 : 1.5707963267948966, Eigen::Vector3d::UnitY()));
      const decision::PredictedPlate target{plate, math::Transform<math::PlateFrame, math::WorldFrame>(math::SE3(orientation, {3, 0, 1})),
        vision::PoseCovariance::Identity() * 1e-5, vision::PlateDimensions(core::Metres(0.135), core::Metres(0.055)),
        Eigen::Vector3d::Zero(), true};
      return decision::InterceptSolution(source, source.exposure, source.exposure, core::advance(source.exposure, core::Seconds(0.2)),
        {{20 * std::cos(yaw), 20 * std::sin(yaw), 0}, core::Seconds(0.2)}, target, 2);
    };
    auto source = stamp(1, 10);
    CHECK(selector.select(source, 1, "profile", {candidate(source, 0, 0), candidate(source, 1, 0.2)},
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()) == 0);
    const auto second = stamp(2, 20);
    CHECK(selector.select(second, 1, "profile", {candidate(second, 0, 0.2), candidate(second, 1, 0)},
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()) == 0);
    const auto third = stamp(3, 120);
    CHECK(selector.select(third, 1, "profile", {candidate(third, 0, 0.2), candidate(third, 1, 0)},
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()) == 1);
    const auto fourth = stamp(4, 130);
    CHECK(selector.select(fourth, 1, "profile", {candidate(fourth, 0, 0.2), candidate(fourth, 1, 0, false)},
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero()) == 0);
  });
}
