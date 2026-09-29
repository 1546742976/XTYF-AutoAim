#include "autoaim/mission/target_selector.hpp"
#include "support/snapshot_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    mission::TargetSelector selector({core::Seconds(0.1), core::Seconds(0.3), 0.1});
    const auto original = test::snapshot(false);
    const auto shifted = [&](std::uint64_t id, double y, int ms) {
      const auto& s = *original;
      auto pose = s.visible_plate;
      pose.plate_to_world = math::Transform<math::PlateFrame, math::WorldFrame>(
        math::SE3(pose.plate_to_world.value().rotation(), {3, y, 1}));
      const core::Stamp source(id, 1, core::TimePoint(ms * 1000000, core::ClockDomain::replay));
      const auto aligned = vision::AlignedPose(source.exposure, source.exposure, source.exposure,
        s.historical_pose->gimbal_to_world, true);
      return std::make_shared<const estimation::TargetSnapshot>(source, source.exposure, id, s.state, s.covariance,
        s.physical_plate, s.quality, s.pose_reliable, aligned, s.motion, s.geometry, pose,
        s.visible_dimensions, s.time_origin, s.timing_uncertainty, s.timing_evidence);
    };
    const auto a = shifted(1, 0, 100), b = shifted(2, 2, 100);
    const auto now = a->source.exposure;
    CHECK(selector.select({b, a}, 1, now, {0, 0, 1}, Eigen::Vector3d::UnitX())->target_id == 1);
    CHECK(selector.select({a, b}, 1, now, {0, 0, 1}, {3, 2, 0})->target_id == 1);
    CHECK(!selector.select({b}, 1, now, {0, 0, 1}, Eigen::Vector3d::UnitX()));
    const auto later = shifted(2, 2, 410);
    CHECK(selector.select({later}, 1, later->source.exposure, {0, 0, 1}, Eigen::Vector3d::UnitX())->target_id == 2);
    CHECK(!selector.select({later}, 2, later->source.exposure, {0, 0, 1}, Eigen::Vector3d::UnitX()));
  });
}
