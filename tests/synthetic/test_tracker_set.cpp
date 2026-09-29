#include "support/tracker_fixture.hpp"
#include "support/geometry_fixture.hpp"
#include "test_support.hpp"
#include <set>

int main() {
  using namespace autoaim;

  return test::run([] {
    auto profile = std::make_shared<const estimation::GeometryProfile>(
        test::geometry(4, estimation::HeightLayout::distinct, false));

    auto cv = std::make_shared<const estimation::MotionModel>(0.01, 0.01, core::Seconds(0.5));
    auto ca = std::make_shared<const estimation::MotionModel>(0.01, 0.01, core::Seconds(0.5), 10);
    estimation::TrackerSet targets(1, {profile}, cv, ca, test::tracker_options(),
                                   {2, 0.1, core::Metres(0.7)});

    for (std::uint64_t frame : {1, 2, 4, 5, 6}) {
      const core::Stamp source(frame, 1,
                               core::TimePoint(frame * 20000000, core::ClockDomain::replay));

      estimation::ObservationBatch batch;

      for (double y : {-1, 1})
        for (std::size_t board : {2, 3, 0, 1}) {
          const auto original = test::observation(*profile, source,
              math::Point3<math::WorldFrame>({3, y, 1}), core::Radians(0.2), board);
          auto detection = original->detection;
          detection.raw_class_id = frame % 2 ? 3 : 30;
          detection.category = vision::TargetCategory::three;
          batch.push_back(std::make_shared<const estimation::Observation>(original->source,
              detection, original->pnp, original->world_candidates, original->historical_pose,
              original->time_origin, original->timing_uncertainty, original->timing_evidence));
        }

      CHECK(targets.update(source, batch).value());
      const auto snapshots = targets.snapshots(source.exposure);
      CHECK(snapshots.size() == 2);
      std::set<std::uint64_t> ids;

      for (const auto& snapshot : snapshots)
        ids.insert(snapshot->target_id);

      CHECK(ids == std::set<std::uint64_t>({1, 2}));
    }

    // 0.4 s 大于 lost(0.3)，但仍小于模型时域(0.5)：也必须按新目标重新获取。
    const core::Stamp reacquired(7, 1, core::TimePoint(520000000, core::ClockDomain::replay));
    CHECK(targets.update(reacquired, {test::observation(*profile, reacquired,
        math::Point3<math::WorldFrame>({3, -1, 1}), core::Radians(0.2), 2)}).value());
    const auto fresh = targets.snapshots(reacquired.exposure);
    CHECK(fresh.size() == 1 && fresh.front()->target_id == 3);
    CHECK(!fresh.front()->physical_plate && !fresh.front()->pose_reliable);

    const core::Stamp empty(8, 1, core::TimePoint(1000000000, core::ClockDomain::replay));
    CHECK(targets.update(empty, {}).value());
    CHECK(targets.snapshots(empty.exposure).empty());
    targets.reset(2);
    CHECK(!targets.update(empty, {}));
    CHECK_THROWS(std::invalid_argument, targets.reset(2));
  });
}
