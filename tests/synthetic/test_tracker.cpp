#include "support/tracker_fixture.hpp"
#include "support/geometry_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    auto profile = std::make_shared<const estimation::GeometryProfile>(test::geometry(4, estimation::HeightLayout::distinct, true));
    auto cv = std::make_shared<const estimation::MotionModel>(0.01, 0.02, core::Seconds(0.5));
    auto ca = std::make_shared<const estimation::MotionModel>(0.01, 0.02, core::Seconds(0.5), 10);
    estimation::Tracker tracker(7, {profile}, cv, ca, test::tracker_options());
    std::shared_ptr<const estimation::TargetSnapshot> saved;
    double saved_phase = 0;
    for (std::uint64_t frame = 1; frame <= 30; ++frame) {
      const core::Stamp source(frame, 1, core::TimePoint(frame * 20000000, core::ClockDomain::replay));
      const math::Point3<math::WorldFrame> center({3 + 0.002 * frame, 0, 1});
      estimation::ObservationBatch observations;
      // 顺序从物理板 2 开始，验证不把首板映射到 profile[0]。
      for (std::size_t board : {2, 3, 0, 1})
        observations.push_back(test::observation(*profile, source, center, core::Radians(0.2 + 0.01 * frame), board));
      const auto updated = tracker.update(observations);
      CHECK(updated && updated.value());
      const auto snapshot = tracker.snapshot(source.exposure);
      CHECK(snapshot && snapshot->target_id == 7 && snapshot->source.frame_id == frame);
      CHECK(tracker.hypothesis_count() == 4);
      if (frame == 1) CHECK(!snapshot->physical_plate && !snapshot->pose_reliable);
      if (frame == 20) { saved = snapshot; saved_phase = saved->state.phase.value(); }
      if (frame == 30) {
        CHECK(snapshot->physical_plate && snapshot->pose_reliable);
        CHECK(snapshot->quality == estimation::TrackingQuality::converged);
        CHECK((snapshot->state.center.metres() - center.metres()).norm() < 0.03);
        CHECK(!tracker.update(observations));
      }
    }
    CHECK(saved->source.frame_id == 20 && saved->state.phase.value() == saved_phase);
    CHECK(!tracker.snapshot(core::TimePoint(1000000000, core::ClockDomain::replay)));
    // 太长跳帧触发重新获取，不能沿用之前的身份/收敛支持。
    const core::Stamp restarted(40, 1, core::TimePoint(2000000000, core::ClockDomain::replay));
    const auto restarted_observation = test::observation(*profile, restarted,
      math::Point3<math::WorldFrame>({3.1, 0, 1}), core::Radians(0.5), 2);
    CHECK(tracker.update({restarted_observation}).value());
    CHECK(!tracker.snapshot(restarted.exposure)->physical_plate);
    CHECK(!tracker.update({test::observation(*profile,
      core::Stamp(41, 0, core::TimePoint(2020000000, core::ClockDomain::replay)),
      math::Point3<math::WorldFrame>({3.1, 0, 1}), core::Radians(0.5), 2)}));
    auto symmetric = std::make_shared<const estimation::GeometryProfile>(test::geometry(4, estimation::HeightLayout::same, false));
    estimation::Tracker uncertain(8, {symmetric}, cv, ca, test::tracker_options());
    for (std::uint64_t frame = 1; frame <= 8; ++frame) {
      const core::Stamp source(frame, 1, core::TimePoint(frame * 20000000, core::ClockDomain::replay));
      const auto obs = test::observation(*symmetric, source, math::Point3<math::WorldFrame>({3, 0, 1}), core::Radians(0.2), 2);
      CHECK(uncertain.update({obs}).value());
      const auto snapshot = uncertain.snapshot(source.exposure);
      CHECK(snapshot && !snapshot->physical_plate && !snapshot->pose_reliable);
    }
  });
}
