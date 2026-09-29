#include "autoaim/estimation/armor_id.hpp"
#include "support/geometry_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto geometry = test::geometry(4, estimation::HeightLayout::distinct, true);
    const math::Point3<math::WorldFrame> center({3, 1, 0.5});
    const estimation::ObservedPose pose{geometry.plate_pose(center, core::Radians(0.8), 2),
      vision::PoseCovariance::Identity() * 0.001};
    const auto hypotheses = estimation::initial_identity_hypotheses(pose, geometry);
    CHECK(hypotheses.size() == 4);
    CHECK((hypotheses[2].state.center.metres() - center.metres()).norm() < 1e-12);
    CHECK_NEAR(hypotheses[2].state.phase.value(), 0.8, 1e-12);
    // 每个编号都能解释首帧，不能因数组排序选 0。
    for (const auto& hypothesis : hypotheses) {
      const auto reproduced = geometry.plate_pose(hypothesis.state.center, hypothesis.state.phase, hypothesis.first_plate);
      CHECK((reproduced.value().translation() - pose.plate_to_world.value().translation()).norm() < 1e-12);
    }
    estimation::IdentityResolver resolver(4, {3, 1, 20, 0.5});
    for (std::uint64_t frame = 1; frame <= 4; ++frame) {
      const core::Stamp stamp(frame, 1, core::TimePoint(frame, core::ClockDomain::replay));
      CHECK(resolver.observe(stamp, {0, 0, 0, 0}));
      CHECK(!resolver.selected());
      CHECK(!resolver.observe(stamp, {20, 20, 0, 20}));
    }
    const core::Stamp distinct(5, 1, core::TimePoint(5, core::ClockDomain::replay));
    CHECK(resolver.observe(distinct, {10, 10, 0, 10}));
    CHECK(!resolver.selected());
    CHECK(resolver.observe(core::Stamp(6, 1, core::TimePoint(6, core::ClockDomain::replay)), {10, 10, 0, 10}));
    CHECK(!resolver.selected());
    CHECK(resolver.observe(core::Stamp(7, 1, core::TimePoint(7, core::ClockDomain::replay)), {10, 10, 0, 10}));
    CHECK(resolver.selected() && *resolver.selected() == 2);
    CHECK(!resolver.observe(core::Stamp(6, 0, core::TimePoint(6, core::ClockDomain::replay)), {10, 10, 0, 10}));
  });
}
