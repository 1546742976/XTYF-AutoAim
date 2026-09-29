#include "autoaim/decision/predictor.hpp"
#include "support/snapshot_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto snapshot = test::snapshot();
    const decision::BallisticModel ballistic({0, 0, -9.81}, core::Seconds(1), core::Metres(0.01));
    const decision::InterceptRequest request{
        core::advance(snapshot->state_time, core::Seconds(0.01)), core::Seconds(0.02),
        math::Point3<math::WorldFrame>({0, 0, 0}), core::MetresPerSecond(20), 2};

    const decision::InterceptLimits limits{20, core::Seconds(1e-7), core::Metres(1e-5)};
    const auto result =
        decision::solve_intercept(*snapshot, request, {0.001, 0.001, 1}, ballistic, limits);

    CHECK(result && result.value().iterations <= limits.maximum_iterations);
    CHECK_NEAR(core::elapsed(result.value().fire_at, request.estimated_send).value(), 0.02, 1e-12);
    CHECK_NEAR(core::elapsed(result.value().hit_at, result.value().fire_at).value(),
               result.value().ballistic.flight_time.value(), 1e-9);

    const auto impact = ballistic.position(request.launch_origin,
                                           result.value().ballistic.initial_velocity_world_mps,
                                           result.value().ballistic.flight_time);

    CHECK((impact.metres() - result.value().plate.pose.value().translation()).norm() < 1e-5);
    CHECK(core::same_time(result.value().source.exposure, snapshot->source.exposure));
    CHECK(!decision::solve_intercept(*snapshot, request, {0, 0, 1}, ballistic,
                                     {1, core::Seconds(1e-12), core::Metres(1e-12)}));
    // 移动/旋转目标；时间门限足够宽，唯一改变的是最终位置残差门限。
    const auto loose = decision::solve_intercept(*snapshot, request, {0, 0, 1}, ballistic,
        {1, core::Seconds(1), core::Metres(0.1)});
    CHECK(loose && loose.value().iterations == 1);
    const auto loose_impact = ballistic.position(request.launch_origin,
        loose.value().ballistic.initial_velocity_world_mps, loose.value().ballistic.flight_time);
    const double residual =
        (loose_impact.metres() - loose.value().plate.pose.value().translation()).norm();
    CHECK(residual > 1e-8 && residual < 0.1);
    CHECK(!decision::solve_intercept(*snapshot, request, {0, 0, 1}, ballistic,
        {1, core::Seconds(1), core::Metres(residual / 2)}));

    auto invalid = request;
    invalid.speed = core::MetresPerSecond(0);
    CHECK(!decision::solve_intercept(*snapshot, invalid, {0, 0, 1}, ballistic, limits));
    CHECK(
        !decision::solve_intercept(*test::snapshot(false), request, {0, 0, 1}, ballistic, limits));
  });
}
