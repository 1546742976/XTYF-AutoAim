#include "autoaim/decision/predictor.hpp"
#include "support/snapshot_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto snapshot = test::snapshot();
    const auto later = core::advance(snapshot->state_time, core::Seconds(0.1));
    const auto prediction = decision::predict_future(*snapshot, later, {0.001, 0.001, 1});
    CHECK(prediction && prediction.value().plates.size() == 4);
    CHECK_NEAR(prediction.value().state.phase.value(), 0.3, 1e-12);
    CHECK_NEAR(prediction.value().state.center.metres().x(), 3.01, 1e-12);
    CHECK(prediction.value().source.frame_id == snapshot->source.frame_id);
    CHECK_NEAR(snapshot->state.phase.value(), 0.2, 0);
    CHECK(core::same_time(snapshot->state_time, snapshot->source.exposure));
    const auto unknown = test::snapshot(false);
    const auto fallback = decision::predict_future(*unknown, later, {0.001, 0.001, 1});
    CHECK(fallback && fallback.value().plates.size() == 1);
    CHECK(!fallback.value().plates.front().physical_plate &&
          !fallback.value().plates.front().reliable);

    CHECK(fallback.value().plates[0].covariance(0, 0) >
          unknown->visible_plate.covariance->operator()(0, 0));

    CHECK(!decision::predict_future(
        *snapshot, core::advance(snapshot->state_time, core::Seconds(-0.1)), {0, 0, 1}));

    auto state = snapshot->state;
    state.acceleration_mps2 = {2, -1, 0.5};
    auto covariance = snapshot->covariance;
    covariance(0, 9) = covariance(9, 0) = 0.0002;
    const estimation::TargetSnapshot accelerating(
        snapshot->source, snapshot->state_time, snapshot->target_id, state, covariance,
        snapshot->physical_plate, snapshot->quality, snapshot->pose_reliable,
        snapshot->historical_pose,
        std::make_shared<const estimation::MotionModel>(
            snapshot->motion->with_linear_acceleration(0.4)),
        snapshot->geometry, snapshot->visible_plate, snapshot->visible_dimensions,
        snapshot->time_origin, snapshot->timing_uncertainty, snapshot->timing_evidence);
    const auto accelerated = decision::predict_future(accelerating, later, {0.001, 0.001, 1});
    CHECK(accelerated);
    CHECK_NEAR(accelerated.value().state.center.metres().x(), 3.02, 1e-12);
    CHECK_NEAR(accelerated.value().state.velocity_mps.x(), 0.3, 1e-12);
    // Pxx + t² Pvv + t² Pxa + t⁴ Paa/4 + qj t⁵/20。
    CHECK_NEAR(accelerated.value().covariance(0, 0),
               0.001 + 0.00001 + 0.000002 + 0.000000025 + 0.0000002, 1e-12);
    CHECK((accelerating.state.acceleration_mps2 - state.acceleration_mps2).norm() == 0);
  });
}
