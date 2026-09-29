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
  });
}
