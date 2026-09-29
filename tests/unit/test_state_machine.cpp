#include "autoaim/estimation/state_machine.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    estimation::TrackingStateMachine state({3, core::Seconds(0.02), core::Seconds(0.05),
      core::Seconds(0.2), 0.1, 0.2});
    const auto stamp = [](std::uint64_t id, std::int64_t ms) {
      return core::Stamp(id, 1, core::TimePoint(ms * 1000000, core::ClockDomain::replay));
    };
    estimation::QualityFacts facts{true, true, true, true, false, 0.05};
    CHECK(state.observe(stamp(1, 10), facts));
    CHECK(state.observe(stamp(2, 20), facts));
    CHECK(state.quality() != estimation::TrackingQuality::converged);
    CHECK(!state.observe(stamp(2, 20), facts));
    CHECK(state.observe(stamp(3, 30), facts));
    CHECK(state.lifecycle() == estimation::TrackLifecycle::tracking && state.quality() == estimation::TrackingQuality::converged);
    facts.phase_variance = 0.15;
    CHECK(state.observe(stamp(4, 40), facts));
    CHECK(state.quality() == estimation::TrackingQuality::converged);
    facts.identity_known = false;
    CHECK(state.observe(stamp(5, 50), facts));
    CHECK(state.quality() == estimation::TrackingQuality::degraded);
    state.tick(stamp(6, 101).exposure);
    CHECK(state.lifecycle() == estimation::TrackLifecycle::coasting);
    state.tick(stamp(7, 250).exposure);
    CHECK(state.lifecycle() == estimation::TrackLifecycle::lost);
    facts.fault = true;
    CHECK(state.observe(stamp(8, 260), facts));
    CHECK(state.lifecycle() == estimation::TrackLifecycle::lost);
  });
}
