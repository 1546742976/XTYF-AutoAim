#include "autoaim/pipeline/frame_sync.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    pipeline::FrameSync sync(4, core::Seconds(0.05), 0);
    const core::TimePoint origin(0, core::ClockDomain::replay);
    auto feedback = [&](double seconds, core::Generation generation, bool valid,
                        bool bad_q = false) {
      return hal::GimbalFeedback{core::advance(origin, core::Seconds(seconds)),
                                 generation,
                                 {bad_q ? 0.0 : 1.0, 0, 0, 0},
                                 0,
                                 0,
                                 25,
                                 valid,
                                 true,
                                 std::nullopt};
    };

    CHECK(sync.push(feedback(0, 0, true)));
    CHECK(sync.push(feedback(0.02, 0, true)));
    const auto time = core::advance(origin, core::Seconds(0.01));
    CHECK(sync.query(time, 0));
    CHECK(sync.query(time, 0));
    CHECK(!sync.query(core::advance(origin, core::Seconds(-0.01)), 0));
    CHECK(!sync.query(core::advance(origin, core::Seconds(1)), 0));
    CHECK(!sync.push(feedback(0.01, 0, true)));
    CHECK(sync.push(feedback(0.04, 0, false)));
    CHECK(!sync.query(core::advance(origin, core::Seconds(0.03)), 0));
    CHECK(sync.push(feedback(0.06, 0, true, true)));
    CHECK(!sync.query(core::advance(origin, core::Seconds(0.06)), 0));
    sync.reset(1);
    CHECK(!sync.query(time, 0));
    CHECK(!sync.push(feedback(0.08, 0, true)));
    CHECK(sync.push(feedback(0.08, 1, true)));
    CHECK(sync.push(feedback(0.18, 1, true)));
    CHECK(!sync.query(core::advance(origin, core::Seconds(0.10)), 1));
  });
}
