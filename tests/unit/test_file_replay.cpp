#include "autoaim/hal/file_replay.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    hal::ReplayClock clock(core::TimePoint(0, core::ClockDomain::replay));
    hal::FileReplay replay(std::filesystem::path(__FILE__).parent_path().parent_path() / "fixtures/replay.yaml", clock);
    auto first = replay.next(1);
    CHECK(first && std::holds_alternative<hal::GimbalFeedback>(first.value()));
    CHECK(clock.now().nanoseconds() == 100);
    auto second = replay.next(2);
    CHECK(second);
    const auto frame = std::get<std::shared_ptr<const core::CapturedFrame>>(second.value());
    CHECK(frame->stamp.generation == 2 && frame->stamp.exposure.nanoseconds() == 150);
    CHECK(frame->image.width == 2 && frame->image.pixels->at(2) == 255);
    CHECK(frame->timing_evidence.level() == core::EvidenceLevel::declared);
    CHECK(replay.next(2));
    CHECK(clock.now().nanoseconds() == 200);
    CHECK(std::holds_alternative<hal::ReplayEnd>(replay.next(2).value()));
    CHECK_THROWS(std::invalid_argument, clock.advance_to(core::TimePoint(199, core::ClockDomain::replay)));
  });
}
