#include "autoaim/pipeline/pipeline.hpp"
#include "test_support.hpp"

int main(int argc, char** argv) {
  using namespace autoaim;
  return test::run([&] {
    CHECK(argc == 2);
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    auto config = pipeline::load_pipeline_config(root / "config/offline/armor.yaml");
    CHECK(config);
    config.value().detector = vision::Yolov5Options{argv[1], "CPU", vision::TeamColor::red, 0.5, 0.5, 128, 16};
    auto clock = std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay));
    auto* time = clock.get();
    pipeline::Pipeline pipeline(std::move(config).value(), std::move(clock), [](const control::Command&) {
      return hal::WriteResult{hal::WriteStatus::complete, 29, {}};
    }, control::PublishSchedule::replay_events);
    pipeline.start();
    auto pixels = std::make_shared<const std::vector<std::uint8_t>>(640 * 480 * 3, 0);
    for (int n = 1; n <= 3; ++n) {
      time->advance(core::Seconds(0.01));
      const auto now = time->now();
      pipeline.feedback({now, 1, {1, 0, 0, 0}, 0, 0, 20, true, true, std::nullopt});
      CHECK(pipeline.frame(core::CapturedFrame(core::Stamp(n, 1, now), core::Image(640, 480, 1920, pixels), now,
        core::TimeOrigin::synthetic, core::Seconds(0.001), core::Evidence::declared())));
    }
    pipeline.process_pending();
    pixels.reset();
    pipeline.finish_pending(core::Seconds(10));
    CHECK(pipeline.replay_tick());
    const auto metrics = pipeline.metrics();
    CHECK(metrics.accepted_results >= 1 && metrics.accepted_results <= 2);
    CHECK(metrics.queue_drops[static_cast<std::size_t>(pipeline::DropReason::capacity)] == 1);
    pipeline.close();
    CHECK(!pipeline.publisher_status().first_failure);
  });
}
