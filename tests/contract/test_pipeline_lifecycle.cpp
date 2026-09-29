#include "autoaim/pipeline/pipeline.hpp"
#include "test_support.hpp"
#include <atomic>

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    auto config = pipeline::load_pipeline_config(root / "config/offline/armor.yaml");
    CHECK(config);
    std::atomic<int> stopped{0};
    pipeline::Pipeline pipeline(
        std::move(config).value(),
        std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay)),
        [&](const control::Command& command, const control::CommandMetadata&) {
          if (!command.control && !command.shoot)
            ++stopped;

          return hal::WriteResult{hal::WriteStatus::complete, 29, {}};
        },
        control::PublishSchedule::replay_events);

    pipeline.start();
    CHECK_THROWS(std::logic_error, pipeline.start());
    pipeline.close();
    pipeline.close();
    CHECK(stopped == 1);
    CHECK(pipeline.publisher_status().stop_written);
    CHECK_THROWS(std::logic_error, pipeline.start());
  });
}
