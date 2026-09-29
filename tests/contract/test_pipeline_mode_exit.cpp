#include "autoaim/pipeline/pipeline.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    auto config = pipeline::load_pipeline_config(root / "config/offline/armor.yaml");
    CHECK(config);
    const auto evidence = core::Evidence::from_report(core::MeasurementReport::evaluate(
      {"synthetic-camera", "synthetic-v1", "2026-09-29", "mode event simulation"}, {0, 0}, 0, core::ClockDomain::replay));
    config.value().control_channel = evidence;
    auto clock = std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay));
    auto* time = clock.get();
    pipeline::Pipeline pipeline(std::move(config).value(), std::move(clock), [](const control::Command&) {
      return hal::WriteResult{hal::WriteStatus::complete, 29, {}};
    }, control::PublishSchedule::replay_events);
    pipeline.start();
    for (const auto invalid_time : {core::TimePoint(1000000000, core::ClockDomain::replay),
        core::TimePoint(0, core::ClockDomain::host_monotonic)}) {
      pipeline.feedback({invalid_time, pipeline.generation(), {1, 0, 0, 0}, 0, 0, 20, true, true,
        hal::OperatorInput{time->now(), true, false, 1, evidence}});
      CHECK(pipeline.generation() == 1);
    }
    const auto input = [&](bool manual, std::uint64_t event) {
      time->advance(core::Seconds(0.01));
      pipeline.feedback({time->now(), pipeline.generation(), {1, 0, 0, 0}, 0, 0, 20, true, true,
        hal::OperatorInput{time->now(), true, manual, event, evidence}});
      CHECK(pipeline.replay_tick());
    };
    input(false, 1); CHECK(pipeline.generation() == 2);
    input(false, 1); CHECK(pipeline.generation() == 2);
    input(true, 1); CHECK(pipeline.generation() == 3);
    input(false, 1); CHECK(pipeline.generation() == 3);
    input(false, 2); CHECK(pipeline.generation() == 4);
    input(false, 2);
    time->advance(core::Seconds(0.1));
    CHECK(pipeline.replay_tick()); CHECK(pipeline.generation() == 5);
    pipeline.close();
    CHECK(pipeline.publisher_status().stop_written && !pipeline.publisher_status().device_stop_confirmed);
  });
}
