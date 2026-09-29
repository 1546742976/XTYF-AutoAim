#include "autoaim/pipeline/pipeline.hpp"
#include "test_support.hpp"
#include <atomic>

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    auto config = pipeline::load_pipeline_config(root / "config/offline/armor.yaml");
    CHECK(config);

    // 仅记录端的模拟通道证据；视觉/几何仍未实测，整链不得开火。
    config.value().control_channel = core::Evidence::from_report(core::MeasurementReport::evaluate(
        {"synthetic-camera", "synthetic-v1", "2026-09-29", "in-memory output test"}, {0, 0}, 0,
        core::ClockDomain::replay));

    auto clock = std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay));
    auto* time = clock.get();
    std::atomic<int> shots{0}, controls{0};
    std::size_t observed_frames = 0;
    pipeline::Pipeline pipeline(
        std::move(config).value(), std::move(clock),
        [&](const control::Command& command, const control::CommandMetadata&) {
          if (command.shoot)
            ++shots;

          if (command.control)
            ++controls;

          return hal::WriteResult{hal::WriteStatus::complete, 29, {}};
        },
        control::PublishSchedule::replay_events,
        [&](const vision::FramePacket& frame, const vision::DetectionBatch& batch,
            const estimation::ObservationBatch& observations) {
          CHECK(frame.frame.stamp.frame_id == batch.source.frame_id);
          CHECK(!observations.empty());
          ++observed_frames;
        });

    pipeline.start();
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(640 * 480 * 3, 0);

    for (int y = 210; y < 241; ++y)
      for (int x : {280, 281, 282, 283, 350, 351, 352, 353})
        (*pixels)[(y * 640 + x) * 3 + 2] = 255;

    for (int n = 1; n <= 6; ++n) {
      time->advance(core::Seconds(0.02));
      const auto now = time->now();
      pipeline.feedback({now, 1, {1, 0, 0, 0}, 0, 0, 20, true, true, std::nullopt});
      const core::CapturedFrame frame(core::Stamp(n, 1, now), core::Image(640, 480, 1920, pixels),
                                      now, core::TimeOrigin::synthetic, core::Seconds(0.001),
                                      core::Evidence::declared());

      CHECK(pipeline.frame(frame));
      pipeline.process_pending();
      CHECK(pipeline.replay_tick());
    }

    const auto metrics = pipeline.metrics();
    core::CaptureMetadata wrong_roi;
    wrong_roi.roi_offset = std::array<int, 2>{2, 0};
    CHECK_THROWS(std::invalid_argument,
        pipeline.frame(core::CapturedFrame(core::Stamp(7, 1, time->now()),
            core::Image(640, 480, 1920, pixels), time->now(), core::TimeOrigin::synthetic,
            core::Seconds(0), core::Evidence::missing(), wrong_roi)));
    CHECK(metrics.accepted_results == 6 && metrics.detections >= 6);
    CHECK(metrics.preprocessing.samples == 6 && metrics.postprocessing.samples == 6);
    CHECK(metrics.queue_timings.pool_copy.samples == 6 &&
          metrics.queue_timings.waiting.samples == 6);
    CHECK(metrics.inference_wall.total_ms == 0); // 传统路径没有神经网络推理。
    CHECK(metrics.valid_poses >= 6 && metrics.observations >= 6);
    CHECK(metrics.decisions >= 1 && metrics.submitted_intents >= 1);
    pipeline.close();
    CHECK(shots == 0 && controls >= 1);
    CHECK(observed_frames == 6);
    CHECK(pipeline.publisher_status().stop_written);
  });
}
