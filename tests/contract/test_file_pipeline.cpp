#include "autoaim/pipeline/pipeline.hpp"
#include "test_support.hpp"
#include <opencv2/imgcodecs.hpp>
#include <fstream>

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto directory = std::filesystem::current_path() / "file-pipeline-fixture";
    std::filesystem::create_directories(directory);
    CHECK(cv::imwrite((directory / "image.png").string(), cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0))));
    {
      std::ofstream manifest(directory / "events.yaml");
      manifest << "domain: replay\nevents:\n"
        "  - {at_ns: 100, kind: feedback, quaternion_wxyz: [1, 0, 0, 0], yaw_rad: 0, pitch_rad: 0, bullet_speed_mps: 20, pose_valid: true, status_valid: true}\n"
        "  - {at_ns: 100, kind: image, frame_id: 1, exposure_ns: 100, path: image.png, time_origin: synthetic, timing_sigma_s: 0}\n";
      CHECK(manifest.good());
    }
    const auto run = [&] {
      auto config = pipeline::load_pipeline_config(root / "config/offline/armor.yaml");
      CHECK(config);
      config.value().input_manifest = directory / "events.yaml";
      std::vector<bool> writes;
      pipeline::Pipeline pipeline(std::move(config).value(),
        std::make_unique<hal::ReplayClock>(core::TimePoint(0, core::ClockDomain::replay)), [&](const control::Command& command) {
          writes.push_back(command.shoot); return hal::WriteResult{hal::WriteStatus::complete, 29, {}};
        }, control::PublishSchedule::replay_events);
      const auto result = pipeline.run_file();
      CHECK(result);
      CHECK(result.value().received_frames == 1 && result.value().accepted_results == 1);
      CHECK(!pipeline.publisher_status().first_failure);
      return writes;
    };
    const auto first = run(), second = run();
    CHECK(first == second && first.size() == 2);
    CHECK(std::none_of(first.begin(), first.end(), [](bool value) { return value; }));
  });
}
