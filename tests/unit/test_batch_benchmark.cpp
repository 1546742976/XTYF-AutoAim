#include "autoaim/pipeline/offline_tools.hpp"
#include "autoaim/hal/session_writer.hpp"
#include "test_support.hpp"
#include <chrono>
#include <fstream>

int main(int argc, char** model_paths) {
  using namespace autoaim;

  return test::run([&] {
    CHECK(argc == 1 || argc == 3);
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto directory = std::filesystem::temp_directory_path() /
        ("autoaim-benchmark-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    CHECK(std::filesystem::create_directory(directory));
    struct Cleanup {
      std::filesystem::path path;
      ~Cleanup() {
        std::filesystem::remove_all(path);
      }
    } cleanup{directory};
    hal::SessionWriter writer(directory / "input", {"fixture", "v1", std::nullopt});
    for (int id = 1; id <= 3; ++id) {
      const core::TimePoint at(id * 10000000, core::ClockDomain::replay);
      auto pixels = std::make_shared<std::vector<std::uint8_t>>(640 * 480 * 3, 0);
      if (id == 1)
        for (int y = 210; y < 241; ++y)
          for (int x : {280, 281, 282, 283, 350, 351, 352, 353})
            (*pixels)[(y * 640 + x) * 3 + 2] = 255;
      YAML::Node annotations;
      if (id == 1)
        annotations = YAML::Load(R"([{category: unknown, color: red, plate_type: small,
          corners: [[281,210],[351,210],[351,240],[281,240]],
          visible: [true,true,true,true], track_id: fixture}])");
      if (id == 2)
        annotations = YAML::Load("[]");
      const auto image = std::make_shared<const core::CapturedFrame>(core::Stamp(id, 1, at),
          core::Image(640, 480, 1920, pixels), at, core::TimeOrigin::synthetic,
          core::Seconds(0), core::Evidence::missing());
      writer.append(at, image, annotations);
    }
    writer.finish();
    std::vector<std::string> arguments{"bench_detector", "--dataset",
        (directory / "input/events.yaml").string(), "--config",
        (root / "config/offline/armor.yaml").string(), "--config",
        (root / "config/offline/armor.yaml").string(), "--iou", "0.5", "--output",
        (directory / "comparison").string()};
    if (argc == 3) {
      for (int model = 0; model < 2; ++model) {
        const auto name = model == 0 ? "yolov5.yaml" : "yolo11.yaml";
        auto config = YAML::LoadFile((root / "config/offline" / name).string());
        config["detector"]["model_path"] = std::string(model_paths[model + 1]);
        config["calibration_file"] = (root / "config/offline/calibration.yaml").string();
        config["geometry_files"][0] = (root / "config/offline/geometry.yaml").string();
        const auto path = directory / name;
        {
          std::ofstream output(path);
          output << config;
          CHECK(output.good());
        }
        arguments.push_back("--config");
        arguments.push_back(path.string());
      }
    }
    std::vector<char*> argv;
    for (auto& argument : arguments)
      argv.push_back(argument.data());
    CHECK(pipeline::run_detector_benchmark(int(argv.size()), argv.data()) == 0);
    const auto report = YAML::LoadFile((directory / "comparison/report.yaml").string());
    const auto measured = YAML::LoadFile((directory / "comparison/timing.yaml").string());
    CHECK(report["runs"].size() == (argc == 3 ? 4 : 2));
    if (argc == 3) {
      CHECK(report["runs"][2]["detector"].as<std::string>() == "yolov5");
      CHECK(report["runs"][3]["detector"].as<std::string>() == "yolo11");
      CHECK(report["runs"][3]["model_weights"]["bytes"].as<std::uint64_t>() > 0);
    }
    CHECK(report["runs"][0]["configuration_file"]["fnv1a64"].as<std::string>().size() == 16);
    CHECK(report["runs"][0]["calibration_parameters"]["width"].as<int>() == 640);
    CHECK(YAML::Dump(report["runs"][0]) == YAML::Dump(report["runs"][1]));
    CHECK(report["runs"][0]["command_writes"].as<std::uint64_t>() > 0);
    CHECK(report["runs"][0]["command_format"].as<std::string>() == "uart14-recording");
    const auto summary = report["runs"][0]["summary"];
    CHECK(summary["frames"].as<int>() == 3 && summary["unlabeled_frames"].as<int>() == 1);
    CHECK(summary["negative_frames"].as<int>() == 1);
    CHECK(summary["geometric_tp"].as<int>() == 1);
    CHECK(summary["mean_position_error_m"].IsNull());
    CHECK(summary["usable_chain_recall"].as<double>() == 0);
    CHECK(measured["runs"][0]["pool_copy"]["samples"].as<int>() == 3);
    CHECK(pipeline::run_detector_benchmark(int(argv.size()), argv.data()) == 1);

    // 同一数据的不同有效期必须重新分层；分别覆盖旧输入和独立按键。
    for (const bool button : {false, true}) {
      const std::string tag = button ? "button" : "legacy";
      const auto input = directory / (tag + "-input");
      hal::SessionWriter samples(input, {});
      const core::TimePoint sampled(1000000, core::ClockDomain::replay);
      if (button)
        samples.append(sampled, hal::ButtonSample{sampled, true, true, false});
      else
        samples.append(sampled, hal::GimbalFeedback{sampled, 1, {1, 0, 0, 0}, 0, 0, 20,
            true, true, hal::OperatorInput{sampled, true, true, 0, core::Evidence::declared()}});
      for (int id = 1; id <= 3; ++id) {
        const core::TimePoint at(id * 10000000, core::ClockDomain::replay);
        const core::TimePoint exposure(id * 10000000 - 5000000, core::ClockDomain::replay);
        const auto pixels = std::make_shared<std::vector<std::uint8_t>>(640 * 480 * 3, 0);
        samples.append(at, std::make_shared<const core::CapturedFrame>(
            core::Stamp(id, 1, exposure), core::Image(640, 480, 1920, pixels), at,
            core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing()),
            YAML::Load("[]"));
      }
      samples.finish();

      std::vector<std::string> compare{"bench_detector", "--dataset",
          (input / "events.yaml").string(), "--iou", "0.5", "--output",
          (directory / (tag + "-comparison")).string()};
      for (int variant = 0; variant < 2; ++variant) {
        auto config = YAML::LoadFile((root / "config/offline/armor.yaml").string());
        config["calibration_file"] = (root / "config/offline/calibration.yaml").string();
        config["geometry_files"][0] = (root / "config/offline/geometry.yaml").string();
        config["operator_input"]["kind"] = button ? "button" : "legacy_enable_event";
        config["safety"]["operator_age_s"] = variant ? 0.001 : 0.1;
        config["queue"]["maximum_age_s"] = variant ? 0.001 : 0.1;
        const auto path = directory / (tag + std::to_string(variant) + ".yaml");
        {
          std::ofstream output(path);
          output << config;
          CHECK(output.good());
        }
        compare.push_back("--config");
        compare.push_back(path.string());
      }
      std::vector<char*> values;
      for (auto& value : compare)
        values.push_back(value.data());
      CHECK(pipeline::run_detector_benchmark(int(values.size()), values.data()) == 0);
      const auto layered = YAML::LoadFile(
          (directory / (tag + "-comparison/report.yaml")).string())["runs"];
      CHECK(layered[0]["by_intervention"]["manual"]["frames"].as<int>() == 3);
      CHECK(layered[1]["by_intervention"]["unknown"]["frames"].as<int>() == 3);
      CHECK(layered[0]["summary"]["expired_frames"].as<int>() == 0);
      CHECK(layered[1]["summary"]["expired_frames"].as<int>() == 3);
      CHECK(layered[1]["summary"]["rejected_or_dropped_frames"].as<int>() == 3);
    }
  });
}
