#include "autoaim/pipeline/offline_tools.hpp"
#include "autoaim/hal/session_writer.hpp"
#include "autoaim/pipeline/run_metadata.hpp"
#include "autoaim/vision/calibration_report.hpp"
#include "support/calibration_fixture.hpp"
#include "../../src/pipeline/session_annotation.hpp"
#include "../../src/pipeline/yaml_output.hpp"
#include "test_support.hpp"
#include <chrono>
#include <fstream>
#include <sstream>

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

    const auto text_path = directory / "text.yaml";
    YAML::Node text_node;
    text_node["value"] = 1.0 / 3.0;
    text_node["count"] = 42;
#ifdef _WIN32
    constexpr const char* text_bytes = "value: 0.33333333333333331\r\ncount: 42\r\n";
    constexpr const char* empty_bytes = "[]\r\n";
#else
    constexpr const char* text_bytes = "value: 0.33333333333333331\ncount: 42\n";
    constexpr const char* empty_bytes = "[]\n";
#endif
    pipeline::yaml_detail::write_text(text_path, text_node, "Cannot write evaluation report");
    CHECK(pipeline::annotation_detail::read_bytes(text_path) == text_bytes);
    pipeline::yaml_detail::write_text(text_path, YAML::Load("[]"),
                                      "Cannot write evaluation report");
    CHECK(pipeline::annotation_detail::read_bytes(text_path) == empty_bytes);
    const auto write_fails = [&](const std::filesystem::path& path, const char* message) {
      bool failed = false;
      try {
        pipeline::yaml_detail::write_text(path, text_node, message);
      } catch (const std::runtime_error& error) {
        CHECK(std::string(error.what()) == message);
        failed = true;
      }
      CHECK(failed);
    };
    for (const auto* message : {"Cannot write calibration output",
                                "Cannot write evaluation report"}) {
      write_fails(directory / "missing-parent/text.yaml", message);
      CHECK(!std::filesystem::exists(directory / "missing-parent"));
      write_fails(directory, message);
#ifdef __linux__
      write_fails("/dev/full", message);
#endif
    }

    // 报告文件单独变化也必须可追溯；记录的是已装载证据，而非看到文件就授予资格。
    const auto data = test::calibration_dataset();
    const auto solution = vision::solve_intrinsics(data);
    auto intrinsic_report = vision::intrinsic_report(data, solution,
        {"synthetic-camera", "synthetic-v1", "2026-09-30", "synthetic independent holdout"},
        core::ClockDomain::replay, 0.01);
    namespace detail = pipeline::annotation_detail;
    const auto report_path = directory / "intrinsics.yaml";
    detail::write_yaml(report_path, intrinsic_report);
    auto calibration = YAML::LoadFile((root / "config/offline/calibration.yaml").string());
    calibration["calibration"]["camera_matrix"] =
        std::vector<double>(solution.matrix.val, solution.matrix.val + 9);
    calibration["calibration"]["distortion"] = solution.distortion;
    calibration["calibration"]["intrinsic_report_file"] = "intrinsics.yaml";
    detail::write_yaml(directory / "calibration.yaml", calibration);
    auto configuration = YAML::LoadFile((root / "config/offline/armor.yaml").string());
    configuration["calibration_file"] = "calibration.yaml";
    configuration["geometry_files"][0] = (root / "config/offline/geometry.yaml").string();
    const auto config_path = directory / "config.yaml";
    detail::write_yaml(config_path, configuration);
    const auto loaded = pipeline::load_pipeline_config(config_path);
    CHECK(loaded);
    const auto original = pipeline::describe_run(config_path, loaded.value());
    CHECK(original["run_metadata_schema_version"].as<int>() == 2);
    CHECK(original["calibration_reports"]["intrinsics"]["evidence_level"].as<std::string>() ==
          "simulation");
    CHECK(original["calibration_reports"]["extrinsics"]["file"].IsNull());
    CHECK(!original["calibration_reports"]["extrinsics"]["qualified"].as<bool>());
    const auto original_hash = original["calibration_reports"]["intrinsics"]["file"]
        ["fnv1a64"].as<std::string>();
    detail::write_bytes(report_path, detail::read_bytes(report_path) + "\n# different bytes\n");
    const auto changed = pipeline::describe_run(config_path, loaded.value());
    CHECK(changed["calibration_reports"]["intrinsics"]["file"]["fnv1a64"]
          .as<std::string>() != original_hash);
    CHECK(YAML::Dump(changed["calibration_parameters"]) ==
          YAML::Dump(original["calibration_parameters"]));
    CHECK(YAML::Dump(changed["calibration_file"]) == YAML::Dump(original["calibration_file"]));
    intrinsic_report.remove("intrinsic_quality");
    detail::write_yaml(report_path, intrinsic_report);
    const auto legacy = pipeline::load_pipeline_config(config_path);
    CHECK(legacy);
    CHECK(pipeline::describe_run(config_path, legacy.value())["calibration_reports"]
          ["intrinsics"]["evidence_level"].as<std::string>() == "missing");

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
    CHECK(report["report_schema_version"].as<int>() == 1);
    CHECK(report["command_line"].size() == arguments.size());
    CHECK(report["working_directory"].as<std::string>() ==
          std::filesystem::current_path().string());
    CHECK(report["annotations_file"].IsNull() && report["annotations_reviewed"].IsNull());
    CHECK(report["annotations_fingerprint"].IsNull());
    CHECK(report["pose_metrics"].as<std::string>() == "not_produced");
    CHECK(report["dataset_fingerprint"]["files"].size() == 4);
    CHECK(report["build"]["source_sha256"].as<std::string>().size() == 64);
    CHECK(report["build"]["compiler"].as<std::string>().size() > 0);
    CHECK(report["build"]["opencv_version"].as<std::string>().size() > 0);
    const auto report_bytes = detail::read_bytes(directory / "comparison/report.yaml");
    const auto timing_bytes = detail::read_bytes(directory / "comparison/timing.yaml");
    std::ostringstream diagnostics;
    auto* previous_stderr = std::cerr.rdbuf(diagnostics.rdbuf());
    const int repeated = pipeline::run_detector_benchmark(int(argv.size()), argv.data());
    std::cerr.rdbuf(previous_stderr);
    CHECK(repeated == 1);
    CHECK(diagnostics.str() ==
          "batch benchmark: Configurations/dataset/new output directory required\n");
    CHECK(detail::read_bytes(directory / "comparison/report.yaml") == report_bytes);
    CHECK(detail::read_bytes(directory / "comparison/timing.yaml") == timing_bytes);

    const auto run = [](std::vector<std::string> values, bool annotation = false) {
      std::vector<char*> pointers;
      for (auto& value : values)
        pointers.push_back(value.data());

      return annotation ? pipeline::run_annotate_session(int(pointers.size()), pointers.data()) :
                          pipeline::run_detector_benchmark(int(pointers.size()), pointers.data());
    };
    for (const std::string value : {"missing", "", "abc", "0", "-1", "1.1", "NaN", "Inf",
                                    "0.5junk", "1e9999"}) {
      std::vector<std::string> invalid{"bench_detector", "--dataset",
          (directory / "input/events.yaml").string(), "--config",
          (root / "config/offline/armor.yaml").string(), "--output",
          (directory / "invalid").string()};
      if (value != "missing") {
        invalid.push_back("--iou");
        invalid.push_back(value);
      }
      CHECK(run(invalid) == 1);
      CHECK(!std::filesystem::exists(directory / "invalid"));
    }
    for (const std::string flag : {"--pose-position-limit-m", "--pose-rotation-limit-rad"}) {
      for (const std::string value : {"0.5junk", "", "abc", "NaN", "Inf", "-1", "1e9999"}) {
        std::vector<std::string> invalid{"bench_detector", "--dataset",
            (directory / "input/events.yaml").string(), "--config",
            (root / "config/offline/armor.yaml").string(), "--iou", "0.5", "--output",
            (directory / "invalid-pose").string(), "--pose-reference", "test-reference",
            "--pose-position-limit-m", "0", "--pose-rotation-limit-rad", "0", flag, value};
        CHECK(run(invalid) == 1);
        CHECK(!std::filesystem::exists(directory / "invalid-pose"));
      }
    }
    CHECK(run({"bench_detector", "--dataset", (directory / "input/events.yaml").string(),
        "--config", (root / "config/offline/armor.yaml").string(), "--iou", "0.5", "--output",
        (directory / "zero-pose-limits").string(), "--pose-reference", "test-reference",
        "--pose-position-limit-m", "0", "--pose-rotation-limit-rad", "0"}) == 0);

    CHECK(run({"annotate", "--export", (directory / "annotation-draft").string(),
               "--session", (directory / "input").string()}, true) == 0);
    const auto annotations_file = directory / "annotation-draft/annotations.yaml";
    auto annotations = detail::read_document(annotations_file);
    annotations["reviewed"] = true;
    annotations["frames"].remove(2);
    detail::write_yaml(annotations_file, annotations);
    const auto certified = directory / "certified";
    CHECK(run({"annotate", "--apply", "--session", (directory / "input").string(),
               "--annotations", annotations_file.string(), "--output", certified.string()},
               true) == 0);
    std::vector<std::string> verified{"bench_detector", "--dataset",
        (certified / "events.yaml").string(), "--config",
        (root / "config/offline/armor.yaml").string(), "--iou", "1", "--output",
        (directory / "verified-report").string()};
    CHECK(run(verified) == 0);
    const auto verified_report = detail::read_document(directory / "verified-report/report.yaml");
    CHECK(verified_report["annotations_reviewed"].as<bool>());
    CHECK(verified_report["annotations_file"].as<std::string>() ==
          (certified / "annotations.yaml").string());
    CHECK(verified_report["annotations_fingerprint"]["fnv1a64"].as<std::string>().size() == 16);
    const auto first = detail::read_bytes(directory / "verified-report/report.yaml");
    std::filesystem::rename(directory / "verified-report", directory / "saved-report");
    CHECK(run(verified) == 0);
    CHECK(first == detail::read_bytes(directory / "verified-report/report.yaml"));
    const auto pristine_labels = detail::read_bytes(certified / "annotations.yaml");
    detail::write_bytes(certified / "annotations.yaml", pristine_labels + "# changed\n");
    verified.back() = (directory / "tampered-report").string();
    CHECK(run(verified) == 1);
    CHECK(!std::filesystem::exists(directory / "tampered-report"));

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
