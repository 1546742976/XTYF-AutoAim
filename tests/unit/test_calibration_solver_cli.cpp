#include "autoaim/pipeline/offline_tools.hpp"
#include "support/calibration_fixture.hpp"
#include "test_support.hpp"
#include <chrono>
#include <fstream>

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto root = std::filesystem::temp_directory_path() /
        ("autoaim-calibration-cli-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    CHECK(std::filesystem::create_directory(root));
    struct Cleanup {
      std::filesystem::path path;
      ~Cleanup() {
        std::filesystem::remove_all(path);
      }
    } cleanup{root};

    YAML::Node manifest = YAML::Load(
        "board: {kind: chessboard, columns: 7, rows: 5, spacing_m: 0.04}\n"
        "image_width: 640\nimage_height: 480\nsource_kind: synthetic\n"
        "provenance: {device_id: cli-test, configuration_id: v1, date: '2026-09-29'}\n"
        "validation_limits: {rms_px: 0.01}\nsamples: []\n");

    for (const auto& view : test::calibration_dataset().views) {
      YAML::Node sample;
      sample["id"] = view.id;
      sample["split"] = view.validation ? "validation" : "fit";
      std::vector<std::vector<float>> points;

      for (const auto& p : view.points)
        points.push_back({p.x, p.y});
      sample["image_points"] = points;
      manifest["samples"].push_back(sample);
    }

    {
      YAML::Emitter emitter;
      emitter.SetFloatPrecision(9);
      emitter << manifest;
      std::ofstream file(root / "dataset.yaml");
      file << emitter.c_str();
      CHECK(file.good());
    }

    std::vector<std::string> arguments{"calibration_tool", "--solve-intrinsics",
        (root / "dataset.yaml").string(), "--output", (root / "result").string()};
    std::vector<char*> argv;

    for (auto& argument : arguments)
      argv.push_back(argument.data());

    CHECK(pipeline::run_calibration_tool(int(argv.size()), argv.data()) == 0);
    const auto output = YAML::LoadFile((root / "result/intrinsics.yaml").string());
    CHECK(output["calibration"]["distortion"].size() == 5);
    CHECK(!output["calibration"]["camera_to_gimbal_translation_m"]);
    const auto report = YAML::LoadFile((root / "result/intrinsic-report.yaml").string());
    CHECK(report["validation_ids"].size() == 4);
    CHECK(report["source_kind"].as<std::string>() == "synthetic");
    CHECK(pipeline::run_calibration_tool(int(argv.size()), argv.data()) == 1);

    // 独立手眼数据使用刚求得的内参投影，验证 CLI 的方向、文件复制与报告导入。
    cv::Matx33d matrix;
    const auto matrix_values = output["calibration"]["camera_matrix"].as<std::vector<double>>();
    std::copy(matrix_values.begin(), matrix_values.end(), matrix.val);
    const auto distortion = output["calibration"]["distortion"].as<std::vector<double>>();
    manifest["samples"] = YAML::Node(YAML::NodeType::Sequence);
    manifest["maximum_pair_skew_s"] = 0;
    manifest["validation_limits"]["rotation_rad"] = 0.001;
    manifest["validation_limits"]["translation_m"] = 0.001;
    const math::SE3 expected(math::rotation_exp({0.05, -0.1, 0.03}), {0.04, -0.02, 0.03});
    const math::SE3 reference(Eigen::Quaterniond::Identity(), {-0.12, -0.08, 1.6});

    for (int i = 0; i < 20; ++i) {
      const math::SE3 gimbal(math::rotation_exp({0.12 * std::sin(i), 0.14 * std::cos(i),
                                                0.04 * std::sin(i * 0.7)}),
                             {0.02 * std::sin(i), 0.03 * std::cos(i), 0});
      const auto board = gimbal.compose(expected).inverse().compose(reference);
      const auto r = math::rotation_log(board.rotation());
      const auto t = board.translation();
      std::vector<cv::Point2f> projected;
      cv::projectPoints(test::calibration_dataset().board.object_points(),
                        cv::Vec3d(r.x(), r.y(), r.z()), cv::Vec3d(t.x(), t.y(), t.z()),
                        matrix, distortion, projected);
      YAML::Node sample;
      sample["id"] = "hand-" + std::to_string(i);
      sample["split"] = i < 16 ? "fit" : "validation";
      sample["image_time_ns"] = i;
      sample["pose_time_ns"] = i;
      const auto q = gimbal.rotation();
      const auto gt = gimbal.translation();
      sample["gimbal_to_reference"]["quaternion_wxyz"] =
          std::vector<double>{q.w(), q.x(), q.y(), q.z()};
      sample["gimbal_to_reference"]["translation_m"] =
          std::vector<double>{gt.x(), gt.y(), gt.z()};

      for (const auto& p : projected)
        sample["image_points"].push_back(std::vector<float>{p.x, p.y});
      manifest["samples"].push_back(sample);
    }

    {
      YAML::Emitter emitter;
      emitter.SetDoublePrecision(17);
      emitter.SetFloatPrecision(9);
      emitter << manifest;
      std::ofstream file(root / "hand.yaml");
      file << emitter.c_str();
      CHECK(file.good());
    }

    arguments = {"calibration_tool", "--solve-hand-eye", (root / "hand.yaml").string(),
        "--intrinsics", (root / "result/intrinsics.yaml").string(), "--output",
        (root / "hand-result").string()};
    argv.clear();
    for (auto& argument : arguments)
      argv.push_back(argument.data());
    CHECK(pipeline::run_calibration_tool(int(argv.size()), argv.data()) == 0);
    const auto complete = core::Config::load(root / "hand-result/calibration.yaml");
    CHECK(complete);
    const auto calibration = vision::load_calibration(complete.value(), root / "hand-result");
    CHECK(calibration);
    CHECK(calibration.value().qualified(core::ClockDomain::replay, "cli-test", "v1"));
    CHECK(!calibration.value().qualified(core::ClockDomain::host_monotonic, "cli-test", "v1"));
    CHECK((calibration.value().camera_to_gimbal().value().translation() -
           expected.translation()).norm() < 1e-4);
  });
}
