#include "autoaim/vision/calibration_report.hpp"
#include "support/calibration_fixture.hpp"
#include "../../src/vision/calibration_quality.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    CHECK(vision::calibration_detail::point_fingerprint({}) == "cbf29ce484222325:0");
    const std::vector<cv::Point2f> points{{0, 0}, {1, 2.5F}, {123.25F, 0.125F}};
    CHECK(vision::calibration_detail::point_fingerprint(points) == "a8998fd56c1dbaa2:24");
    auto negative_zero = points;
    negative_zero[0] = {-0.0F, -0.0F};
    CHECK(vision::calibration_detail::point_fingerprint(negative_zero) ==
          "a8998fd56c1dbaa2:24");

    const auto data = test::calibration_dataset();
    const auto solution = vision::solve_intrinsics(data);
    const vision::Calibration calibration(solution.image_size.width, solution.image_size.height,
        solution.matrix, solution.distortion,
        math::Transform<math::CameraFrame, math::GimbalFrame>(
            math::SE3(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero())),
        core::Evidence::declared(), core::Evidence::declared());
    auto report = vision::intrinsic_report(data, solution,
        {"test-camera", "test-v1", "2026-09-29", "synthetic independent holdout"},
        core::ClockDomain::replay, 0.01);
    const auto evaluate = [&](const YAML::Node& node) {
      YAML::Emitter emitter;
      emitter.SetDoublePrecision(17);
      emitter << node;
      const auto config = core::Config::parse(emitter.c_str());
      CHECK(config);

      return vision::calibration_report_evidence(config.value(), calibration,
                                                 vision::CalibrationCapability::intrinsics);
    };
    CHECK(evaluate(report).qualifies(core::ClockDomain::replay, "test-camera", "test-v1"));
    auto legacy = YAML::Clone(report);
    legacy.remove("intrinsic_quality");
    CHECK(evaluate(legacy).level() == core::EvidenceLevel::missing);
    CHECK(!evaluate(report).qualifies(core::ClockDomain::host_monotonic, "test-camera", "test-v1"));
    CHECK(!evaluate(report).qualifies(core::ClockDomain::replay, "other", "test-v1"));
    CHECK(!calibration.qualified(core::ClockDomain::replay, "test-camera", "test-v1"));
    CHECK_THROWS(std::invalid_argument, vision::intrinsic_report(data, solution,
        {"test-camera", "test-v1", "2026-09-29", "synthetic"},
        core::ClockDomain::host_monotonic, 0.01));

    auto bad = YAML::Clone(report);
    bad["passed"] = true;
    bad["intrinsic_quality"]["information"] = std::vector<std::vector<double>>(
        9, std::vector<double>(9, 1));
    CHECK(evaluate(bad).level() == core::EvidenceLevel::missing);
    bad = YAML::Clone(report);
    bad["intrinsic_quality"]["samples"][1]["points_fingerprint"] =
        bad["intrinsic_quality"]["samples"][0]["points_fingerprint"].as<std::string>();
    CHECK_THROWS(std::invalid_argument, evaluate(bad));
    bad = YAML::Clone(report);
    bad["intrinsic_quality"]["samples"][0]["id"] = "not-the-fitting-sample";
    CHECK_THROWS(std::invalid_argument, evaluate(bad));
    bad = YAML::Clone(report);
    bad["intrinsic_quality"]["minimum_information_ratio"] = 0;
    CHECK_THROWS(std::invalid_argument, evaluate(bad));
    bad = YAML::Clone(report);
    bad["domain"] = "host_monotonic";
    CHECK_THROWS(std::invalid_argument, evaluate(bad));
    bad = YAML::Clone(report);
    bad["passed"] = true;
    bad["validation"]["rms_px"][0] = 10;
    CHECK(evaluate(bad).level() == core::EvidenceLevel::missing);
    bad = YAML::Clone(report);
    bad["parameters"]["width"] = 1280;
    CHECK_THROWS(std::invalid_argument, evaluate(bad));
    bad = YAML::Clone(report);
    bad["parameters"]["roi_offset"] = std::vector<int>{2, 0};
    CHECK_THROWS(std::invalid_argument, evaluate(bad));
    bad = YAML::Clone(report);
    bad["capability"] = "extrinsics";
    CHECK_THROWS(std::invalid_argument, evaluate(bad));
    bad = YAML::Clone(report);
    bad["validation_ids"][0] = bad["fit_ids"][0].as<std::string>();
    CHECK_THROWS(std::invalid_argument, evaluate(bad));
    bad = YAML::Clone(report);
    bad["validation_ids"] = std::vector<std::string>{};
    bad["validation"]["rms_px"] = std::vector<double>{};
    CHECK(evaluate(bad).level() == core::EvidenceLevel::missing);
  });
}
