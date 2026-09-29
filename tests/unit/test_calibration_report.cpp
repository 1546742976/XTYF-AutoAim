#include "autoaim/vision/calibration_report.hpp"
#include "support/calibration_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
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
    CHECK(!evaluate(report).qualifies(core::ClockDomain::host_monotonic, "test-camera", "test-v1"));
    CHECK(!evaluate(report).qualifies(core::ClockDomain::replay, "other", "test-v1"));
    CHECK(!calibration.qualified(core::ClockDomain::replay, "test-camera", "test-v1"));
    CHECK_THROWS(std::invalid_argument, vision::intrinsic_report(data, solution,
        {"test-camera", "test-v1", "2026-09-29", "synthetic"},
        core::ClockDomain::host_monotonic, 0.01));

    auto bad = YAML::Clone(report);
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
