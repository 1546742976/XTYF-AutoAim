#include "autoaim/vision/calibration.hpp"
#include "autoaim/vision/calibration_report.hpp"
#include <algorithm>

namespace autoaim::vision {
Calibration::Calibration(int width, int height, cv::Matx33d intrinsic,
                         std::vector<double> distortion,
                         math::Transform<math::CameraFrame, math::GimbalFrame> extrinsic,
                         core::Evidence intrinsic_evidence, core::Evidence extrinsic_evidence,
                         std::array<int, 2> roi_offset)
    : width_(width), height_(height), roi_offset_(roi_offset), intrinsic_(intrinsic),
      distortion_(std::move(distortion)),
      extrinsic_(std::move(extrinsic)), intrinsic_evidence_(std::move(intrinsic_evidence)),
      extrinsic_evidence_(std::move(extrinsic_evidence)) {
  const auto count = distortion_.size();

  if (width <= 0 || height <= 0 || roi_offset_[0] < 0 || roi_offset_[1] < 0 ||
      !std::all_of(std::begin(intrinsic_.val), std::end(intrinsic_.val),
                   [](double value) {
                     return std::isfinite(value);
                   }) ||
      intrinsic_(0, 0) <= 0 || intrinsic_(1, 1) <= 0 || intrinsic_(0, 1) != 0 ||
      intrinsic_(1, 0) != 0 || intrinsic_(2, 0) != 0 || intrinsic_(2, 1) != 0 ||
      intrinsic_(2, 2) != 1 ||
      (count != 4 && count != 5 && count != 8 && count != 12 && count != 14) ||
      !std::all_of(distortion_.begin(), distortion_.end(), [](double value) {
        return std::isfinite(value);
      }))
    throw std::invalid_argument("Invalid camera calibration");
}

bool Calibration::qualified(core::ClockDomain domain, const std::string& device,
                            const std::string& configuration) const {
  return intrinsic_evidence_.qualifies(domain, device, configuration) &&
         extrinsic_evidence_.qualifies(domain, device, configuration);
}

core::Result<Calibration> load_calibration(const core::Config& config,
                                          const std::filesystem::path& report_base) {
  try {
    const auto matrix = config.require<std::vector<double>>("calibration.camera_matrix");
    const auto q =
        config.require<std::vector<double>>("calibration.camera_to_gimbal_quaternion_wxyz");

    const auto t =
        config.require<std::vector<double>>("calibration.camera_to_gimbal_translation_m");

    if (matrix.size() != 9 || q.size() != 4 || t.size() != 3)
      throw std::invalid_argument("Invalid calibration dimensions");

    cv::Matx33d intrinsic;
    std::copy(matrix.begin(), matrix.end(), intrinsic.val);

    std::array<int, 2> roi{0, 0};
    if (config.contains("calibration.roi_offset")) {
      const auto values = config.require<std::vector<int>>("calibration.roi_offset");

      if (values.size() != 2)
        throw std::invalid_argument("Calibration ROI offset requires x,y");
      roi = {values[0], values[1]};
    }

    const Calibration candidate(
        config.require<int>("calibration.width"), config.require<int>("calibration.height"),
        intrinsic, config.require<std::vector<double>>("calibration.distortion"),
        math::Transform<math::CameraFrame, math::GimbalFrame>(
            math::SE3(Eigen::Quaterniond(q[0], q[1], q[2], q[3]), {t[0], t[1], t[2]})),
        config.declared_evidence("calibration.intrinsics_declared"),
        config.declared_evidence("calibration.extrinsics_declared"), roi);
    const auto evidence = [&](const char* key, const char* declared,
                              CalibrationCapability capability) {
      if (!config.contains(key))
        return config.declared_evidence(declared);

      const auto report = core::Config::load(report_base / config.require<std::string>(key));

      if (!report)
        throw std::invalid_argument(report.error().message);

      return calibration_report_evidence(report.value(), candidate, capability);
    };

    return core::Result<Calibration>::success(Calibration(
        candidate.width(), candidate.height(), candidate.intrinsic(), candidate.distortion(),
        candidate.camera_to_gimbal(),
        evidence("calibration.intrinsic_report_file", "calibration.intrinsics_declared",
                 CalibrationCapability::intrinsics),
        evidence("calibration.extrinsic_report_file", "calibration.extrinsics_declared",
                 CalibrationCapability::extrinsics), roi));
  } catch (const std::invalid_argument& error) {
    return core::Result<Calibration>::failure(core::ErrorCode::invalid_input, error.what());
  }
}
} // namespace autoaim::vision
