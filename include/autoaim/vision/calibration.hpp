#pragma once

#include "autoaim/core/config.hpp"
#include "autoaim/math/transform.hpp"
#include <opencv2/core.hpp>

namespace autoaim::vision {
class Calibration {
public:
  Calibration(int width, int height, cv::Matx33d intrinsic, std::vector<double> distortion,
      math::Transform<math::CameraFrame, math::GimbalFrame> extrinsic,
      core::Evidence intrinsic_evidence, core::Evidence extrinsic_evidence);
  int width() const noexcept { return width_; }
  int height() const noexcept { return height_; }
  const cv::Matx33d& intrinsic() const noexcept { return intrinsic_; }
  const std::vector<double>& distortion() const noexcept { return distortion_; }
  const math::Transform<math::CameraFrame, math::GimbalFrame>& camera_to_gimbal() const noexcept { return extrinsic_; }
  bool qualified(core::ClockDomain domain, const std::string& device, const std::string& configuration) const;
private:
  int width_;
  int height_;
  cv::Matx33d intrinsic_;
  std::vector<double> distortion_;
  math::Transform<math::CameraFrame, math::GimbalFrame> extrinsic_;
  core::Evidence intrinsic_evidence_;
  core::Evidence extrinsic_evidence_;
};

// YAML 中全部字段显式提供，证据最高为“声明”；不沿用旧设备的标定值作为默认值。
core::Result<Calibration> load_calibration(const core::Config& config);
}  // namespace autoaim::vision
