#pragma once

#include "autoaim/core/config.hpp"
#include "autoaim/math/transform.hpp"
#include <opencv2/core.hpp>
#include <array>

namespace autoaim::vision {
class Calibration {
public:
  Calibration(int width, int height, cv::Matx33d intrinsic, std::vector<double> distortion,
              math::Transform<math::CameraFrame, math::GimbalFrame> extrinsic,
              core::Evidence intrinsic_evidence, core::Evidence extrinsic_evidence,
              std::array<int, 2> roi_offset = {0, 0});

  const std::array<int, 2>& roi_offset() const noexcept {
    return roi_offset_;
  }

  int width() const noexcept {
    return width_;
  }

  int height() const noexcept {
    return height_;
  }

  const cv::Matx33d& intrinsic() const noexcept {
    return intrinsic_;
  }

  const std::vector<double>& distortion() const noexcept {
    return distortion_;
  }

  const math::Transform<math::CameraFrame, math::GimbalFrame>& camera_to_gimbal() const noexcept {
    return extrinsic_;
  }

  bool qualified(core::ClockDomain domain, const std::string& device,
                 const std::string& configuration) const;

private:
  int width_;
  int height_;
  std::array<int, 2> roi_offset_;
  cv::Matx33d intrinsic_;
  std::vector<double> distortion_;
  math::Transform<math::CameraFrame, math::GimbalFrame> extrinsic_;
  core::Evidence intrinsic_evidence_;
  core::Evidence extrinsic_evidence_;
};

// 配置布尔项最高为声明。可选报告路径相对 report_base，只有匹配参数的验证报告可提升证据。
core::Result<Calibration> load_calibration(const core::Config& config,
                                          const std::filesystem::path& report_base = {});
} // namespace autoaim::vision
