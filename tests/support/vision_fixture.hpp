#pragma once

#include "autoaim/vision/pnp.hpp"
#include <opencv2/calib3d.hpp>

namespace test {
inline autoaim::vision::Calibration synthetic_calibration() {
  using namespace autoaim;
  const auto evidence = core::Evidence::from_report(core::MeasurementReport::evaluate(
      {"synthetic-camera", "v1", "2026-09-29", "synthetic projection"}, {0, 0}, 0.1,
      core::ClockDomain::replay));

  return vision::Calibration(1280, 1024, {1200, 0, 640, 0, 1200, 512, 0, 0, 1},
                             {0.01, -0.01, 0.001, 0.001, 0},
                             math::Transform<math::CameraFrame, math::GimbalFrame>(math::SE3(
                                 Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero())),
                             evidence, evidence);
}

inline autoaim::vision::Detection project_plate(const autoaim::math::SE3& pose,
                                                const autoaim::vision::PlateDimensions& dimensions,
                                                const autoaim::vision::Calibration& calibration) {
  using namespace autoaim;
  cv::Matx33d rotation;
  const auto matrix = pose.rotation().toRotationMatrix();

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      rotation(r, c) = matrix(r, c);

  cv::Vec3d rvec, tvec(pose.translation().x(), pose.translation().y(), pose.translation().z());
  cv::Rodrigues(rotation, rvec);
  const auto corners = vision::object_corners(dimensions);
  const std::vector<cv::Point3d> points(corners.begin(), corners.end());
  std::vector<cv::Point2d> projected;
  cv::projectPoints(points, rvec, tvec, calibration.intrinsic(), calibration.distortion(),
                    projected);

  vision::Detection detection{{}, 1, vision::TeamColor::red, 3, true};

  for (std::size_t i = 0; i < 4; ++i)
    detection.corners[i] = projected[i];
  detection.category = vision::TargetCategory::three;

  return detection;
}
} // namespace test
