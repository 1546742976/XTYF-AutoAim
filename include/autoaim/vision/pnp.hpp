#pragma once

#include "autoaim/vision/calibration.hpp"
#include "autoaim/vision/detection.hpp"
#include <optional>

namespace autoaim::vision {
struct PlateDimensions {
  core::Metres width;
  core::Metres height;
  PlateDimensions(core::Metres width_m, core::Metres height_m);
};

using PlateToCamera = math::Transform<math::PlateFrame, math::CameraFrame>;

// [相机系平移扰动(m), 相机系左乘旋转扰动(rad)]；平移和姿态分别扰动。
using PoseCovariance = Eigen::Matrix<double, 6, 6>;

struct PoseCandidate {
  PlateToCamera plate_to_camera;
  double rms_px;
  double maximum_error_px;
  double view_cosine;
  std::optional<PoseCovariance> covariance = std::nullopt;
  bool geometry_accepted = false;
};

// 板系：x 向物理右、y 向物理上、z 为外法向，右手系；相机 x 右/y 下/z 前。
// 物体点和输入像素都依次为物理 TL, TR, BR, BL，所有尺寸为米。
std::array<cv::Point3d, 4> object_corners(const PlateDimensions& dimensions);

// 保留所有正深度的 IPPE 候选；不以“大角度”拒绝，不施加零滚转/固定俯仰。
std::vector<PoseCandidate> ippe_candidates(const Detection& detection,
                                           const PlateDimensions& dimensions,
                                           const Calibration& calibration);

struct PlateIdentity {
  std::uint64_t target_id;
  std::size_t physical_plate;
  std::string profile_id;
  bool operator==(const PlateIdentity& other) const noexcept;
};

struct SamePlatePrior {
  PlateIdentity identity;
  core::Stamp source;
  core::TimePoint predicted_for;

  // 已经外推并转换到“本次曝光的相机系”，不是直接拿上一帧相机坐标比较。
  PlateToCamera predicted_pose;
};

struct PnpQualityOptions {
  double maximum_rms_px;
  double maximum_corner_error_px;
  double minimum_edge_px;
  double minimum_view_cosine;
  double ambiguity_rms_gap_px;
  double pixel_sigma_px;
  double minimum_information_ratio;
  core::Metres prior_position_gate;
  core::Radians prior_rotation_gate;
  core::Seconds prior_maximum_age;
};

struct PnpEstimate {
  const core::Stamp source;
  const PlateDimensions dimensions;
  std::vector<PoseCandidate> candidates;
  std::optional<std::size_t> selected;
  bool pose_valid;
  bool pose_reliable;
  bool ambiguous;

  PnpEstimate(core::Stamp stamp, PlateDimensions size)
      : source(stamp), dimensions(size), pose_valid(false), pose_reliable(false), ambiguous(false) {
  }
};

PnpEstimate solve_pose(const core::Stamp& source, const Detection& detection,
                       const PlateDimensions& dimensions, const Calibration& calibration,
                       const PnpQualityOptions& options, const std::string& device_id,
                       const std::string& calibration_id,
                       const std::optional<PlateIdentity>& identity,
                       const std::optional<SamePlatePrior>& prior);
} // namespace autoaim::vision
