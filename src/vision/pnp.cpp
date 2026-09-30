#include "autoaim/vision/pnp.hpp"
#include "autoaim/math/numeric.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace autoaim::vision {
PlateDimensions::PlateDimensions(core::Metres width_m, core::Metres height_m)
    : width(width_m), height(height_m) {
  if (width.value() <= 0 || height.value() <= 0)
    throw std::invalid_argument("Nonpositive plate size");
}

std::array<cv::Point3d, 4> object_corners(const PlateDimensions& size) {
  const double x = size.width.value() / 2, y = size.height.value() / 2;

  return {cv::Point3d(-x, y, 0), {x, y, 0}, {x, -y, 0}, {-x, -y, 0}};
}

#if !AUTOAIM_I2_LM
std::vector<PoseCandidate> ippe_candidates(const Detection& detection,
                                           const PlateDimensions& dimensions,
                                           const Calibration& calibration) {
  std::vector<cv::Point2d> pixels;

  for (const auto& point : detection.corners) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      return {};

    pixels.emplace_back(point.x, point.y);
  }

  const auto points = object_corners(dimensions);
  const std::vector<cv::Point3d> objects(points.begin(), points.end());
  const auto edge = pixels[1] - pixels[0];

  if (!std::isfinite(edge.x) || !std::isfinite(edge.y) || cv::norm(edge) == 0)
    return {};

  const double theta = std::atan2(edge.y, edge.x);
  const double c = std::cos(theta), s = std::sin(theta);
  const cv::Matx33d solver_from_plate(c, s, 0, s, -c, 0, 0, 0, -1);
  std::vector<cv::Point3d> solver_objects;

  // S 是正交且 det=+1 的等价板系，避开 OpenCV 4.5.4 IPPE rot2vec 的半周奇点。
  // 不改角点对应；仍只调用一次 IPPE，返回后恢复 R = R' S，保留双候选。
  for (const auto& point : objects) {
    const auto transformed = solver_from_plate * cv::Vec3d(point.x, point.y, point.z);
    solver_objects.emplace_back(transformed[0], transformed[1], transformed[2]);
  }

  std::vector<cv::Mat> rotations, translations;

  // IPPE（非 IPPE_SQUARE）支持非正方形共面点；Generic 接口保留多解。
  // https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
  cv::solvePnPGeneric(solver_objects, pixels, calibration.intrinsic(), calibration.distortion(),
                      rotations, translations, false, cv::SOLVEPNP_IPPE);

  std::vector<PoseCandidate> candidates;

  for (std::size_t i = 0; i < rotations.size(); ++i) {
    if (!cv::checkRange(rotations[i]) || !cv::checkRange(translations[i]))
      continue;

    cv::Mat rotation;
    cv::Rodrigues(rotations[i], rotation);
    rotation = rotation * cv::Mat(solver_from_plate);
    Eigen::Matrix3d matrix;
    Eigen::Vector3d translation;

    for (int row = 0; row < 3; ++row) {
      translation[row] = translations[i].at<double>(row);

      for (int col = 0; col < 3; ++col)
        matrix(row, col) = rotation.at<double>(row, col);
    }

    math::SE3 pose(Eigen::Quaterniond(matrix), translation);
    bool positive_depth = true;

    for (const auto& point : objects)
      positive_depth = positive_depth && pose.apply({point.x, point.y, point.z}).z() > 0;

    if (!positive_depth)
      continue;

    std::vector<cv::Point2d> projected;
    cv::Mat restored_rotation;
    cv::Rodrigues(rotation, restored_rotation);
    cv::projectPoints(objects, restored_rotation, translations[i], calibration.intrinsic(),
                      calibration.distortion(), projected);

    double sum = 0, maximum = 0;

    for (std::size_t k = 0; k < pixels.size(); ++k) {
      const double error = cv::norm(projected[k] - pixels[k]);
      sum += error * error;
      maximum = std::max(maximum, error);
    }

    const double cosine = (matrix * Eigen::Vector3d::UnitZ()).dot(-translation.normalized());
    candidates.push_back({PlateToCamera(std::move(pose)), std::sqrt(sum / 4), maximum, cosine});
  }

  std::stable_sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    return a.rms_px < b.rms_px;
  });

  return candidates;
}
#else

// I2-LM is selected by its CMake algorithm option.
// Equal-weight corner reprojection only: no image gradients, temporal prior or new evidence.
std::vector<PoseCandidate> ippe_candidates(const Detection& detection,
                                           const PlateDimensions& dimensions,
                                           const Calibration& calibration) {
  std::vector<cv::Point2d> pixels;
  for (const auto& point : detection.corners) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      return {};
    pixels.emplace_back(point.x, point.y);
  }

  const auto points = object_corners(dimensions);
  const std::vector<cv::Point3d> objects(points.begin(), points.end());
  const auto edge = pixels[1] - pixels[0];
  if (!std::isfinite(edge.x) || !std::isfinite(edge.y) || cv::norm(edge) == 0)
    return {};

  const double theta = std::atan2(edge.y, edge.x);
  const double c = std::cos(theta), s = std::sin(theta);
  const cv::Matx33d solver_from_plate(c, s, 0, s, -c, 0, 0, 0, -1);
  std::vector<cv::Point3d> solver_objects;
  for (const auto& point : objects) {
    const auto transformed = solver_from_plate * cv::Vec3d(point.x, point.y, point.z);
    solver_objects.emplace_back(transformed[0], transformed[1], transformed[2]);
  }

  // Evaluate both original and refined poses in the original physical plate frame.
  const auto evaluate = [&](const cv::Mat& rvec, const cv::Mat& tvec)
      -> std::optional<PoseCandidate> {
    if (!cv::checkRange(rvec) || !cv::checkRange(tvec))
      return std::nullopt;
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    rotation = rotation * cv::Mat(solver_from_plate);
    Eigen::Matrix3d matrix;
    Eigen::Vector3d translation;
    for (int row = 0; row < 3; ++row) {
      translation[row] = tvec.at<double>(row);
      for (int col = 0; col < 3; ++col)
        matrix(row, col) = rotation.at<double>(row, col);
    }
    if (!matrix.allFinite() || !translation.allFinite() || translation.norm() == 0)
      return std::nullopt;
    math::SE3 pose(Eigen::Quaterniond(matrix), translation);
    for (const auto& point : objects)
      if (!(pose.apply({point.x, point.y, point.z}).z() > 0))
        return std::nullopt;

    std::vector<cv::Point2d> projected;
    cv::Mat restored_rotation;
    cv::Rodrigues(rotation, restored_rotation);
    cv::projectPoints(objects, restored_rotation, tvec, calibration.intrinsic(),
                        calibration.distortion(), projected);
    double sum = 0, maximum = 0;
    for (std::size_t k = 0; k < pixels.size(); ++k) {
      const double error = cv::norm(projected[k] - pixels[k]);
      if (!std::isfinite(error))
        return std::nullopt;
      sum += error * error;
      maximum = std::max(maximum, error);
    }
    const double cosine = (matrix * Eigen::Vector3d::UnitZ()).dot(-translation.normalized());
    if (!std::isfinite(sum) || !std::isfinite(cosine))
      return std::nullopt;
    return PoseCandidate{PlateToCamera(std::move(pose)), std::sqrt(sum / 4), maximum, cosine};
  };

  std::vector<cv::Mat> rotations, translations;
  try {
    cv::solvePnPGeneric(solver_objects, pixels, calibration.intrinsic(), calibration.distortion(),
                          rotations, translations, false, cv::SOLVEPNP_IPPE);
  } catch (const cv::Exception&) {
    return {};
  }

  std::vector<PoseCandidate> originals, candidates;
  for (std::size_t i = 0; i < rotations.size(); ++i) {
    std::optional<PoseCandidate> original;
    try {
      original = evaluate(rotations[i], translations[i]);
    } catch (const cv::Exception&) {
      continue;
    }
    if (!original)
      continue;
    originals.push_back(*original);
    candidates.push_back(*original);
    try {
      auto refined_rotation = rotations[i].clone();
      auto refined_translation = translations[i].clone();
      // Refine each IPPE seed in the same equivalent solver frame, never seed from the winner.
      cv::solvePnPRefineLM(solver_objects, pixels, calibration.intrinsic(),
                            calibration.distortion(), refined_rotation, refined_translation,
                            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                               20, 1e-6));
      const auto refined = evaluate(refined_rotation, refined_translation);
      if (refined && refined->rms_px <= original->rms_px &&
          refined->maximum_error_px <= original->maximum_error_px)
        candidates.back() = *refined;
    } catch (const cv::Exception&) {
      // A failed/invalid refinement retains this seed's valid original candidate.
    }
  }

  // LM may converge both seeds to one minimum. Keep the original pair in that case;
  // duplicated local minima are not evidence that the other planar branch disappeared.
  // The 1e-8/1e-6 before and 1e-5/1e-4 after limits below are experimental numerical
  // duplicate tolerances (relative metres/radians), not physical acceptance thresholds.
  for (std::size_t i = 0; i < candidates.size(); ++i)
    for (std::size_t j = i + 1; j < candidates.size(); ++j) {
      const auto& first = candidates[i].plate_to_camera.value();
      const auto& second = candidates[j].plate_to_camera.value();
      const auto& old_first = originals[i].plate_to_camera.value();
      const auto& old_second = originals[j].plate_to_camera.value();
      const double scale = std::max({1.0, first.translation().norm(), second.translation().norm()});
      const double old_scale =
          std::max({1.0, old_first.translation().norm(), old_second.translation().norm()});
      const bool distinct_before =
          (old_first.translation() - old_second.translation()).norm() > 1e-8 * old_scale ||
          math::rotation_log(old_first.rotation() * old_second.rotation().conjugate()).norm() >
              1e-6;
      const bool merged_after =
          (first.translation() - second.translation()).norm() <= 1e-5 * scale &&
          math::rotation_log(first.rotation() * second.rotation().conjugate()).norm() <= 1e-4;
      if (distinct_before && merged_after)
        candidates = originals;
    }

  std::stable_sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    return a.rms_px < b.rms_px;
  });
  return candidates;
}
#endif

bool PlateIdentity::operator==(const PlateIdentity& other) const noexcept {
  return target_id == other.target_id && physical_plate == other.physical_plate &&
         profile_id == other.profile_id;
}

namespace {
Eigen::Matrix<double, 8, 1> project(const math::SE3& pose, const PlateDimensions& dimensions,
                                    const Calibration& calibration) {
  cv::Matx33d matrix;
  const auto rotation = pose.rotation().toRotationMatrix();

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      matrix(r, c) = rotation(r, c);

  cv::Vec3d rvec, tvec(pose.translation().x(), pose.translation().y(), pose.translation().z());
  cv::Rodrigues(matrix, rvec);
  const auto corners = object_corners(dimensions);
  const std::vector<cv::Point3d> objects(corners.begin(), corners.end());
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(objects, rvec, tvec, calibration.intrinsic(), calibration.distortion(), pixels);
  Eigen::Matrix<double, 8, 1> result;

  for (int k = 0; k < 4; ++k) {
    result[2 * k] = pixels[k].x;
    result[2 * k + 1] = pixels[k].y;
  }

  return result;
}

std::optional<PoseCovariance> covariance(const PoseCandidate& candidate, const Detection& detection,
                                         const PlateDimensions& dimensions,
                                         const Calibration& calibration,
                                         const PnpQualityOptions& options) {
  const auto& pose = candidate.plate_to_camera.value();
  Eigen::Matrix<double, 8, 6> jacobian;
  constexpr double step = 1e-5;

  for (int k = 0; k < 6; ++k) {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero(), rotation = translation;

    if (k < 3)
      translation[k] = step;
    else
      rotation[k - 3] = step;

    const math::SE3 plus(math::rotation_exp(rotation) * pose.rotation(),
                         pose.translation() + translation);

    const math::SE3 minus(math::rotation_exp(-rotation) * pose.rotation(),
                          pose.translation() - translation);

    jacobian.col(k) =
        (project(plus, dimensions, calibration) - project(minus, dimensions, calibration)) /
        (2 * step);
  }

  // 固定标定参数下的局部近似；多峰由候选集保留，不能靠放大单一高斯掩盖歧义。
  const double sigma = options.pixel_sigma_px / std::max(0.1, double(detection.confidence)) /
                       std::max(0.1, candidate.view_cosine) * (detection.corners_reliable ? 1 : 2);

  const auto information = (jacobian.transpose() * jacobian / (sigma * sigma)).eval();
  auto solved = math::solve_positive_definite(information, PoseCovariance::Identity(),
                                              options.minimum_information_ratio);

  if (!solved)
    return std::nullopt;

  PoseCovariance result = solved.value();
  result = ((result + result.transpose()) / 2).eval();

  if (!math::covariance_valid(result))
    return std::nullopt;

  return result;
}
} // namespace

PnpEstimate solve_pose(const core::Stamp& source, const Detection& detection,
                       const PlateDimensions& dimensions, const Calibration& calibration,
                       const PnpQualityOptions& options, const std::string& device_id,
                       const std::string& calibration_id,
                       const std::optional<PlateIdentity>& identity,
                       const std::optional<SamePlatePrior>& prior) {
  for (const double value :
       {options.maximum_rms_px, options.maximum_corner_error_px, options.minimum_edge_px,
        options.ambiguity_rms_gap_px, options.pixel_sigma_px, options.minimum_information_ratio,
        options.prior_position_gate.value(), options.prior_rotation_gate.value(),
        options.prior_maximum_age.value()})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("Nonpositive PnP quality limit");

  if (!std::isfinite(options.minimum_view_cosine) || options.minimum_view_cosine <= 0 ||
      options.minimum_view_cosine > 1 || options.minimum_information_ratio >= 1)
    throw std::invalid_argument("Invalid PnP quality ratio");

  PnpEstimate result(source, dimensions);

  if (!std::isfinite(detection.confidence) || detection.confidence <= 0 || detection.confidence > 1)
    return result;

  std::vector<cv::Point2f> contour(detection.corners.begin(), detection.corners.end());

  for (int i = 0; i < 4; ++i) {
    const auto& p = contour[i];

    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
        p.x >= calibration.width() || p.y >= calibration.height() ||
        cv::norm(p - contour[(i + 1) % 4]) < options.minimum_edge_px)
      return result;
  }

  if (!cv::isContourConvex(contour) || cv::contourArea(contour, true) <= 0)
    return result;

  result.candidates = ippe_candidates(detection, dimensions, calibration);
  std::vector<std::size_t> accepted;

  for (std::size_t i = 0; i < result.candidates.size(); ++i) {
    auto& candidate = result.candidates[i];
    candidate.geometry_accepted = candidate.rms_px <= options.maximum_rms_px &&
                                  candidate.maximum_error_px <= options.maximum_corner_error_px &&
                                  candidate.view_cosine > 0;

    if (!candidate.geometry_accepted)
      continue;

    candidate.covariance = covariance(candidate, detection, dimensions, calibration, options);
    accepted.push_back(i);
  }

  if (accepted.empty())
    return result;

  result.pose_valid = true;
  result.selected = accepted.front();
  result.ambiguous = accepted.size() > 1 &&
                     result.candidates[accepted[1]].rms_px - result.candidates[accepted[0]].rms_px <
                         options.ambiguity_rms_gap_px;

  const bool same_board =
      prior && identity && identity->target_id != 0 && !identity->profile_id.empty() &&
      prior->identity == *identity && prior->source.generation == source.generation &&
      prior->source.frame_id < source.frame_id &&
      core::same_time(prior->predicted_for, source.exposure) &&
      core::fresh(prior->source.exposure, source.exposure, options.prior_maximum_age);

  bool continuity = true;

  if (same_board) {
    std::vector<std::size_t> consistent;

    for (const auto index : accepted) {
      const auto& value = result.candidates[index].plate_to_camera.value();
      const auto& prediction = prior->predicted_pose.value();

      if ((value.translation() - prediction.translation()).norm() <=
              options.prior_position_gate.value() &&
          math::rotation_log(value.rotation() * prediction.rotation().conjugate()).norm() <=
              options.prior_rotation_gate.value())
        consistent.push_back(index);
    }

    continuity = !consistent.empty();

    if (consistent.size() == 1) {
      result.selected = consistent.front();
      result.ambiguous = false;
    }
  }

  const auto& selected = result.candidates[*result.selected];
  result.pose_reliable = detection.corners_reliable && !result.ambiguous && continuity &&
                         selected.view_cosine >= options.minimum_view_cosine &&
                         selected.covariance &&
                         calibration.qualified(source.exposure.domain(), device_id, calibration_id);

  return result;
}
} // namespace autoaim::vision
