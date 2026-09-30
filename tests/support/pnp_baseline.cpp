// Default implementation retained only for alternative regression tests.
// Keep synchronized with the production #if !AUTOAIM_I2_LM branch.
#include "autoaim/vision/pnp.hpp"
#include <opencv2/calib3d.hpp>
#include <algorithm>

namespace autoaim::vision {
std::vector<PoseCandidate> ippe_candidates_baseline(const Detection& detection,
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
} // namespace autoaim::vision
