#include "autoaim/vision/calibration_solver.hpp"
#include "calibration_quality.hpp"
#include <opencv2/calib3d.hpp>
#include <set>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <cstring>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace autoaim::vision {
namespace {
void check_views(const CalibrationDataset& data) {
  const auto count = data.board.object_points().size();
  std::set<std::string> ids;
  std::set<std::string> contents;

  if (data.views.empty())
    throw std::invalid_argument("No usable calibration views");

  const auto size = data.views.front().image_size;

  for (const auto& view : data.views) {
    if (view.id.empty() || !ids.insert(view.id).second || view.image_size != size ||
        size.width <= 0 || size.height <= 0 || view.points.size() != count)
      throw std::invalid_argument("Invalid calibration view identity/size/point count");

    for (const auto& p : view.points)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
          p.x >= size.width || p.y >= size.height)
        throw std::invalid_argument("Invalid calibration image point");

    if (!contents.insert(calibration_detail::point_fingerprint(view.points)).second)
      throw std::invalid_argument("Repeated calibration points across samples or splits");
  }
}

void check_intrinsics(const IntrinsicSolution& value) {
  Calibration(value.image_size.width, value.image_size.height, value.matrix, value.distortion,
              math::Transform<math::CameraFrame, math::GimbalFrame>(
                  math::SE3(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero())),
              core::Evidence::missing(), core::Evidence::missing());

  if (value.distortion.size() != 5)
    throw std::invalid_argument("Calibration solver requires five distortion coefficients");
}

double reprojection_rms(const std::vector<cv::Point3f>& objects,
                        const std::vector<cv::Point2f>& observed, const cv::Mat& rotation,
                        const cv::Mat& translation, const cv::Matx33d& matrix,
                        const std::vector<double>& distortion) {
  std::vector<cv::Point2f> predicted;
  cv::projectPoints(objects, rotation, translation, matrix, distortion, predicted);
  double squared = 0;

  for (std::size_t i = 0; i < observed.size(); ++i) {
    const auto delta = predicted[i] - observed[i];
    squared += delta.dot(delta);
  }

  return std::sqrt(squared / observed.size());
}
} // namespace

namespace calibration_detail {
std::string point_fingerprint(const std::vector<cv::Point2f>& points) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  std::uint64_t hash = 14695981039346656037ULL;

  for (const auto& point : points)
    for (float value : {point.x, point.y}) {
      if (value == 0)
        value = 0; // +0/-0 表示同一位置。
      std::uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      for (int byte = 0; byte < 4; ++byte) {
        hash ^= (bits >> (byte * 8)) & 0xffU;
        hash *= 1099511628211ULL;
      }
    }

  std::ostringstream result;
  result.imbue(std::locale::classic());
  result << std::hex << std::setw(16) << std::setfill('0') << hash << ':' << std::dec
         << points.size() * 8;

  return result.str();
}

IntrinsicInformation intrinsic_information(const CalibrationDataset& data,
                                            const IntrinsicSolution& solution) {
  check_views(data);
  check_intrinsics(solution);
  const auto objects = data.board.object_points();
  IntrinsicInformation information = IntrinsicInformation::Zero();
  std::vector<std::string> fitted;

  for (const auto& view : data.views) {
    if (view.validation)
      continue;

    fitted.push_back(view.id);
    cv::Mat rotation, translation, jacobian;
    if (!cv::solvePnP(objects, view.points, solution.matrix, solution.distortion,
                       rotation, translation, false, cv::SOLVEPNP_ITERATIVE))
      throw std::invalid_argument("Cannot assess intrinsic view pose: " + view.id);

    std::vector<cv::Point2f> projected;
    cv::projectPoints(objects, rotation, translation, solution.matrix, solution.distortion,
                        projected, jacobian);
    Eigen::MatrixXd pose(jacobian.rows, 6), intrinsic(jacobian.rows, 9);
    for (int row = 0; row < jacobian.rows; ++row) {
      for (int col = 0; col < 6; ++col)
        pose(row, col) = jacobian.at<double>(row, col);
      for (int col = 0; col < 9; ++col)
        intrinsic(row, col) = jacobian.at<double>(row, col + 6);
    }

    // 剔除各图六维位姿可以解释的像素变化；不能把重拟合位姿当内参约束。
    const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(pose);
    const Eigen::MatrixXd reduced = intrinsic - pose * decomposition.solve(intrinsic);
    information.noalias() += reduced.transpose() * reduced;
  }

  if (fitted != solution.fitted_ids || fitted.size() < 3)
    throw std::invalid_argument("Intrinsic fitting samples do not match solution");
  if (!information.allFinite() || (information.diagonal().array() <= 0).any())
    return IntrinsicInformation::Zero();

  const Eigen::Matrix<double, 9, 1> scale = information.diagonal().cwiseSqrt().cwiseInverse();
  information = (scale.asDiagonal() * information * scale.asDiagonal()).eval();

  return ((information + information.transpose()) / 2).eval();
}

bool information_usable(const IntrinsicInformation& information) {
  if (!information.allFinite() || !information.isApprox(information.transpose(), 1e-10) ||
      !information.diagonal().isApprox(Eigen::Matrix<double, 9, 1>::Ones(), 1e-8))
    return false;

  Eigen::SelfAdjointEigenSolver<IntrinsicInformation> solver(information, Eigen::EigenvaluesOnly);

  return solver.info() == Eigen::Success && solver.eigenvalues().maxCoeff() > 0 &&
         solver.eigenvalues().minCoeff() >
             minimum_information_ratio * solver.eigenvalues().maxCoeff();
}
} // namespace calibration_detail

IntrinsicSolution solve_intrinsics(const CalibrationDataset& data) {
  check_views(data);
  const auto objects = data.board.object_points();
  std::vector<std::vector<cv::Point3f>> object_sets;
  std::vector<std::vector<cv::Point2f>> image_sets;
  std::vector<std::string> ids;

  for (const auto& view : data.views)
    if (!view.validation) {
      object_sets.push_back(objects);
      image_sets.push_back(view.points);
      ids.push_back(view.id);
    }

  if (ids.size() < 3)
    throw std::invalid_argument("At least three fitting views required");

  cv::Mat matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distortion = cv::Mat::zeros(5, 1, CV_64F);
  std::vector<cv::Mat> rotations, translations;
  const double rms = cv::calibrateCamera(
      object_sets, image_sets, data.views.front().image_size, matrix, distortion,
      rotations, translations, 0, {cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9});

  IntrinsicSolution solution{data.views.front().image_size, {}, {}, rms, {}, ids};
  solution.roi_offset = data.roi_offset;
  matrix.copyTo(cv::Mat(solution.matrix, false));
  solution.distortion.assign(distortion.ptr<double>(), distortion.ptr<double>() + 5);
  check_intrinsics(solution);

  if (!std::isfinite(rms))
    throw std::runtime_error("Intrinsic optimization did not produce finite residuals");

  for (std::size_t i = 0; i < ids.size(); ++i)
    solution.per_view_rms_px.push_back(reprojection_rms(objects, image_sets[i], rotations[i],
                                                       translations[i], solution.matrix,
                                                       solution.distortion));

  if (!calibration_detail::information_usable(
          calibration_detail::intrinsic_information(data, solution)))
    throw std::invalid_argument("Intrinsic views have insufficient parameter observability");

  return solution;
}

std::vector<double> validate_intrinsics(const CalibrationDataset& data,
                                        const IntrinsicSolution& intrinsics) {
  check_views(data);
  check_intrinsics(intrinsics);

  if (data.views.front().image_size != intrinsics.image_size ||
      data.roi_offset != intrinsics.roi_offset)
    throw std::invalid_argument("Validation resolution differs from intrinsic calibration");

  const auto objects = data.board.object_points();
  std::vector<double> residuals;

  for (const auto& view : data.views)
    if (view.validation) {
      cv::Mat rotation, translation;

      if (!cv::solvePnP(objects, view.points, intrinsics.matrix, intrinsics.distortion,
                       rotation, translation, false, cv::SOLVEPNP_ITERATIVE) ||
          translation.at<double>(2) <= 0)
        throw std::runtime_error("Validation board pose failed: " + view.id);

      residuals.push_back(reprojection_rms(objects, view.points, rotation, translation,
                                           intrinsics.matrix, intrinsics.distortion));
    }

  return residuals;
}

namespace {
cv::Mat rotation_matrix(const math::SE3& pose) {
  cv::Mat rotation(3, 3, CV_64F);
  const auto eigen = pose.rotation().toRotationMatrix();

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      rotation.at<double>(r, c) = eigen(r, c);

  return rotation;
}

cv::Mat translation_vector(const math::SE3& pose) {
  return (cv::Mat_<double>(3, 1) << pose.translation().x(), pose.translation().y(),
          pose.translation().z());
}

math::SE3 from_cv(const cv::Mat& rotation, const cv::Mat& translation) {
  if (!cv::checkRange(rotation) || !cv::checkRange(translation) ||
      std::abs(cv::determinant(rotation) - 1) > 1e-5)
    throw std::runtime_error("Hand-eye solver produced invalid rigid transform");

  Eigen::Matrix3d matrix;

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      matrix(r, c) = rotation.at<double>(r, c);

  return math::SE3(Eigen::Quaterniond(matrix), {translation.at<double>(0),
                    translation.at<double>(1), translation.at<double>(2)});
}

math::SE3 board_pose(const CalibrationView& view, const CalibrationBoard& board,
                     const IntrinsicSolution& intrinsics) {
  cv::Mat rvec, tvec, rotation;

  if (!cv::solvePnP(board.object_points(), view.points, intrinsics.matrix, intrinsics.distortion,
                   rvec, tvec, false, cv::SOLVEPNP_ITERATIVE) || tvec.at<double>(2) <= 0)
    throw std::runtime_error("Cannot recover calibration board pose: " + view.id);

  cv::Rodrigues(rvec, rotation);

  return from_cv(rotation, tvec);
}

void require_hand_eye_input(const CalibrationDataset& data, const IntrinsicSolution& intrinsics) {
  check_views(data);
  check_intrinsics(intrinsics);

  if (data.views.front().image_size != intrinsics.image_size ||
      data.roi_offset != intrinsics.roi_offset)
    throw std::invalid_argument("Hand-eye resolution differs from intrinsics");

  for (const auto& view : data.views)
    if (!view.gimbal_to_reference)
      throw std::invalid_argument("Every hand-eye view requires complete gimbal_to_reference");
}

HandEyeResiduals pose_residuals(const std::vector<math::SE3>& poses, const math::SE3& reference) {
  HandEyeResiduals residuals;

  for (const auto& pose : poses) {
    residuals.rotation_rad.push_back(
        math::rotation_log(reference.rotation().conjugate() * pose.rotation()).norm());
    residuals.translation_m.push_back((reference.translation() - pose.translation()).norm());
  }

  return residuals;
}
} // namespace

HandEyeSolution solve_hand_eye(const CalibrationDataset& data,
                               const IntrinsicSolution& intrinsics) {
  require_hand_eye_input(data, intrinsics);
  std::vector<math::SE3> gimbal, board;

  for (const auto& view : data.views)
    if (!view.validation) {
      gimbal.push_back(*view.gimbal_to_reference);
      board.push_back(board_pose(view, data.board, intrinsics));
    }

  if (gimbal.size() < 3)
    throw std::invalid_argument("At least three fitting hand-eye poses required");

  Eigen::Matrix3d excitation = Eigen::Matrix3d::Zero();

  for (std::size_t i = 1; i < gimbal.size(); ++i) {
    const Eigen::Vector3d delta =
        math::rotation_log(gimbal.front().rotation().conjugate() * gimbal[i].rotation());
    excitation += delta * delta.transpose();

    for (std::size_t j = 0; j < i; ++j)
      if (math::rotation_log(gimbal[j].rotation().conjugate() * gimbal[i].rotation()).norm() <
              1e-8 &&
          (gimbal[j].translation() - gimbal[i].translation()).norm() < 1e-8)
        throw std::invalid_argument("Repeated fitting hand-eye pose");
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(excitation);

  // 至少两个非平行旋转轴；数值秩检查不是实机精度验收门限。
  if (eigen.info() != Eigen::Success || eigen.eigenvalues()(2) < 1e-8 ||
      eigen.eigenvalues()(1) / eigen.eigenvalues()(2) < 1e-6)
    throw std::invalid_argument("Hand-eye motions have insufficient rotational excitation");

  std::vector<cv::Mat> gimbal_r, gimbal_t, board_r, board_t;

  for (std::size_t i = 0; i < gimbal.size(); ++i) {
    gimbal_r.push_back(rotation_matrix(gimbal[i]));
    gimbal_t.push_back(translation_vector(gimbal[i]));
    board_r.push_back(rotation_matrix(board[i]));
    board_t.push_back(translation_vector(board[i]));
  }

  cv::Mat rotation, translation;
  cv::calibrateHandEye(gimbal_r, gimbal_t, board_r, board_t, rotation, translation,
                       cv::CALIB_HAND_EYE_PARK);
  const auto camera_to_gimbal = from_cv(rotation, translation);
  std::vector<math::SE3> world_boards;
  Eigen::Vector3d mean_t = Eigen::Vector3d::Zero();
  Eigen::Vector4d mean_q = Eigen::Vector4d::Zero();

  for (std::size_t i = 0; i < gimbal.size(); ++i) {
    world_boards.push_back(gimbal[i].compose(camera_to_gimbal).compose(board[i]));
    mean_t += world_boards.back().translation();
    Eigen::Vector4d q = world_boards.back().rotation().coeffs();

    if (q.dot(world_boards.front().rotation().coeffs()) < 0)
      q = -q;
    mean_q += q;
  }

  if (mean_q.norm() < 1e-8)
    throw std::runtime_error("Inconsistent board orientation after hand-eye calibration");

  const math::SE3 reference(Eigen::Quaterniond(mean_q.normalized()), mean_t / gimbal.size());
  const auto residuals = pose_residuals(world_boards, reference);

  return {math::Transform<math::CameraFrame, math::GimbalFrame>(camera_to_gimbal), reference,
          residuals.rotation_rad, residuals.translation_m};
}

HandEyeResiduals validate_hand_eye(const CalibrationDataset& data,
                                   const IntrinsicSolution& intrinsics,
                                   const HandEyeSolution& solution) {
  require_hand_eye_input(data, intrinsics);
  std::vector<math::SE3> world_boards;

  for (const auto& view : data.views)
    if (view.validation)
      world_boards.push_back(view.gimbal_to_reference->compose(solution.camera_to_gimbal.value())
                                .compose(board_pose(view, data.board, intrinsics)));

  // 参考板位姿只从拟合集估计，验证时不重拟合它来掩盖方向或偏移错误。
  return pose_residuals(world_boards, solution.board_to_reference);
}
} // namespace autoaim::vision
