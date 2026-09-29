#pragma once

#include "autoaim/vision/calibration.hpp"
#include <optional>

namespace autoaim::vision {
enum class BoardKind { chessboard, symmetric_circles };

struct CalibrationBoard {
  BoardKind kind;
  int columns;
  int rows;
  double spacing_m; // 相邻内角点/圆心间距，米；不是棋盘外框尺寸。
  std::vector<cv::Point3f> object_points() const;
};

struct CalibrationView {
  std::string id;
  cv::Size image_size;
  std::vector<cv::Point2f> points;
  bool validation; // true 的样本不得参与拟合。
  std::optional<math::SE3> gimbal_to_reference;
};

struct CalibrationDataset {
  CalibrationBoard board;
  std::vector<CalibrationView> views;
  std::vector<std::string> rejected;
  bool simulated = true; // 未指定来源按模拟处理；不能由调用者选择时钟域来升级。
  std::array<int, 2> roi_offset{0, 0};
};

// 本地图片或显式二维点清单；不接受设备索引。路径相对于 manifest。
// 检测失败列入 rejected；尺寸、点序数量、重复样本和不完整位姿是输入错误。
CalibrationDataset load_calibration_dataset(const std::filesystem::path& manifest);

struct IntrinsicSolution {
  cv::Size image_size;
  cv::Matx33d matrix;
  std::vector<double> distortion; // k1,k2,p1,p2,k3
  double rms_px;
  std::vector<double> per_view_rms_px;
  std::vector<std::string> fitted_ids;
  std::array<int, 2> roi_offset{0, 0};
};

// 纯离线求解，不产生能力证据。验证残差与参数绑定后才可评估报告。
IntrinsicSolution solve_intrinsics(const CalibrationDataset& data);
std::vector<double> validate_intrinsics(const CalibrationDataset& data,
                                        const IntrinsicSolution& intrinsics);

struct HandEyeSolution {
  math::Transform<math::CameraFrame, math::GimbalFrame> camera_to_gimbal;
  math::SE3 board_to_reference;
  std::vector<double> fit_rotation_rad;
  std::vector<double> fit_translation_m;
};

struct HandEyeResiduals {
  std::vector<double> rotation_rad;
  std::vector<double> translation_m;
};

HandEyeSolution solve_hand_eye(const CalibrationDataset& data,
                               const IntrinsicSolution& intrinsics);
HandEyeResiduals validate_hand_eye(const CalibrationDataset& data,
                                   const IntrinsicSolution& intrinsics,
                                   const HandEyeSolution& solution);
} // namespace autoaim::vision
