#pragma once

#include "autoaim/vision/calibration_solver.hpp"

namespace autoaim::vision::calibration_detail {
using IntrinsicInformation = Eigen::Matrix<double, 9, 9>;

// 数值可辨识性下限，不是设备精度门限；列归一化消除 m/px/畸变参数量纲影响。
inline constexpr double minimum_information_ratio = 1e-10;

// 有序二维点的 FNV-1a64 + 字节数，仅用于内容去重，不证明素材真实性。
std::string point_fingerprint(const std::vector<cv::Point2f>& points);
IntrinsicInformation intrinsic_information(const CalibrationDataset& data,
                                            const IntrinsicSolution& solution);
bool information_usable(const IntrinsicInformation& information);
} // namespace autoaim::vision::calibration_detail
