#pragma once

#include "autoaim/vision/calibration_solver.hpp"
#include <opencv2/calib3d.hpp>

namespace test {
inline autoaim::vision::CalibrationDataset calibration_dataset(double noise_px = 0) {
  using namespace autoaim;
  vision::CalibrationDataset data{{vision::BoardKind::chessboard, 7, 5, 0.04}, {}, {}};
  const auto objects = data.board.object_points();
  const cv::Matx33d matrix(800, 0, 320, 0, 810, 240, 0, 0, 1);
  cv::RNG rng(42);

  for (int i = 0; i < 24; ++i) {
    vision::CalibrationView view{std::to_string(i), {640, 480}, {}, i >= 20, std::nullopt};
    const cv::Vec3d r(0.4 * std::sin(i * 0.7), 0.45 * std::cos(i * 0.6), 0.08 * std::sin(i));
    const cv::Vec3d t(-0.12 + 0.04 * std::sin(i), -0.08 + 0.03 * std::cos(i),
                      0.8 + 0.03 * (i % 5));
    cv::projectPoints(objects, r, t, matrix, std::vector<double>{0.05, -0.02, 0.001, -0.001, 0.01},
                      view.points);

    for (auto& point : view.points) {
      point.x += float(rng.gaussian(noise_px));
      point.y += float(rng.gaussian(noise_px));
    }

    data.views.push_back(std::move(view));
  }

  return data;
}
} // namespace test
