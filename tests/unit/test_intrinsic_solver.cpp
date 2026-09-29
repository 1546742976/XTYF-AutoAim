#include "support/calibration_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    auto data = test::calibration_dataset();
    const auto solution = vision::solve_intrinsics(data);
    CHECK(solution.fitted_ids.size() == 20 && solution.per_view_rms_px.size() == 20);
    CHECK(solution.distortion.size() == 5);
    CHECK(solution.rms_px < 0.001);
    CHECK_NEAR(solution.matrix(0, 0), 800, 0.1);
    CHECK_NEAR(solution.matrix(1, 1), 810, 0.1);
    const auto residuals = vision::validate_intrinsics(data, solution);
    CHECK(residuals.size() == 4);

    for (auto value : residuals)
      CHECK(value < 0.001);

    auto noisy = test::calibration_dataset(0.1);
    const auto noisy_solution = vision::solve_intrinsics(noisy);
    CHECK_NEAR(noisy_solution.matrix(0, 0), 800, 8);
    CHECK(noisy_solution.rms_px > 0.01 && noisy_solution.rms_px < 0.3);
    auto repeated = test::calibration_dataset();
    for (auto& view : repeated.views)
      view.points = repeated.views.front().points;
    CHECK_THROWS(std::invalid_argument, vision::solve_intrinsics(repeated));

    auto contaminated = test::calibration_dataset();
    contaminated.views.back().points = contaminated.views.front().points;
    CHECK_THROWS(std::invalid_argument, vision::validate_intrinsics(contaminated, solution));

    // 平移变化不能替代平面标定所需的姿态激励；样本不是重复点集。
    auto frontal = test::calibration_dataset();
    for (std::size_t i = 0; i < frontal.views.size(); ++i)
      cv::projectPoints(frontal.board.object_points(), cv::Vec3d(0, 0, 0),
          cv::Vec3d(-0.12 + 0.002 * i, -0.08, 0.8 + 0.002 * i),
          cv::Matx33d(800, 0, 320, 0, 810, 240, 0, 0, 1),
          std::vector<double>{0, 0, 0, 0, 0}, frontal.views[i].points);
    CHECK_THROWS(std::invalid_argument, vision::solve_intrinsics(frontal));

    for (std::size_t i = 0; i < frontal.views.size(); ++i)
      cv::projectPoints(frontal.board.object_points(),
          cv::Vec3d(1e-5 * std::sin(double(i)), 1e-5 * std::cos(double(i)), 0),
          cv::Vec3d(-0.12 + 0.002 * i, -0.08, 0.8 + 0.002 * i),
          cv::Matx33d(800, 0, 320, 0, 810, 240, 0, 0, 1),
          std::vector<double>{0, 0, 0, 0, 0}, frontal.views[i].points);
    CHECK_THROWS(std::invalid_argument, vision::solve_intrinsics(frontal));

    noisy.views.back().image_size.width = 800;
    CHECK_THROWS(std::invalid_argument, vision::solve_intrinsics(noisy));
    data.views.back().points.front().x = NAN;
    CHECK_THROWS(std::invalid_argument, vision::solve_intrinsics(data));
    data = test::calibration_dataset();
    for (auto& view : data.views)
      view.validation = true;
    CHECK_THROWS(std::invalid_argument, vision::solve_intrinsics(data));
  });
}
