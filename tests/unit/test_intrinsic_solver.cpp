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
