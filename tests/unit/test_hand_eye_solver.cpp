#include "autoaim/vision/calibration_solver.hpp"
#include "test_support.hpp"
#include <opencv2/calib3d.hpp>

int main() {
  using namespace autoaim;

  return test::run([] {
    vision::CalibrationDataset data{{vision::BoardKind::chessboard, 7, 5, 0.04}, {}, {}};
    vision::IntrinsicSolution intrinsic{{640, 480}, {800, 0, 320, 0, 810, 240, 0, 0, 1},
                                         {0, 0, 0, 0, 0}, 0, {}, {}};
    const math::SE3 expected(math::rotation_exp({0.05, -0.1, 0.03}), {0.04, -0.02, 0.03});
    const math::SE3 reference(Eigen::Quaterniond::Identity(), {-0.12, -0.08, 1.6});

    for (int i = 0; i < 20; ++i) {
      const math::SE3 gimbal(math::rotation_exp({0.12 * std::sin(i), 0.14 * std::cos(i),
                                                0.04 * std::sin(i * 0.7)}),
                             {0.02 * std::sin(i), 0.03 * std::cos(i), 0});
      const auto board = gimbal.compose(expected).inverse().compose(reference);
      const auto axis = math::rotation_log(board.rotation());
      const auto t = board.translation();
      vision::CalibrationView view{std::to_string(i), {640, 480}, {}, i >= 16, gimbal};
      cv::projectPoints(data.board.object_points(), cv::Vec3d(axis.x(), axis.y(), axis.z()),
                        cv::Vec3d(t.x(), t.y(), t.z()), intrinsic.matrix, intrinsic.distortion,
                        view.points);
      data.views.push_back(std::move(view));
    }

    const auto solution = vision::solve_hand_eye(data, intrinsic);
    CHECK((solution.camera_to_gimbal.value().translation() - expected.translation()).norm() < 1e-4);
    CHECK(math::rotation_log(solution.camera_to_gimbal.value().rotation().conjugate() *
                             expected.rotation()).norm() < 1e-4);
    const auto validation = vision::validate_hand_eye(data, intrinsic, solution);
    CHECK(validation.rotation_rad.size() == 4 && validation.translation_m.size() == 4);

    for (std::size_t i = 0; i < 4; ++i) {
      CHECK(validation.rotation_rad[i] < 1e-4);
      CHECK(validation.translation_m[i] < 1e-4);
    }

    auto invalid = data;
    invalid.views[1].gimbal_to_reference = invalid.views[0].gimbal_to_reference;
    CHECK_THROWS(std::invalid_argument, vision::solve_hand_eye(invalid, intrinsic));
    invalid = data;
    invalid.views[0].gimbal_to_reference.reset();
    CHECK_THROWS(std::invalid_argument, vision::solve_hand_eye(invalid, intrinsic));
    invalid = data;

    for (std::size_t i = 0; i < invalid.views.size(); ++i)
      invalid.views[i].gimbal_to_reference.emplace(math::rotation_exp({0, 0, 0.01 * i}),
                                                  Eigen::Vector3d::Zero());

    CHECK_THROWS(std::invalid_argument, vision::solve_hand_eye(invalid, intrinsic));
    invalid = data;
    invalid.views.back().gimbal_to_reference = data.views.back().gimbal_to_reference->inverse();
    const auto wrong = vision::validate_hand_eye(invalid, intrinsic, solution);
    CHECK(wrong.rotation_rad.back() > 0.05 || wrong.translation_m.back() > 0.05);
  });
}
