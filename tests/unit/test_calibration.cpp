#include "autoaim/vision/calibration.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto config = core::Config::parse(R"(calibration:
  width: 640
  height: 480
  camera_matrix: [800, 0, 320, 0, 800, 240, 0, 0, 1]
  distortion: [0, 0, 0, 0, 0]
  camera_to_gimbal_quaternion_wxyz: [1, 0, 0, 0]
  camera_to_gimbal_translation_m: [0.1, 0, 0]
  intrinsics_declared: true
  extrinsics_declared: true
)");
    CHECK(config);
    auto calibration = vision::load_calibration(config.value());
    CHECK(calibration && calibration.value().width() == 640);
    CHECK_NEAR(calibration.value().intrinsic()(0, 0), 800, 0);
    CHECK_NEAR(calibration.value().camera_to_gimbal().value().translation().x(), 0.1, 1e-12);
    CHECK(!calibration.value().qualified(core::ClockDomain::host_monotonic, "sim", "v1"));
    CHECK(!vision::load_calibration(core::Config::parse("calibration: {}").value()));
    CHECK_THROWS(std::invalid_argument, vision::Calibration(640, 480, cv::Matx33d::zeros(),
      {0, 0, 0, 0}, calibration.value().camera_to_gimbal(), core::Evidence::missing(), core::Evidence::missing()));
  });
}
