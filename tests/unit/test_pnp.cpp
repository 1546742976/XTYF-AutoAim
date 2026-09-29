#include "autoaim/vision/pnp.hpp"
#include "support/vision_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto calibration = test::synthetic_calibration();
    const vision::PlateDimensions dimensions(core::Metres(0.135), core::Metres(0.055));
    const auto facing = Eigen::Quaterniond(Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitX()));
    const auto rotation = Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.9, Eigen::Vector3d::UnitY()) * facing;
    const math::SE3 truth(rotation, {0.2, 0.1, 1.5});
    auto detection = test::project_plate(truth, dimensions, calibration);
    const auto candidates = vision::ippe_candidates(detection, dimensions, calibration);
    CHECK(candidates.size() == 2);
    CHECK(candidates.front().rms_px < 0.001);
    CHECK((candidates.front().plate_to_camera.value().translation() - truth.translation()).norm() < 1e-4);
    CHECK(math::rotation_log(candidates.front().plate_to_camera.value().rotation() * truth.rotation().conjugate()).norm() < 1e-4);
    CHECK(candidates.front().view_cosine > 0);
    detection.corners[0].x = std::numeric_limits<float>::quiet_NaN();
    CHECK(vision::ippe_candidates(detection, dimensions, calibration).empty());
    CHECK_THROWS(std::invalid_argument, vision::PlateDimensions(core::Metres(0), core::Metres(1)));
  });
}
