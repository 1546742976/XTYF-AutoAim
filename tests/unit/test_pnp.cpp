#include "autoaim/vision/pnp.hpp"
#include "autoaim/math/angle.hpp"
#include "autoaim/vision/corner_refine.hpp"
#include "support/vision_fixture.hpp"
#include "test_support.hpp"

namespace {
void check_perturbed_geometry() {
  using namespace autoaim;
  std::size_t count = 0;
  double maximum_rms = 0, maximum_position = 0, maximum_rotation = 0;

  for (const double width : {0.135, 0.230})
    for (const double tilt : {0.0, 0.65, 1.2})
      for (const double roll : {0.0, 0.5, 1.1, 1.57, math::pi})
        for (const double perturbation : {-1e-6, 0.0, 1e-6})
          for (const bool distorted : {false, true})
            for (const bool noisy : {false, true}) {
              const double k1 = distorted ? 0.12 : 0, k2 = distorted ? -0.04 : 0;
              const double p1 = distorted ? 0.003 : 0, p2 = distorted ? -0.002 : 0;
              const double k3 = distorted ? 0.01 : 0;
              const vision::Calibration calibration(640, 480,
                  {1000, 0, 320, 0, 900, 240, 0, 0, 1}, {k1, k2, p1, p2, k3},
                  math::Transform<math::CameraFrame, math::GimbalFrame>(math::SE3(
                      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero())),
                  core::Evidence::declared(), core::Evidence::declared());
              const Eigen::Quaterniond rotation =
                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(math::pi + perturbation, Eigen::Vector3d::UnitX());
              const Eigen::Vector3d location(0.13, -0.08, 2.0);
              const std::array<Eigen::Vector3d, 4> points{
                  Eigen::Vector3d(-width / 2, 0.0275, 0), {width / 2, 0.0275, 0},
                  {width / 2, -0.0275, 0}, {-width / 2, -0.0275, 0}};
              vision::Detection input{{}, 1, vision::TeamColor::red, 3, true};

              // 独立五参数投影，不调用生产物体点函数或 OpenCV projectPoints。
              for (std::size_t k = 0; k < points.size(); ++k) {
                const Eigen::Vector3d camera = rotation * points[k] + location;
                const double x = camera.x() / camera.z(), y = camera.y() / camera.z();
                const double r2 = x * x + y * y;
                const double radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
                const double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
                const double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
                input.corners[k] = {
                    float(1000 * (x * radial + dx) + 320 + (noisy ? 0.01 * sin(k + 1) : 0)),
                    float(900 * (y * radial + dy) + 240 + (noisy ? 0.01 * cos(k + 1) : 0))};
              }

              const vision::PlateDimensions size(core::Metres(width), core::Metres(0.055));
              const auto candidates = vision::ippe_candidates(input, size, calibration);
              CHECK(candidates.size() == 2);
              const auto& best = candidates.front();
              const auto& pose = best.plate_to_camera.value();
              const double position_error = (pose.translation() - location).norm();
              const double rotation_error =
                  math::rotation_log(pose.rotation() * rotation.conjugate()).norm();
              CHECK(best.rms_px < (noisy ? 0.02 : 0.001));
              CHECK(position_error < (noisy ? 0.002 : 1e-4));
              CHECK(rotation_error < (noisy ? 0.02 : 1e-4));
              CHECK(best.view_cosine > 0);
              CHECK(math::rotation_log(pose.rotation() *
                  candidates.back().plate_to_camera.value().rotation().conjugate()).norm() >
                  1e-6);

              if (!noisy) {
                maximum_rms = std::max(maximum_rms, best.rms_px);
                maximum_position = std::max(maximum_position, position_error);
                maximum_rotation = std::max(maximum_rotation, rotation_error);
              }
              ++count;
            }

  std::cout << "Independent geometry cases=" << count << " noiseless max RMS=" << maximum_rms
            << " px position=" << maximum_position << " m rotation=" << maximum_rotation
            << " rad\n";
}
} // namespace

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto calibration = test::synthetic_calibration();
    const vision::PlateDimensions dimensions(core::Metres(0.135), core::Metres(0.055));
    const auto facing =
        Eigen::Quaterniond(Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitX()));

    const auto rotation = Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(0.9, Eigen::Vector3d::UnitY()) * facing;

    const math::SE3 truth(rotation, {0.2, 0.1, 1.5});
    auto detection = test::project_plate(truth, dimensions, calibration);
    const auto candidates = vision::ippe_candidates(detection, dimensions, calibration);
    CHECK(candidates.size() == 2);
    CHECK(candidates.front().rms_px < 0.001);
    CHECK((candidates.front().plate_to_camera.value().translation() - truth.translation()).norm() <
          1e-4);

    CHECK(math::rotation_log(candidates.front().plate_to_camera.value().rotation() *
                             truth.rotation().conjugate())
              .norm() < 1e-4);

    CHECK(candidates.front().view_cosine > 0);
    auto collapsed_edge = detection;
    collapsed_edge.corners[1] = collapsed_edge.corners[0];
    CHECK(vision::ippe_candidates(collapsed_edge, dimensions, calibration).empty());
    detection.corners[0].x = std::numeric_limits<float>::quiet_NaN();
    CHECK(vision::ippe_candidates(detection, dimensions, calibration).empty());
    CHECK_THROWS(std::invalid_argument, vision::PlateDimensions(core::Metres(0), core::Metres(1)));

    // 独立针孔算式与手写物理点，不调用被测 object_corners/project_plate 生成真值。
    const vision::Calibration pinhole(640, 480, {1000, 0, 320, 0, 900, 240, 0, 0, 1},
        {0, 0, 0, 0, 0},
        math::Transform<math::CameraFrame, math::GimbalFrame>(
            math::SE3(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero())),
        core::Evidence::declared(), core::Evidence::declared());
    for (const double width : {0.135, 0.230}) {
      const vision::PlateDimensions size(core::Metres(width), core::Metres(0.055));
      const std::array<Eigen::Vector3d, 4> physical{
          Eigen::Vector3d(-width / 2, 0.0275, 0), {width / 2, 0.0275, 0},
          {width / 2, -0.0275, 0}, {-width / 2, -0.0275, 0}};
      for (const double roll : {0.0, 1.1, math::pi}) {
        const Eigen::Quaterniond orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.65, Eigen::Vector3d::UnitY()) * facing;
        const Eigen::Vector3d location(0.13, -0.08, 2.0);
        vision::Detection reference{{}, 1, vision::TeamColor::red, 3, true};
        for (std::size_t k = 0; k < physical.size(); ++k) {
          const Eigen::Vector3d camera = orientation * physical[k] + location;
          reference.corners[k] = {float(1000 * camera.x() / camera.z() + 320),
                                  float(900 * camera.y() / camera.z() + 240)};
        }
        const auto actual = vision::ippe_candidates(reference, size, pinhole);
        if (actual.empty() || actual.front().rms_px >= 0.001) {
          std::cerr << "Independent PnP width=" << width << " roll=" << roll;
          for (const auto& candidate : actual)
            std::cerr << " rms=" << candidate.rms_px << " t=" <<
                candidate.plate_to_camera.value().translation().transpose() << " r=" <<
                math::rotation_log(candidate.plate_to_camera.value().rotation() *
                                   orientation.conjugate()).transpose();
          std::cerr << '\n';
        }
        CHECK(!actual.empty() && actual.front().rms_px < 0.001);
        CHECK((actual.front().plate_to_camera.value().translation() - location).norm() < 1e-4);
        CHECK(math::rotation_log(actual.front().plate_to_camera.value().rotation() *
                                 orientation.conjugate()).norm() < 1e-4);

        auto mirrored_raw = reference;
        mirrored_raw.corners = {reference.corners[1], reference.corners[0],
                                reference.corners[3], reference.corners[2]};
        const vision::CornerMapping mirror({1, 0, 3, 2}, "unknown-model", "unverified",
                                            core::Evidence::declared());
        const auto corrected = mirror.apply(mirrored_raw, core::ClockDomain::replay);
        CHECK(corrected.corners == reference.corners && !corrected.corners_reliable);

        // 同比放大尺寸会同比放大距离，重投影仍很小；低 RMS 不能证明实际物理尺寸。
        const auto enlarged = vision::ippe_candidates(reference,
            vision::PlateDimensions(core::Metres(2 * width), core::Metres(0.110)), pinhole);
        CHECK(!enlarged.empty() && enlarged.front().rms_px < 0.001);
        CHECK((enlarged.front().plate_to_camera.value().translation() - 2 * location).norm() <
              2e-4);
      }
    }
    check_perturbed_geometry();
  });
}
