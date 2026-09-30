#include "autoaim/vision/pnp.hpp"
#include "autoaim/math/angle.hpp"
#include "support/vision_fixture.hpp"
#include "test_support.hpp"
#include <limits>

namespace autoaim::vision {
// Defined in tests/support/pnp_baseline.cpp and linked only for this candidate test.
// The production library has one CMake-selected implementation.
std::vector<PoseCandidate> ippe_candidates_baseline(const Detection& detection,
                                                    const PlateDimensions& dimensions,
                                                    const Calibration& calibration);
} // namespace autoaim::vision

namespace {
autoaim::math::SE3 unchecked_lm(const autoaim::vision::PoseCandidate& seed,
                               const autoaim::vision::Detection& detection,
                               const autoaim::vision::PlateDimensions& dimensions,
                               const autoaim::vision::Calibration& calibration) {
  // Independent OpenCV refinement demonstrates that the merge fixture really merges;
  // merely returning the unmodified baseline cannot satisfy the improvement fixture.
  const auto edge = detection.corners[1] - detection.corners[0];
  const double theta = std::atan2(double(edge.y), double(edge.x));
  const double c = std::cos(theta), s = std::sin(theta);
  const cv::Matx33d equivalent(c, s, 0, s, -c, 0, 0, 0, -1);
  const auto& pose = seed.plate_to_camera.value();
  cv::Matx33d matrix;
  const auto rotation = pose.rotation().toRotationMatrix();
  for (int r = 0; r < 3; ++r)
    for (int col = 0; col < 3; ++col)
      matrix(r, col) = rotation(r, col);
  cv::Vec3d rvec, tvec(pose.translation().x(), pose.translation().y(), pose.translation().z());
  cv::Rodrigues(matrix * equivalent.t(), rvec);
  std::vector<cv::Point3d> objects;
  for (const auto& point : autoaim::vision::object_corners(dimensions)) {
    const auto transformed = equivalent * cv::Vec3d(point.x, point.y, point.z);
    objects.emplace_back(transformed[0], transformed[1], transformed[2]);
  }
  std::vector<cv::Point2d> pixels;
  for (const auto& point : detection.corners)
    pixels.emplace_back(point.x, point.y);
  cv::solvePnPRefineLM(objects, pixels, calibration.intrinsic(), calibration.distortion(),
                        rvec, tvec, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                                                       20, 1e-6));
  cv::Rodrigues(rvec, matrix);
  matrix = matrix * equivalent;
  Eigen::Matrix3d refined;
  for (int r = 0; r < 3; ++r)
    for (int col = 0; col < 3; ++col)
      refined(r, col) = matrix(r, col);
  return autoaim::math::SE3(Eigen::Quaterniond(refined), {tvec[0], tvec[1], tvec[2]});
}

void check_equal(const autoaim::vision::PoseCandidate& actual,
                 const autoaim::vision::PoseCandidate& expected) {
  const auto& a = actual.plate_to_camera.value();
  const auto& e = expected.plate_to_camera.value();
  CHECK((a.translation() - e.translation()).norm() < 1e-10);
  CHECK(autoaim::math::rotation_log(a.rotation() * e.rotation().conjugate()).norm() < 1e-10);
  CHECK_NEAR(actual.rms_px, expected.rms_px, 1e-10);
  CHECK_NEAR(actual.maximum_error_px, expected.maximum_error_px, 1e-10);
  CHECK_NEAR(actual.view_cosine, expected.view_cosine, 1e-10);
}
} // namespace

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto calibration = test::synthetic_calibration();
    const vision::PlateDimensions dimensions(core::Metres(0.135), core::Metres(0.055));

    // Fixed noisy projected plate. The best branch improves; refining the other raises
    // its maximum corner error and must retain that branch's original IPPE solution.
    const vision::Detection noisy{{cv::Point2f(916.511962890625f, 626.0585327148438f),
                                    {1064.4320068359375f, 649.2195434570312f},
                                    {1063.5577392578125f, 753.19921875f},
                                    {916.86376953125f, 713.1968383789062f}},
                                   1, vision::TeamColor::red, 3, true};
    const auto baseline = vision::ippe_candidates_baseline(noisy, dimensions, calibration);
    const auto refined = vision::ippe_candidates(noisy, dimensions, calibration);
    CHECK(baseline.size() == 2 && refined.size() == 2);
    CHECK(refined.front().rms_px + 0.001 < baseline.front().rms_px);
    CHECK(refined.front().maximum_error_px < baseline.front().maximum_error_px);
    check_equal(refined.back(), baseline.back());
    CHECK(math::rotation_log(refined.front().plate_to_camera.value().rotation() *
                               refined.back().plate_to_camera.value().rotation().conjugate())
              .norm() > 0.1);
    for (const auto& candidate : refined) {
      CHECK(std::isfinite(candidate.rms_px) && std::isfinite(candidate.view_cosine));
      CHECK(!candidate.covariance && !candidate.geometry_accepted);
      for (const auto& point : vision::object_corners(dimensions))
        CHECK(candidate.plate_to_camera.value().apply({point.x, point.y, point.z}).z() > 0);
    }

    // Both LM initializations approach the same minimum. The whole original pair must
    // survive; retaining only the best/refined pose would erase the planar ambiguity.
    const vision::Detection merging{{cv::Point2f(1003.7588500976562f, 562.8726806640625f),
                                      {1057.879150390625f, 793.4990234375f},
                                      {962.7047729492188f, 809.7005615234375f},
                                      {912.2699584960938f, 578.5528564453125f}},
                                     1, vision::TeamColor::red, 3, true};
    const auto original_pair = vision::ippe_candidates_baseline(merging, dimensions, calibration);
    CHECK(original_pair.size() == 2);
    const auto lm_first = unchecked_lm(original_pair[0], merging, dimensions, calibration);
    const auto lm_second = unchecked_lm(original_pair[1], merging, dimensions, calibration);
    CHECK((lm_first.translation() - lm_second.translation()).norm() < 1e-5);
    CHECK(math::rotation_log(lm_first.rotation() * lm_second.rotation().conjugate()).norm() <
          1e-4);
    CHECK(math::rotation_log(original_pair[0].plate_to_camera.value().rotation() *
                               original_pair[1].plate_to_camera.value().rotation().conjugate())
              .norm() > 0.01);
    const auto preserved = vision::ippe_candidates(merging, dimensions, calibration);
    CHECK(preserved.size() == original_pair.size());
    check_equal(preserved[0], original_pair[0]);
    check_equal(preserved[1], original_pair[1]);

    auto invalid = noisy;
    invalid.corners[2].x = std::numeric_limits<float>::quiet_NaN();
    CHECK(vision::ippe_candidates(invalid, dimensions, calibration).empty());
    invalid = noisy;
    invalid.corners[3].y = std::numeric_limits<float>::infinity();
    CHECK(vision::ippe_candidates(invalid, dimensions, calibration).empty());
    invalid = noisy;
    invalid.corners[1] = invalid.corners[0];
    CHECK(vision::ippe_candidates(invalid, dimensions, calibration).empty());
    invalid.corners = {cv::Point2f(600, 500), {610, 500}, {620, 500}, {630, 500}};
    // Image-contour validity belongs to solve_pose. With distortion, the candidate
    // factory alone can still return algebraic poses for collinear image pixels.
    const core::Stamp source(1, 1, core::TimePoint(20000000, core::ClockDomain::replay));
    const vision::PnpQualityOptions options{
        3, 5, 2, 0.1, 0.01, 0.5, 1e-10, core::Metres(0.1), core::Radians(0.1), core::Seconds(0.1)};
    const auto rejected = vision::solve_pose(source, invalid, dimensions, calibration, options,
                                             "synthetic-camera", "v1", std::nullopt, std::nullopt);
    CHECK(!rejected.pose_valid && !rejected.pose_reliable && rejected.candidates.empty());
    std::cout << "I2-LM best RMS " << baseline.front().rms_px << " -> "
              << refined.front().rms_px << "; merged branches retained\n";
  });
}
