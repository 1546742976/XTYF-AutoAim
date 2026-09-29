#include "autoaim/vision/pnp.hpp"
#include "autoaim/math/numeric.hpp"
#include "support/vision_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const auto calibration = test::synthetic_calibration();
    const vision::PlateDimensions size(core::Metres(0.135), core::Metres(0.055));
    const auto face = Eigen::Quaterniond(Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitX()));
    const math::SE3 pose(Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitY()) * face, {0.2, 0.1, 1.5});
    const auto detection = test::project_plate(pose, size, calibration);
    const core::Stamp source(2, 1, core::TimePoint(20000000, core::ClockDomain::replay));
    vision::PnpQualityOptions options{3, 5, 2, 0.1, 0.01, 0.5, 1e-10,
      core::Metres(0.1), core::Radians(0.1), core::Seconds(0.1)};
    const auto solve = [&](const vision::Detection& input, const vision::PnpQualityOptions& limits,
                           std::optional<vision::PlateIdentity> id = std::nullopt,
                           std::optional<vision::SamePlatePrior> prior = std::nullopt) {
      return vision::solve_pose(source, input, size, calibration, limits, "synthetic-camera", "v1", id, prior);
    };
    const auto result = solve(detection, options);
    CHECK(result.pose_valid && result.pose_reliable && !result.ambiguous);
    CHECK(math::covariance_valid(*result.candidates[*result.selected].covariance));
    auto untrusted = detection; untrusted.corners_reliable = false;
    CHECK(solve(untrusted, options).pose_valid && !solve(untrusted, options).pose_reliable);
    auto crossed = detection; std::swap(crossed.corners[1], crossed.corners[2]);
    CHECK(!solve(crossed, options).pose_valid);
    auto occluded = detection; occluded.corners[1] = occluded.corners[0];
    CHECK(!solve(occluded, options).pose_valid);
    auto ambiguous_options = options; ambiguous_options.ambiguity_rms_gap_px = 10;
    CHECK(solve(detection, ambiguous_options).ambiguous);
    const vision::PlateIdentity id{5, 2, "layout"};
    const vision::SamePlatePrior prior{id, core::Stamp(1, 1, core::TimePoint(10000000, core::ClockDomain::replay)),
      source.exposure, vision::PlateToCamera(pose)};
    CHECK(solve(detection, ambiguous_options, id, prior).pose_reliable);
    CHECK(!solve(detection, ambiguous_options, vision::PlateIdentity{5, 1, "layout"}, prior).pose_reliable);
    const vision::SamePlatePrior future{id, core::Stamp(1, 1, core::TimePoint(30000000, core::ClockDomain::replay)),
      source.exposure, vision::PlateToCamera(pose)};
    CHECK(!solve(detection, ambiguous_options, id, future).pose_reliable);
    auto ill_conditioned = options; ill_conditioned.minimum_information_ratio = 0.5;
    CHECK(solve(detection, ill_conditioned).pose_valid && !solve(detection, ill_conditioned).pose_reliable);
  });
}
