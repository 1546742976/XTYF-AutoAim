#include "autoaim/decision/hit_probability.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const core::Stamp source(1, 1, core::TimePoint(1, core::ClockDomain::replay));
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(-1.5707963267948966, Eigen::Vector3d::UnitY()));
    const decision::PredictedPlate plate{2,
      math::Transform<math::PlateFrame, math::WorldFrame>(math::SE3(rotation, {3, 0, 0})),
      vision::PoseCovariance::Identity() * 1e-8, vision::PlateDimensions(core::Metres(0.135), core::Metres(0.055)),
      {0, 0.1, 0}, true};
    const decision::BallisticModel model({0, 0, -9.81}, core::Seconds(1), core::Metres(0.01));
    const math::Point3<math::WorldFrame> origin({0, 0, 0});
    const auto trajectory = model.solve(origin, math::Point3<math::WorldFrame>({3, 0, 0}), core::MetresPerSecond(20)).value();
    const decision::InterceptSolution solution(source, source.exposure, source.exposure,
      core::advance(source.exposure, trajectory.flight_time), trajectory, plate, 1);
    decision::ImpactUncertainty noise{0.01, Eigen::Matrix3d::Identity() * 1e-8,
      Eigen::Matrix3d::Identity() * 1e-8, 0.001, Eigen::Matrix3d::Identity() * 1e-8};
    const decision::MarginOptions options{3, core::Metres(0.001), 0.1};
    const auto small = decision::evaluate_impact_margin(solution, origin, model, noise, options);
    CHECK(small && small.value().inside);
    CHECK(small.value().mean_error_plate_m.norm() < 1e-10);
    noise.aim_rotation_covariance_rad2 = Eigen::Matrix3d::Identity() * 0.01;
    const auto large = decision::evaluate_impact_margin(solution, origin, model, noise, options);
    CHECK(large && !large.value().inside);
    CHECK(large.value().covariance_plate_m2.trace() > small.value().covariance_plate_m2.trace());
    noise.speed_sigma_mps = std::numeric_limits<double>::quiet_NaN();
    CHECK(!decision::evaluate_impact_margin(solution, origin, model, noise, options));
  });
}
