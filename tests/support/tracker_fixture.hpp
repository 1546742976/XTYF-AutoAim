#pragma once

#include "autoaim/estimation/tracker.hpp"

namespace test {
inline autoaim::estimation::TrackerOptions tracker_options() {
  using namespace autoaim;

  return {32,
          estimation::StateCovariance::Identity() * 0.1,
          {3, 0.05, 50, 0.5},
          {2, 0.1, 50, 0.5},
          {3, core::Seconds(0.04), core::Seconds(0.06), core::Seconds(0.3), 0.5, 1},
          {4, 100, 100},
          {0.1, 10, 1, 3, core::Seconds(0.2), core::Seconds(0.1)},
          estimation::NisGate(estimation::chi_square_95_limits()),
          core::Seconds(0.1),
          0.01,
          0.001,
          0.1,
          "v1"};
}

// 直接生成带独立真值的三维观测，用于估计器测试；不冒充相机可同时看见所有板。
inline std::shared_ptr<const autoaim::estimation::Observation>
observation(const autoaim::estimation::GeometryProfile& profile, const autoaim::core::Stamp& source,
            const autoaim::math::Point3<autoaim::math::WorldFrame>& center,
            autoaim::core::Radians phase, std::size_t board, double variance = 1e-5) {
  using namespace autoaim;
  const auto world = profile.plate_pose(center, phase, board);
  vision::PnpEstimate pnp(source, profile.plates.at(board).dimensions);
  pnp.candidates.push_back({vision::PlateToCamera(world.value()), 0, 0, 1,
                            vision::PoseCovariance::Identity() * variance, true});

  pnp.selected = 0;
  pnp.pose_valid = true;
  pnp.pose_reliable = true;
  const vision::AlignedPose aligned(source.exposure, source.exposure, source.exposure,
                                    math::Transform<math::GimbalFrame, math::WorldFrame>(math::SE3(
                                        Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero())),
                                    true);

  const auto evidence = core::Evidence::from_report(
      core::MeasurementReport::evaluate({"synthetic-camera", "v1", "2026-09-29", "synthetic clock"},
                                        {0, 0}, 0, core::ClockDomain::replay));

  return std::make_shared<const estimation::Observation>(
      source, vision::Detection{{}, 1, vision::TeamColor::red, 3, true}, pnp,
      std::vector<estimation::ObservedPose>{{world, vision::PoseCovariance::Identity() * variance}},
      aligned, core::TimeOrigin::synthetic, core::Seconds(0), evidence);
}
} // namespace test
