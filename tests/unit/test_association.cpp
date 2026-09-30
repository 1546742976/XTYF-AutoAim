#include "autoaim/estimation/association.hpp"
#include "support/geometry_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    Eigen::MatrixXd costs(2, 2);
    costs << 8, 1, 2, 9;
    const auto assignments = estimation::mutual_nearest_assignment(costs, 10, 0.5);
    CHECK(assignments.size() == 2 && assignments[0].observation == 1 &&
          assignments[1].observation == 0);

    costs << 1, 1.1, 1.1, 1;
    CHECK(estimation::mutual_nearest_assignment(costs, 10, 0.5).empty());
    costs << 1, 9, 1, 9;
    CHECK(estimation::mutual_nearest_assignment(costs, 10, 0.5).empty());
    costs.setConstant(std::numeric_limits<double>::infinity());
    CHECK(estimation::mutual_nearest_assignment(costs, 10, 0.5).empty());
    costs(0, 0) = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS(std::invalid_argument, estimation::mutual_nearest_assignment(costs, 10, 0.5));
    const estimation::MeasurementModel measurement(estimation::MeasurementKind::full_pose);
    const estimation::NisGate gate(estimation::chi_square_95_limits());

    for (std::size_t count : {2, 3, 4}) {
      const auto geometry = test::geometry(count, estimation::HeightLayout::distinct, true);
      const estimation::TargetState state{
          math::Point3<math::WorldFrame>({3, 0, 1}), {0, 0, 0}, core::Radians(0.2), 0, 0};

      estimation::StateEstimator filter(
          state, estimation::StateCovariance::Identity() * 0.001,
          std::make_shared<const estimation::MotionModel>(0.1, 0.1, core::Seconds(1)));

      const core::TimePoint time(1, core::ClockDomain::replay);
      const core::Stamp source(1, 1, time);
      vision::PnpEstimate pnp(source, geometry.plates.back().dimensions);
      pnp.candidates.push_back(
          {vision::PlateToCamera(math::SE3(Eigen::Quaterniond::Identity(), {0, 0, 1})), 0, 0, 1,
           vision::PoseCovariance::Identity() * 0.001, true});

      pnp.selected = 0;
      pnp.pose_valid = true;
      const vision::AlignedPose aligned(
          time, time, time,
          math::Transform<math::GimbalFrame, math::WorldFrame>(
              math::SE3(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero())),
          true);

      const estimation::Observation observation(
          source, {{}, 1, vision::TeamColor::red, 7, true}, pnp,
          {{geometry.plate_pose(state.center, state.phase, count - 1),
            vision::PoseCovariance::Identity() * 0.001}},
          aligned, core::TimeOrigin::synthetic, core::Seconds(0), core::Evidence::missing());

      const auto candidates = estimation::association_candidates(filter, time, geometry,
                                                                 observation, measurement, gate);

      CHECK(!candidates.empty() && candidates.front().physical_plate == count - 1);
      CHECK(estimation::association_candidates(filter, core::advance(time, core::Seconds(0.1)),
                                               geometry, observation, measurement, gate)
                .empty());
    }
  });
}
