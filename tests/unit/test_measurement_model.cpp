#include "autoaim/estimation/measurement_model.hpp"
#include "support/geometry_fixture.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto profile = test::geometry(4, estimation::HeightLayout::distinct, true);
    const estimation::MeasurementModel full(estimation::MeasurementKind::full_pose);
    const estimation::MeasurementModel position(estimation::MeasurementKind::position);
    const estimation::TargetState state{
        math::Point3<math::WorldFrame>({3, 0, 1}), {0, 0, 0}, core::Radians(math::pi - 0.01), 1, 0};

    auto truth = state;
    truth.phase = core::Radians(-math::pi + 0.01);
    const estimation::ObservedPose observed{full.predict(truth, profile, 2),
                                            vision::PoseCovariance::Identity() * 0.01};

    const auto linearized = full.linearize(state, profile, 2, observed);
    CHECK(linearized && linearized.value().residual.size() == 6);
    CHECK_NEAR(linearized.value().residual.tail<3>().norm(), 0.02, 1e-10);
    CHECK(linearized.value().jacobian.leftCols<3>().isApprox(
        (Eigen::Matrix<double, 6, 3>() << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero())
            .finished(),
        1e-8));

    constexpr int phase = estimation::component_index(estimation::StateComponent::phase);
    const Eigen::Vector3d axis = profile.axis_to_world * Eigen::Vector3d::UnitZ();
    CHECK((linearized.value().jacobian.col(phase).tail<3>() - axis).norm() < 1e-7);
    const auto position_only = position.linearize(state, profile, 2, observed);
    CHECK(position_only.value().residual.size() == 3 && position_only.value().noise.rows() == 3);
    CHECK(!full.linearize(state, profile, 4, observed));
    CHECK(!full.linearize(state, profile, 2, {observed.plate_to_world, std::nullopt}));
  });
}
