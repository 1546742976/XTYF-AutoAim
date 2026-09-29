#include "autoaim/estimation/measurement_model.hpp"
#include "autoaim/math/numeric.hpp"
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

    const auto nominal = full.predict(state, profile, 1).value();
    const math::SE3 displaced(math::rotation_exp({0.2, -0.15, 0.1}) * nominal.rotation(),
                              nominal.translation() + Eigen::Vector3d(0.07, -0.02, 0.04));
    const estimation::ObservedPose nonzero{
        math::Transform<math::PlateFrame, math::WorldFrame>(displaced),
        vision::PoseCovariance::Identity() * 0.01};
    const auto actual = full.linearize(state, profile, 1, nonzero);
    CHECK(actual && actual.value().residual.norm() > 0.2);
    const auto residual = [&](const estimation::TargetState& candidate) {
      const auto prediction = full.predict(candidate, profile, 1).value();
      Eigen::Matrix<double, 6, 1> result;
      result.head<3>() = displaced.translation() - prediction.translation();
      result.tail<3>() = math::rotation_log(
          displaced.rotation() * prediction.rotation().conjugate());

      return result;
    };
    for (const double step : {1e-4, 3e-6})
      for (int column = 0; column < estimation::state_dimension; ++column) {
        estimation::StateVector delta = estimation::StateVector::Zero();
        delta[column] = step;
        const auto derivative = ((residual(estimation::add_state_delta(state, delta)) -
            residual(estimation::add_state_delta(state, -delta))) / (2 * step)).eval();
        CHECK((actual.value().jacobian.col(column) + derivative).norm() < 1e-7);
      }

    // 固定线性化下的公共 6D 误差，包含位置/旋转交叉相关；不证明非线性 EKF 一致性。
    Eigen::Matrix<double, 6, 6> factor = Eigen::Matrix<double, 6, 6>::Identity();
    factor(0, 3) = 0.4;
    factor(1, 5) = -0.3;
    const auto common = (factor * factor.transpose() * 0.01).eval();
    Eigen::MatrixXd stacked = Eigen::MatrixXd::Zero(18, 6);
    Eigen::MatrixXd inflated = Eigen::MatrixXd::Zero(18, 18);
    for (int board = 0; board < 3; ++board) {
      Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Identity();
      transform(0, 4) = 0.2 * board;
      transform(2, 3) = -0.1 * board;
      stacked.middleRows<6>(6 * board) = transform;
      inflated.block<6, 6>(6 * board, 6 * board) =
          3 * (transform * common * transform.transpose() +
               vision::PoseCovariance::Identity() * 0.001);
    }
    const Eigen::MatrixXd joint = stacked * common * stacked.transpose() +
                                 Eigen::MatrixXd::Identity(18, 18) * 0.001;
    CHECK(math::covariance_valid(joint));
    CHECK(math::covariance_valid(inflated - joint, 1e-10));
  });
}
