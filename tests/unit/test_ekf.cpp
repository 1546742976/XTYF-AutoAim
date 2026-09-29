#include "autoaim/estimation/ekf.hpp"
#include "autoaim/math/numeric.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    auto model = std::make_shared<const estimation::MotionModel>(0.01, 0.01, core::Seconds(1));
    estimation::Ekf filter(
        {math::Point3<math::WorldFrame>({0, 0, 0}), {0, 0, 0}, core::Radians(0), 0, 0},
        estimation::StateCovariance::Identity(), model);

    estimation::LinearizedMeasurement measurement{
        Eigen::VectorXd::Constant(1, 10), Eigen::MatrixXd::Zero(1, estimation::state_dimension),
        Eigen::MatrixXd::Identity(1, 1) * 0.1};

    measurement.jacobian(0, estimation::component_index(estimation::StateComponent::x)) = 1;
    const auto rejected = filter.update(measurement, 4);
    CHECK(rejected && !rejected.value().accepted);
    CHECK_NEAR(rejected.value().prior_nis, 100 / 1.1, 1e-10);
    CHECK(filter.state().center.metres().x() == 0);
    const auto accepted = filter.update(measurement, 100);
    CHECK(accepted && accepted.value().accepted);
    CHECK_NEAR(accepted.value().prior_nis, rejected.value().prior_nis, 1e-10);
    CHECK_NEAR(filter.state().center.metres().x(), 10 / 1.1, 1e-10);

    for (int i = 0; i < 100; ++i) {
      CHECK(filter.predict(core::Seconds(0.01)));
      measurement.residual[0] = 10 - filter.state().center.metres().x();
      CHECK(filter.update(measurement, 100).value().accepted);
      CHECK(math::covariance_valid(filter.covariance()));
    }

    const auto previous = filter.state();
    measurement.noise(0, 0) = -1;
    CHECK(!filter.update(measurement, 100));
    CHECK(estimation::state_difference(filter.state(), previous).norm() == 0);
    CHECK(filter.change_motion(
        std::make_shared<const estimation::MotionModel>(0.01, 0.01, core::Seconds(1), 5), 0.2));

    CHECK(filter.motion()->kind() == estimation::MotionKind::bounded_acceleration);
  });
}
