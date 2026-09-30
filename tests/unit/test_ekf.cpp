#include "autoaim/estimation/ekf.hpp"
#include "autoaim/estimation/state_estimator.hpp"
#include "autoaim/math/numeric.hpp"
#include "support/tracker_fixture.hpp"
#include "test_support.hpp"
#include <type_traits>

namespace {
void check_compile_time_construction() {
  using namespace autoaim;
  using namespace autoaim::estimation;
  static_assert(std::is_aggregate_v<TrackerOptions>);
  // 此既有 fixture 仍使用追加 eso 字段之前的十三项聚合初始化。
  const auto legacy_options = test::tracker_options();
  CHECK(legacy_options.eso.translation_bandwidth_radps == 10);
  CHECK(legacy_options.eso.angular_bandwidth_radps == 10);
  CHECK(legacy_options.eso.linear_jerk_psd == 1);

  const TargetState initial{math::Point3<math::WorldFrame>({1, 2, 3}), {0.1, 0.2, -0.3},
                            core::Radians(0.2), 0.3, 0.4, {0.5, -0.2, 0.1}};
  const auto motion = std::make_shared<const MotionModel>(0.01, 0.02, core::Seconds(1));
  for (const EsoOptions options : {EsoOptions{}, EsoOptions{3, 5, 0}, EsoOptions{17, 23, 4}}) {
    Ekf direct(initial, StateCovariance::Identity(), motion);
    auto constructed = make_state_estimator<Ekf>(initial, StateCovariance::Identity(), motion, options);
    CHECK(constructed.motion() == direct.motion());
    CHECK(state_difference(constructed.state(), direct.state()).isZero(0));
    CHECK((constructed.covariance() - direct.covariance()).isZero(0));
    for (double dt : {0.01, 0.04, 0.02}) {
      CHECK(direct.predict(core::Seconds(dt)));
      CHECK(constructed.predict(core::Seconds(dt)));
      LinearizedMeasurement measurement{Eigen::VectorXd::Constant(1, 0.1),
                                         Eigen::MatrixXd::Zero(1, state_dimension),
                                         Eigen::MatrixXd::Identity(1, 1) * 0.03};
      measurement.jacobian(0, component_index(StateComponent::x)) = 1;
      const auto expected = direct.update(measurement, 100);
      const auto actual = constructed.update(measurement, 100);
      CHECK(expected && actual && expected.value().accepted && actual.value().accepted);
      CHECK(expected.value().prior_nis == actual.value().prior_nis);
      CHECK(state_difference(constructed.state(), direct.state()).isZero(0));
      CHECK((constructed.covariance() - direct.covariance()).isZero(0));
      CHECK(constructed.motion() == direct.motion());
    }
  }
}
} // namespace

int main() {
  using namespace autoaim;

  return test::run([] {
    check_compile_time_construction();
    auto model = std::make_shared<const estimation::MotionModel>(0.01, 0.01, core::Seconds(1));
    estimation::Ekf filter(
        {math::Point3<math::WorldFrame>({0, 0, 0}), {0, 0, 0}, core::Radians(0), 0, 0},
        estimation::StateCovariance::Identity(), model);
    constexpr int acceleration = estimation::component_index(estimation::StateComponent::ax);
    CHECK(filter.state().acceleration_mps2.norm() == 0);
    CHECK(filter.covariance().middleRows<3>(acceleration).norm() == 0);
    CHECK(filter.covariance().middleCols<3>(acceleration).norm() == 0);

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
      CHECK(filter.state().acceleration_mps2.norm() == 0);
      CHECK(filter.covariance().middleRows<3>(acceleration).norm() == 0);
      CHECK(filter.covariance().middleCols<3>(acceleration).norm() == 0);
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
