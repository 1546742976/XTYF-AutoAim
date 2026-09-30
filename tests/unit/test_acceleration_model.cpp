#include "autoaim/estimation/motion_model.hpp"
#include "autoaim/math/numeric.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const estimation::MotionModel cv(0.1, 0.2, core::Seconds(1));
    const estimation::MotionModel ca(0.1, 0.2, core::Seconds(1), 4);
    const estimation::TargetState state{
        math::Point3<math::WorldFrame>({1, 2, 3}), {0, 0, 0}, core::Radians(0), 1, 2};

    const auto predicted =
        ca.propagate(state, estimation::StateCovariance::Identity(), core::Seconds(0.2));

    CHECK(predicted);
    CHECK_NEAR(predicted.value().state.phase.value(), 0.24, 1e-12);
    CHECK_NEAR(predicted.value().state.omega_radps, 1.4, 1e-12);
    CHECK(math::covariance_valid(predicted.value().covariance));
    auto extreme = state;
    extreme.alpha_radps2 = 50;
    CHECK(ca.propagate(extreme, estimation::StateCovariance::Identity(), core::Seconds(0.2))
              .value()
              .state.alpha_radps2 == 4);

    const auto longer =
        ca.propagate(state, estimation::StateCovariance::Identity(), core::Seconds(0.5));

    constexpr int phase = estimation::component_index(estimation::StateComponent::phase);
    constexpr int alpha = estimation::component_index(estimation::StateComponent::alpha);
    CHECK(longer.value().covariance(phase, phase) > predicted.value().covariance(phase, phase));
    const auto activate = ca.remap_from(cv, state, estimation::StateCovariance::Identity(), 0.3);
    CHECK(activate.value().state.alpha_radps2 == 0);
    CHECK_NEAR(activate.value().covariance(alpha, alpha), 0.3, 0);
    const auto deactivate = cv.remap_from(ca, state, activate.value().covariance, 0.3);
    CHECK(deactivate.value().state.alpha_radps2 == 0 &&
          deactivate.value().covariance.row(alpha).norm() == 0);

    CHECK_NEAR(deactivate.value().covariance(phase, phase), 1, 0);
    auto factor = estimation::StateCovariance::Identity().eval();
    factor(phase, alpha) = 0.3;
    factor(alpha, 0) = -0.2;
    const auto correlated = (factor * factor.transpose()).eval();
    CHECK(correlated.row(alpha).head(alpha).norm() > 0);
    const auto to_cv = cv.remap_from(ca, state, correlated, 0.3);
    CHECK(to_cv && math::covariance_valid(to_cv.value().covariance));
    CHECK(to_cv.value().covariance.row(alpha).norm() == 0);
    CHECK(to_cv.value().covariance.col(alpha).norm() == 0);
    CHECK(to_cv.value().covariance.topLeftCorner(alpha, alpha).isApprox(
        correlated.topLeftCorner(alpha, alpha)));
    const auto to_ca = ca.remap_from(cv, to_cv.value().state, to_cv.value().covariance, 0.3);
    CHECK(to_ca && math::covariance_valid(to_ca.value().covariance));
    CHECK_NEAR(to_ca.value().covariance(alpha, alpha), 0.3, 0);
    CHECK(to_ca.value().covariance.row(alpha).head(alpha).norm() == 0);
    CHECK(to_ca.value().covariance.col(alpha).head(alpha).norm() == 0);

    constexpr int acceleration = estimation::component_index(estimation::StateComponent::ax);
    const auto linear = ca.with_linear_acceleration(0.4);
    CHECK(linear.linear_acceleration_enabled() && !ca.linear_acceleration_enabled());
    CHECK(linear.kind() == ca.kind());
    auto accelerating = state;
    accelerating.velocity_mps = {1, -2, 0.5};
    accelerating.acceleration_mps2 = {2, 1, -0.5};
    factor(acceleration, 0) = 0.2;
    factor(acceleration + 1, phase) = -0.1;
    const auto joint = (factor * factor.transpose()).eval();
    const auto translated = linear.propagate(accelerating, joint, core::Seconds(0.2));
    CHECK(translated);
    CHECK((translated.value().state.center.metres() -
           (accelerating.center.metres() + accelerating.velocity_mps * 0.2 +
            accelerating.acceleration_mps2 * 0.02)).norm() < 1e-12);
    CHECK((translated.value().state.velocity_mps -
           (accelerating.velocity_mps + accelerating.acceleration_mps2 * 0.2)).norm() < 1e-12);
    CHECK((translated.value().state.acceleration_mps2 - accelerating.acceleration_mps2).norm() == 0);

    // 独立构造 F 和连续白 jerk 积分 Q，包含位置/速度/加速度交叉项。
    auto expected_transition = estimation::StateCovariance::Identity().eval();
    auto expected_noise = estimation::StateCovariance::Zero().eval();
    Eigen::Matrix3d integrated;
    integrated << 0.000016, 0.0002, 0.0013333333333333333,
        0.0002, 0.0026666666666666667, 0.02,
        0.0013333333333333333, 0.02, 0.2;
    for (int axis = 0; axis < 3; ++axis) {
      const int indices[] = {axis, axis + 3, acceleration + axis};
      expected_transition(axis, axis + 3) = 0.2;
      expected_transition(axis, acceleration + axis) = 0.02;
      expected_transition(axis + 3, acceleration + axis) = 0.2;
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
          expected_noise(indices[row], indices[col]) = 0.4 * integrated(row, col);
    }
    expected_transition(phase, phase + 1) = 0.2;
    expected_transition(phase, alpha) = 0.02;
    expected_transition(phase + 1, alpha) = 0.2;
    expected_noise.block<3, 3>(phase, phase) = 0.2 * integrated;
    CHECK(translated.value().covariance.isApprox(
        expected_transition * joint * expected_transition.transpose() + expected_noise, 1e-12));
    const auto twice = linear.propagate(translated.value().state, translated.value().covariance,
                                        core::Seconds(0.3));
    const auto once = linear.propagate(accelerating, joint, core::Seconds(0.5));
    CHECK(twice && once);
    CHECK(estimation::state_difference(twice.value().state, once.value().state).norm() < 1e-12);
    CHECK(twice.value().covariance.isApprox(once.value().covariance, 1e-12));

    const auto inactive = ca.propagate(accelerating, joint, core::Seconds(0));
    CHECK(inactive && inactive.value().state.acceleration_mps2.norm() == 0);
    CHECK(inactive.value().covariance.middleRows<3>(acceleration).norm() == 0);
    CHECK(inactive.value().covariance.middleCols<3>(acceleration).norm() == 0);
    const auto mapped_off = ca.remap_from(linear, accelerating, joint, 0.3);
    CHECK(mapped_off && mapped_off.value().state.acceleration_mps2.norm() == 0);
    CHECK(mapped_off.value().covariance.middleRows<3>(acceleration).norm() == 0);
    CHECK(mapped_off.value().covariance.middleCols<3>(acceleration).norm() == 0);
    const auto mapped_on = linear.remap_from(ca, inactive.value().state,
                                             inactive.value().covariance, 0.3, 0.7);
    CHECK(mapped_on && mapped_on.value().state.acceleration_mps2.norm() == 0);
    CHECK_NEAR(mapped_on.value().covariance(acceleration, acceleration), 0.7, 0);
    CHECK(mapped_on.value().covariance.block<3, 9>(acceleration, 0).norm() == 0);
    const auto angular_only = cv.with_linear_acceleration(0.4).remap_from(
        linear, accelerating, joint, 0.3);
    CHECK(angular_only && angular_only.value().state.alpha_radps2 == 0);
    CHECK((angular_only.value().state.acceleration_mps2 - accelerating.acceleration_mps2).norm() == 0);
    CHECK(angular_only.value().covariance.bottomRightCorner<3, 3>().isApprox(
        joint.bottomRightCorner<3, 3>()));
    CHECK(!linear.propagate(accelerating, joint, core::Seconds(1.1)));
    CHECK_THROWS(std::invalid_argument, ca.with_linear_acceleration(-1));
  });
}
