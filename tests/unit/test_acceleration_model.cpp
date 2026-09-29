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
  });
}
