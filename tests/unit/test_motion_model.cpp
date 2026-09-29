#include "autoaim/estimation/motion_model.hpp"
#include "autoaim/math/angle.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    const estimation::MotionModel model(0.1, 0.2, core::Seconds(1));
    const estimation::TargetState state{math::Point3<math::WorldFrame>({1, 2, 3}), {1, 0, 0},
      core::Radians(math::pi - 0.1), 2, 7};
    const auto prediction = model.propagate(state, estimation::StateCovariance::Identity(), core::Seconds(0.1));
    CHECK(prediction);
    CHECK_NEAR(prediction.value().state.center.metres().x(), 1.1, 1e-12);
    CHECK_NEAR(prediction.value().state.phase.value(), -math::pi + 0.1, 1e-12);
    CHECK(prediction.value().state.alpha_radps2 == 0);
    CHECK_NEAR(prediction.value().covariance(estimation::component_index(estimation::StateComponent::alpha),
      estimation::component_index(estimation::StateComponent::alpha)), 0, 0);
    const auto second = model.propagate(prediction.value().state, prediction.value().covariance, core::Seconds(0.2));
    const auto direct = model.propagate(state, estimation::StateCovariance::Identity(), core::Seconds(0.3));
    CHECK(estimation::state_difference(second.value().state, direct.value().state).norm() < 1e-12);
    CHECK((second.value().covariance - direct.value().covariance).norm() < 1e-12);
    CHECK(!model.propagate(state, estimation::StateCovariance::Identity(), core::Seconds(-0.1)));
    CHECK(!model.propagate(state, estimation::StateCovariance::Identity(), core::Seconds(1.1)));
  });
}
