#include "autoaim/estimation/health.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const estimation::NisGate gate(estimation::chi_square_95_limits());
    CHECK_NEAR(gate.limit(4), 9.488, 0.001);
    CHECK(gate.limit(3) != gate.limit(6));
    CHECK_THROWS(std::invalid_argument, gate.limit(0));
    const estimation::TargetState state{
        math::Point3<math::WorldFrame>({0, 0, 0}), {0, 0, 0}, core::Radians(0), 0, 0};

    estimation::HealthMonitor health({3, 10, 10});
    CHECK(!health.observe(state, estimation::StateCovariance::Identity(), false).reset_required);
    CHECK(!health.observe(state, estimation::StateCovariance::Identity(), false).reset_required);
    CHECK(health.observe(state, estimation::StateCovariance::Identity(), false).reset_required);
    health.reset();
    CHECK(health.observe(state, estimation::StateCovariance::Identity(), true)
              .consecutive_rejections == 0);

    CHECK(health.observe(state, estimation::StateCovariance::Identity() * 20, true).reset_required);
    auto bad = state;
    bad.omega_radps = std::numeric_limits<double>::quiet_NaN();
    const auto report = health.observe(bad, estimation::StateCovariance::Identity(), true);
    CHECK(!report.numerical_validity && report.reset_required);
  });
}
