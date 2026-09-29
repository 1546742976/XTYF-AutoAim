#include "autoaim/math/angle.hpp"
#include "test_support.hpp"
#include <limits>

int main() {
  using namespace autoaim;
  return test::run([] {
    CHECK_NEAR(math::wrap_to_pi(core::Radians(math::pi)).value(), -math::pi, 1e-12);
    CHECK_NEAR(math::wrap_to_pi(core::Radians(-math::pi)).value(), -math::pi, 1e-12);
    CHECK_NEAR(math::angle_residual(core::Radians(-math::pi + 0.01),
                                   core::Radians(math::pi - 0.01)).value(), 0.02, 1e-12);
    const auto huge = std::numeric_limits<double>::max();
    CHECK(std::isfinite(math::angle_residual(core::Radians(huge), core::Radians(-huge)).value()));
  });
}
