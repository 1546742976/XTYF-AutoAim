#include "autoaim/estimation/health.hpp"
#include "autoaim/math/numeric.hpp"

namespace autoaim::estimation {
std::array<double, 6> chi_square_95_limits() {
  // NIST 上侧表，P(chi-square <= limit)=0.95，按真实残差维数选择。
  // https://www.itl.nist.gov/div898/handbook/eda/section3/eda3674.htm
  return {3.841, 5.991, 7.815, 9.488, 11.070, 12.592};
}

NisGate::NisGate(std::array<double, 6> limits) : limits_(limits) {
  for (const double limit : limits_)
    if (!std::isfinite(limit) || limit <= 0)
      throw std::invalid_argument("Invalid dimension-specific NIS gate");
}

double NisGate::limit(int dimension) const {
  if (dimension < 1 || dimension > 6)
    throw std::invalid_argument("NIS gate supports 1..6 observation components");

  return limits_[static_cast<std::size_t>(dimension - 1)];
}

HealthMonitor::HealthMonitor(HealthLimits limits) : limits_(limits) {
  if (!limits_.maximum_consecutive_rejections ||
      !std::isfinite(limits_.maximum_position_variance_m2) ||
      limits_.maximum_position_variance_m2 <= 0 ||
      !std::isfinite(limits_.maximum_phase_variance_rad2) ||
      limits_.maximum_phase_variance_rad2 <= 0)
    throw std::invalid_argument("Invalid filter health limits");
}

HealthReport HealthMonitor::observe(const TargetState& state, const StateCovariance& covariance,
                                    bool accepted) {
  if (accepted)
    rejections_ = 0;
  else if (rejections_ < limits_.maximum_consecutive_rejections)
    ++rejections_;

  const bool numeric = state_valid(state) && math::covariance_valid(covariance);
  const bool uncertain =
      !numeric ||
      covariance.diagonal().segment<3>(component_index(StateComponent::x)).maxCoeff() >
          limits_.maximum_position_variance_m2 ||
      covariance(component_index(StateComponent::phase), component_index(StateComponent::phase)) >
          limits_.maximum_phase_variance_rad2;

  return {numeric, uncertain || rejections_ >= limits_.maximum_consecutive_rejections, rejections_};
}
} // namespace autoaim::estimation
