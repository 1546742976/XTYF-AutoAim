#pragma once

#include "autoaim/estimation/motion_model.hpp"
#include <array>

namespace autoaim::estimation {
// 1..6 维高斯创新的理论 95% 上侧门限；实测非高斯残差可以显式提供替代表。
// 不将理论 NIS 门限称为实测命中概率，也不在线用后验减先验冒充 NEES。
std::array<double, 6> chi_square_95_limits();

class NisGate {
public:
  explicit NisGate(std::array<double, 6> limits);
  double limit(int observation_dimension) const;

private:
  std::array<double, 6> limits_;
};

struct HealthLimits {
  std::size_t maximum_consecutive_rejections;
  double maximum_position_variance_m2;
  double maximum_phase_variance_rad2;
};

struct HealthReport {
  bool numerical_validity;
  bool reset_required;
  std::size_t consecutive_rejections;
};

class HealthMonitor {
public:
  explicit HealthMonitor(HealthLimits limits);
  HealthReport observe(const TargetState& state, const StateCovariance& covariance,
                       bool update_accepted);

  void reset() noexcept {
    rejections_ = 0;
  }

private:
  HealthLimits limits_;
  std::size_t rejections_ = 0;
};
} // namespace autoaim::estimation
