#pragma once

#include "autoaim/core/result.hpp"
#include <Eigen/Core>

namespace autoaim::math {
// 协方差允许半正定；求解器要求正定且条件比不低于指定门槛。
bool covariance_valid(const Eigen::MatrixXd& covariance, double tolerance = 1e-10);
core::Result<Eigen::MatrixXd> solve_positive_definite(
  const Eigen::MatrixXd& matrix, const Eigen::MatrixXd& rhs, double minimum_ratio = 1e-12);
}  // namespace autoaim::math
