#include "autoaim/math/numeric.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>

namespace autoaim::math {
bool covariance_valid(const Eigen::MatrixXd& covariance, double tolerance) {
  if (covariance.rows() == 0 || covariance.rows() != covariance.cols() || !covariance.allFinite() ||
      !std::isfinite(tolerance) || tolerance < 0 ||
      !covariance.isApprox(covariance.transpose(), tolerance))
    return false;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance, Eigen::EigenvaluesOnly);

  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -tolerance;
}

core::Result<Eigen::MatrixXd> solve_positive_definite(const Eigen::MatrixXd& matrix,
                                                      const Eigen::MatrixXd& rhs,
                                                      double minimum_ratio) {
  using Result = core::Result<Eigen::MatrixXd>;

  if (!std::isfinite(minimum_ratio) || minimum_ratio <= 0 || minimum_ratio > 1 ||
      matrix.rows() == 0 || matrix.rows() != matrix.cols() || matrix.rows() != rhs.rows() ||
      !matrix.allFinite() || !rhs.allFinite() || !matrix.isApprox(matrix.transpose(), 1e-10))
    return Result::failure(core::ErrorCode::invalid_input, "Invalid symmetric system");

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(matrix, Eigen::EigenvaluesOnly);

  if (spectrum.info() != Eigen::Success || spectrum.eigenvalues().maxCoeff() <= 0 ||
      spectrum.eigenvalues().minCoeff() <= minimum_ratio * spectrum.eigenvalues().maxCoeff())
    return Result::failure(core::ErrorCode::invalid_input, "Singular or ill-conditioned system");

  Eigen::LDLT<Eigen::MatrixXd> factor(matrix);
  Eigen::MatrixXd result = factor.solve(rhs);

  if (factor.info() != Eigen::Success || !result.allFinite())
    return Result::failure(core::ErrorCode::invalid_input, "Numerical solve failed");

  return Result::success(std::move(result));
}
} // namespace autoaim::math
