#include "autoaim/estimation/ekf.hpp"
#include "autoaim/math/numeric.hpp"

namespace autoaim::estimation {
Ekf::Ekf(TargetState state, StateCovariance covariance, std::shared_ptr<const MotionModel> motion)
    : state_(std::move(state)), covariance_(std::move(covariance)), motion_(std::move(motion)) {
  if (!motion_)
    throw std::invalid_argument("EKF motion model is required");

  auto projected = motion_->propagate(state_, covariance_, core::Seconds(0));

  if (!projected)
    throw std::invalid_argument(projected.error().message);

  state_ = projected.value().state;
  covariance_ = projected.value().covariance;
}

core::Result<bool> Ekf::predict(core::Seconds dt) {
  auto result = motion_->propagate(state_, covariance_, dt);

  if (!result)
    return core::Result<bool>::failure(result.error().code, result.error().message);

  state_ = result.value().state;
  covariance_ = result.value().covariance;

  return core::Result<bool>::success(true);
}

core::Result<Innovation> Ekf::innovation(const LinearizedMeasurement& measurement) const {
  using Result = core::Result<Innovation>;
  const auto dimension = measurement.residual.size();

  if (dimension <= 0 || measurement.jacobian.rows() != dimension ||
      measurement.jacobian.cols() != state_dimension || measurement.noise.rows() != dimension ||
      !measurement.residual.allFinite() || !measurement.jacobian.allFinite() ||
      !math::covariance_valid(measurement.noise))
    return Result::failure(core::ErrorCode::invalid_input,
                           "Invalid measurement dimensions/numerics");

  Eigen::MatrixXd covariance =
      measurement.jacobian * covariance_ * measurement.jacobian.transpose() + measurement.noise;

  covariance = ((covariance + covariance.transpose()) / 2).eval();
  auto weighted = math::solve_positive_definite(covariance, measurement.residual);

  if (!weighted)
    return Result::failure(weighted.error().code, weighted.error().message);

  const double nis = measurement.residual.dot(weighted.value().col(0));

  if (!std::isfinite(nis) || nis < 0)
    return Result::failure(core::ErrorCode::invalid_input, "Invalid prior NIS");

  return Result::success({std::move(covariance), nis});
}

core::Result<UpdateReport> Ekf::update(const LinearizedMeasurement& measurement, double nis_limit) {
  using Result = core::Result<UpdateReport>;

  if (!std::isfinite(nis_limit) || nis_limit <= 0)
    return Result::failure(core::ErrorCode::invalid_input, "Invalid NIS gate");

  const auto prior = innovation(measurement);

  if (!prior)
    return Result::failure(prior.error().code, prior.error().message);

  const int dimension = static_cast<int>(measurement.residual.size());

  if (prior.value().nis > nis_limit)
    return Result::success({false, prior.value().nis, dimension});

  auto solved =
      math::solve_positive_definite(prior.value().covariance, measurement.jacobian * covariance_);

  if (!solved)
    return Result::failure(solved.error().code, solved.error().message);

  const Eigen::MatrixXd gain = solved.value().transpose();
  const StateCovariance remainder = StateCovariance::Identity() - gain * measurement.jacobian;
  StateCovariance posterior =
      remainder * covariance_ * remainder.transpose() + gain * measurement.noise * gain.transpose();

  posterior = ((posterior + posterior.transpose()) / 2).eval();

  try {
    auto corrected = motion_->constrain(add_state_delta(state_, gain * measurement.residual));

    if (!state_valid(corrected) || !math::covariance_valid(posterior))
      return Result::failure(core::ErrorCode::invalid_input, "Invalid posterior state/covariance");

    // 上述计算全部成功之后才提交，任何拒绝/数值失败都不会污染先验。
    state_ = std::move(corrected);
    covariance_ = std::move(posterior);
  } catch (const std::exception& error) {
    return Result::failure(core::ErrorCode::invalid_input, error.what());
  }

  return Result::success({true, prior.value().nis, dimension});
}

core::Result<bool> Ekf::change_motion(std::shared_ptr<const MotionModel> model,
                                      double initial_alpha_variance) {
  if (!model)
    return core::Result<bool>::failure(core::ErrorCode::invalid_input, "Missing new motion model");

  auto mapped = model->remap_from(*motion_, state_, covariance_, initial_alpha_variance);

  if (!mapped)
    return core::Result<bool>::failure(mapped.error().code, mapped.error().message);

  state_ = mapped.value().state;
  covariance_ = mapped.value().covariance;
  motion_ = std::move(model);

  return core::Result<bool>::success(true);
}
} // namespace autoaim::estimation
