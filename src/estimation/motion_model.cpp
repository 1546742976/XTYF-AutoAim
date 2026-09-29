#include "autoaim/estimation/motion_model.hpp"
#include "autoaim/math/angle.hpp"
#include "autoaim/math/numeric.hpp"
#include <algorithm>

namespace autoaim::estimation {
bool state_valid(const TargetState& state) noexcept {
  return state.center.metres().allFinite() && state.velocity_mps.allFinite() &&
    std::isfinite(state.phase.value()) && std::isfinite(state.omega_radps) && std::isfinite(state.alpha_radps2);
}
StateVector state_difference(const TargetState& actual, const TargetState& reference) {
  StateVector result;
  result.segment<3>(component_index(StateComponent::x)) = actual.center.metres() - reference.center.metres();
  result.segment<3>(component_index(StateComponent::vx)) = actual.velocity_mps - reference.velocity_mps;
  result[component_index(StateComponent::phase)] = math::angle_residual(actual.phase, reference.phase).value();
  result[component_index(StateComponent::omega)] = actual.omega_radps - reference.omega_radps;
  result[component_index(StateComponent::alpha)] = actual.alpha_radps2 - reference.alpha_radps2;
  return result;
}
TargetState add_state_delta(const TargetState& state, const StateVector& delta) {
  if (!delta.allFinite()) throw std::invalid_argument("Non-finite state correction");
  return {math::Point3<math::WorldFrame>(state.center.metres() + delta.segment<3>(component_index(StateComponent::x))),
    state.velocity_mps + delta.segment<3>(component_index(StateComponent::vx)),
    math::wrap_to_pi(core::Radians(state.phase.value() + delta[component_index(StateComponent::phase)])),
    state.omega_radps + delta[component_index(StateComponent::omega)],
    state.alpha_radps2 + delta[component_index(StateComponent::alpha)]};
}
MotionModel::MotionModel(double translation_noise, double angular_noise, core::Seconds max_horizon)
    : translation_noise_(translation_noise), angular_noise_(angular_noise), max_horizon_(max_horizon) {
  if (!std::isfinite(translation_noise_) || translation_noise_ < 0 || !std::isfinite(angular_noise_) ||
      angular_noise_ < 0 || max_horizon_.value() <= 0) throw std::invalid_argument("Invalid motion model limits");
}
MotionModel::MotionModel(double translation_noise, double jerk_noise, core::Seconds max_horizon, double alpha_limit)
    : MotionModel(translation_noise, jerk_noise, max_horizon) {
  if (!std::isfinite(alpha_limit) || alpha_limit <= 0) throw std::invalid_argument("Invalid angular acceleration bound");
  kind_ = MotionKind::bounded_acceleration;
  alpha_limit_ = alpha_limit;
}
TargetState MotionModel::constrain(const TargetState& state) const {
  if (!state_valid(state)) throw std::invalid_argument("Non-finite motion state");
  auto result = state;
  result.phase = math::wrap_to_pi(state.phase);
  result.alpha_radps2 = kind_ == MotionKind::constant_velocity ? 0 :
    std::clamp(state.alpha_radps2, -alpha_limit_, alpha_limit_);
  return result;
}
core::Result<StatePrediction> MotionModel::remap_from(const MotionModel& previous, const TargetState& state,
    const StateCovariance& covariance, double initial_alpha_variance) const {
  using Result = core::Result<StatePrediction>;
  if (!state_valid(state) || !math::covariance_valid(covariance) ||
      !std::isfinite(initial_alpha_variance) || initial_alpha_variance <= 0)
    return Result::failure(core::ErrorCode::invalid_input, "Invalid motion model mapping");
  auto mapped = constrain(state);
  StateCovariance transform = StateCovariance::Identity(), result = covariance;
  constexpr int alpha = component_index(StateComponent::alpha);
  if (previous.kind_ != kind_ || kind_ == MotionKind::constant_velocity) {
    transform.row(alpha).setZero();
    result = transform * covariance * transform.transpose();
    mapped.alpha_radps2 = 0;
    if (kind_ == MotionKind::bounded_acceleration) result(alpha, alpha) = initial_alpha_variance;
  }
  return Result::success({std::move(mapped), std::move(result), std::move(transform)});
}
core::Result<StatePrediction> MotionModel::propagate(const TargetState& state,
    const StateCovariance& covariance, core::Seconds duration) const {
  using Result = core::Result<StatePrediction>;
  const double dt = duration.value();
  if (!state_valid(state) || !math::covariance_valid(covariance) || dt < 0 || dt > max_horizon_.value())
    return Result::failure(core::ErrorCode::invalid_input, "Invalid motion state/covariance or horizon");
  StateCovariance transition = StateCovariance::Identity(), noise = StateCovariance::Zero();
  constexpr int phase = component_index(StateComponent::phase), omega = component_index(StateComponent::omega);
  constexpr int alpha = component_index(StateComponent::alpha);
  if (kind_ == MotionKind::constant_velocity) {
    transition.row(alpha).setZero(); transition.col(alpha).setZero();
  }
  for (int axis = 0; axis < 3; ++axis) {
    const int position = component_index(StateComponent::x) + axis;
    const int velocity = component_index(StateComponent::vx) + axis;
    transition(position, velocity) = dt;
    noise(position, position) = translation_noise_ * dt * dt * dt / 3;
    noise(position, velocity) = noise(velocity, position) = translation_noise_ * dt * dt / 2;
    noise(velocity, velocity) = translation_noise_ * dt;
  }
  transition(phase, omega) = dt;
  if (kind_ == MotionKind::constant_velocity) {
    noise(phase, phase) = angular_noise_ * dt * dt * dt / 3;
    noise(phase, omega) = noise(omega, phase) = angular_noise_ * dt * dt / 2;
    noise(omega, omega) = angular_noise_ * dt;
  } else {
    transition(phase, alpha) = dt * dt / 2;
    transition(omega, alpha) = dt;
    // 连续白 jerk 的精确离散协方差，长跨度自然增加相位不确定度。
    Eigen::Matrix3d block;
    block << std::pow(dt, 5) / 20, std::pow(dt, 4) / 8, std::pow(dt, 3) / 6,
             std::pow(dt, 4) / 8, std::pow(dt, 3) / 3, dt * dt / 2,
             std::pow(dt, 3) / 6, dt * dt / 2, dt;
    noise.block<3, 3>(phase, phase) = angular_noise_ * block;
  }
  const auto bounded = constrain(state);
  const Eigen::Vector3d center = state.center.metres() + state.velocity_mps * dt;
  const double angle = bounded.phase.value() + bounded.omega_radps * dt + bounded.alpha_radps2 * dt * dt / 2;
  const double speed = bounded.omega_radps + bounded.alpha_radps2 * dt;
  if (!center.allFinite() || !std::isfinite(angle) || !std::isfinite(speed))
    return Result::failure(core::ErrorCode::invalid_input, "Motion extrapolation overflow");
  TargetState predicted{math::Point3<math::WorldFrame>(center), state.velocity_mps,
    math::wrap_to_pi(core::Radians(angle)), speed, bounded.alpha_radps2};
  StateCovariance uncertainty = transition * covariance * transition.transpose() + noise;
  uncertainty = ((uncertainty + uncertainty.transpose()) / 2).eval();
  if (!state_valid(predicted) || !math::covariance_valid(uncertainty))
    return Result::failure(core::ErrorCode::invalid_input, "Motion prediction became non-finite");
  return Result::success({std::move(predicted), std::move(uncertainty), std::move(transition)});
}
}  // namespace autoaim::estimation
