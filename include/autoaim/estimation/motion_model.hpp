#pragma once

#include "autoaim/core/units.hpp"
#include "autoaim/core/result.hpp"
#include "autoaim/math/transform.hpp"
#include <Eigen/Core>

namespace autoaim::estimation {
enum class MotionKind { constant_velocity, bounded_acceleration };
// 下标只在估计器内部打包/线性代数边界使用，下游读取具名状态。
enum class StateComponent : int { x, y, z, vx, vy, vz, phase, omega, alpha, count };
inline constexpr int state_dimension = static_cast<int>(StateComponent::count);
using StateCovariance = Eigen::Matrix<double, state_dimension, state_dimension>;
using StateVector = Eigen::Matrix<double, state_dimension, 1>;
constexpr int component_index(StateComponent component) { return static_cast<int>(component); }
struct TargetState {
  math::Point3<math::WorldFrame> center;
  Eigen::Vector3d velocity_mps;
  core::Radians phase;
  double omega_radps;
  double alpha_radps2;
};
bool state_valid(const TargetState& state) noexcept;
// 打包只用于估计器数值边界；其余模块读取 TargetState 具名量。
StateVector state_difference(const TargetState& actual, const TargetState& reference);
TargetState add_state_delta(const TargetState& state, const StateVector& delta);
struct StatePrediction {
  TargetState state;
  StateCovariance covariance;
  StateCovariance transition;
};

class MotionModel {
public:
  // 连续白噪声功率谱密度：平移 m²/s³，恒角速度模型 rad²/s³。
  // max_horizon 限制一次传播跨度；跳帧必须使用真实 dt，不默默裁短。
  MotionModel(double translation_noise_psd, double angular_noise_psd, core::Seconds max_horizon);
  // 有界角加速度版本：angular_jerk_psd 单位 rad²/s⁵；界限为 rad/s²。
  MotionModel(double translation_noise_psd, double angular_jerk_psd, core::Seconds max_horizon,
              double maximum_abs_alpha_radps2);
  MotionKind kind() const noexcept { return kind_; }
  core::Result<StatePrediction> propagate(const TargetState& state,
      const StateCovariance& covariance, core::Seconds dt) const;
  // CV/CA 使用固定具名布局，但 alpha 的激活/失活必须显式映射。
  // 新激活 alpha=0、交叉协方差=0，方差由调用者明确提供；不是统一膨胀 P。
  core::Result<StatePrediction> remap_from(const MotionModel& previous, const TargetState& state,
      const StateCovariance& covariance, double initial_alpha_variance) const;
  TargetState constrain(const TargetState& state) const;
private:
  MotionKind kind_ = MotionKind::constant_velocity;
  double translation_noise_;
  double angular_noise_;
  core::Seconds max_horizon_;
  double alpha_limit_ = 0;
};
}  // namespace autoaim::estimation
