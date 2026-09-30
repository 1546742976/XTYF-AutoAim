#pragma once

// 估计器由 CMake 的 AUTOAIM_USE_ESO 选择，默认使用 EKF。
// 完整实现始终可编译；实验参数在 filter_types.hpp 中定义。
#include "autoaim/estimation/filter_types.hpp"
#include "autoaim/estimation/measurement_model.hpp"
#include "autoaim/math/numeric.hpp"
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace autoaim::estimation {
// 离散 current observer：预测在先，校正在后，不把带宽增益当作 Kalman 增益。
// 三个平移轴使用三阶 ESO；角 CV/CA 分别使用二阶/三阶 ESO。
// https://arxiv.org/html/2211.07309v2 ，III-C，式 (9)-(10) 后的离散极点公式。
class Eso {
public:
  Eso(TargetState state, StateCovariance covariance, std::shared_ptr<const MotionModel> motion)
      : Eso(std::move(state), std::move(covariance), std::move(motion), EsoOptions{}) {
  }

  Eso(TargetState state, StateCovariance covariance, std::shared_ptr<const MotionModel> motion,
      EsoOptions options)
      : state_(std::move(state)), covariance_(std::move(covariance)), options_(options),
        batch_{state_, covariance_, Gain::Zero(), Eigen::Matrix4d::Zero(),
               Eigen::Vector4d::Zero(), true} {
    if (!motion || !std::isfinite(options_.translation_bandwidth_radps) ||
        options_.translation_bandwidth_radps <= 0 ||
        !std::isfinite(options_.angular_bandwidth_radps) || options_.angular_bandwidth_radps <= 0 ||
        !std::isfinite(options_.linear_jerk_psd) || options_.linear_jerk_psd < 0)
      throw std::invalid_argument("Invalid ESO motion model or experimental parameters");

    // 复制调用者的角模型、时域与角加速度约束，仅将平移模式改为 CA。
    motion_ = std::make_shared<const MotionModel>(
        motion->with_linear_acceleration(options_.linear_jerk_psd));
    const auto projected = motion_->propagate(state_, covariance_, core::Seconds(0));
    if (!projected)
      throw std::invalid_argument(projected.error().message);

    state_ = projected.value().state;
    covariance_ = projected.value().covariance;
    batch_ = new_batch(state_, covariance_, injection_gain(0, motion_->kind()).value(), true);
  }

  const TargetState& state() const noexcept {
    return state_;
  }

  const StateCovariance& covariance() const noexcept {
    return covariance_;
  }

  const std::shared_ptr<const MotionModel>& motion() const noexcept {
    return motion_;
  }

  core::Result<bool> predict(core::Seconds dt) {
    using Result = core::Result<bool>;
    const auto prediction = motion_->propagate(state_, covariance_, dt);
    if (!prediction)
      return Result::failure(prediction.error().code, prediction.error().message);

    // 零时间预测既不丢失本次曝光已经接受的观测，也不重复创建校正间隔。
    if (dt.value() == 0)
      return Result::success(true);

    const double elapsed = elapsed_since_correction_ + dt.value();
    const auto gain = injection_gain(elapsed, motion_->kind());
    if (!gain)
      return Result::failure(gain.error().code, gain.error().message);

    auto next = new_batch(prediction.value().state, prediction.value().covariance,
                          gain.value(), true);
    state_ = prediction.value().state;
    covariance_ = prediction.value().covariance;
    elapsed_since_correction_ = elapsed;
    batch_ = std::move(next);
    return Result::success(true);
  }

  core::Result<Innovation> innovation(const LinearizedMeasurement& measurement) const {
    using Result = core::Result<Innovation>;
    auto prepared = prepare(measurement);
    if (!prepared)
      return Result::failure(prepared.error().code, prepared.error().message);
    return Result::success(std::move(prepared.value().prior));
  }

  core::Result<UpdateReport> update(const LinearizedMeasurement& measurement, double nis_limit) {
    using Result = core::Result<UpdateReport>;
    if (!std::isfinite(nis_limit) || nis_limit <= 0)
      return Result::failure(core::ErrorCode::invalid_input, "Invalid NIS gate");

    const auto prepared = prepare(measurement);
    if (!prepared)
      return Result::failure(prepared.error().code, prepared.error().message);
    const double nis = prepared.value().prior.nis;
    if (nis > nis_limit)
      return Result::success({false, nis, 6});

    const Eigen::Matrix4d information = batch_.information + prepared.value().information;
    const Eigen::Vector4d rhs = batch_.rhs + prepared.value().rhs;
    const auto equivalent_noise =
        math::solve_positive_definite(information, Eigen::Matrix4d::Identity());
    if (!equivalent_noise)
      return Result::failure(equivalent_noise.error().code, equivalent_noise.error().message);

    const Eigen::Vector4d correction = equivalent_noise.value() * rhs;
    // W = (Σ JᵀR⁻¹J)⁻¹JᵀR⁻¹，K = G W，故 K H = G C、K R Kᵀ = G A⁻¹Gᵀ。
    // 每次从固定批先验重算联合 Joseph 后验，不能在上次后验上再次注入完整 G。
    const StateCovariance remainder = StateCovariance::Identity() - batch_.gain * selector();
    StateCovariance posterior = remainder * batch_.covariance * remainder.transpose() +
                                batch_.gain * equivalent_noise.value() * batch_.gain.transpose();
    posterior = ((posterior + posterior.transpose()) / 2).eval();

    try {
      auto corrected = motion_->constrain(add_state_delta(batch_.state, batch_.gain * correction));
      if (!state_valid(corrected) || !math::covariance_valid(posterior))
        return Result::failure(core::ErrorCode::invalid_input,
                               "Invalid ESO posterior state/covariance");

      // 所有检查完成才提交。拒绝或数值失败保留状态、P、模型、计时和已融合信息。
      state_ = std::move(corrected);
      covariance_ = std::move(posterior);
      batch_.information = information;
      batch_.rhs = rhs;
      elapsed_since_correction_ = 0;
      has_corrected_ = true;
    } catch (const std::exception& error) {
      return Result::failure(core::ErrorCode::invalid_input, error.what());
    }
    return Result::success({true, nis, 6});
  }

  core::Result<bool> change_motion(std::shared_ptr<const MotionModel> motion,
                                   double initial_alpha_variance) {
    using Result = core::Result<bool>;
    if (!motion)
      return Result::failure(core::ErrorCode::invalid_input, "Missing new motion model");

    auto replacement = std::make_shared<const MotionModel>(
        motion->with_linear_acceleration(options_.linear_jerk_psd));
    const auto mapped =
        replacement->remap_from(*motion_, state_, covariance_, initial_alpha_variance);
    if (!mapped)
      return Result::failure(mapped.error().code, mapped.error().message);
    const auto gain = injection_gain(elapsed_since_correction_, replacement->kind());
    if (!gain)
      return Result::failure(gain.error().code, gain.error().message);

    // 先映射完成的融合后验，再清除批缓存。已校正滤波器在零时间切换后须等到正时间
    // predict 才能再次校正；不能把这种切换误当成首次初始化来重新注入位置/相位。
    const bool allowed = !has_corrected_ || elapsed_since_correction_ > 0;
    auto next = new_batch(mapped.value().state, mapped.value().covariance, gain.value(), allowed);
    state_ = mapped.value().state;
    covariance_ = mapped.value().covariance;
    motion_ = std::move(replacement);
    batch_ = std::move(next);
    return Result::success(true);
  }

private:
  using Gain = Eigen::Matrix<double, state_dimension, 4>;
  using Selector = Eigen::Matrix<double, 4, state_dimension>;

  struct Batch {
    TargetState state;
    StateCovariance covariance;
    Gain gain;
    Eigen::Matrix4d information;
    Eigen::Vector4d rhs;
    bool correction_allowed;
  };

  struct PreparedMeasurement {
    Innovation prior;
    Eigen::Matrix4d information;
    Eigen::Vector4d rhs;
  };

  static Selector selector() {
    Selector result = Selector::Zero();
    result.block<3, 3>(0, component_index(StateComponent::x)).setIdentity();
    result(3, component_index(StateComponent::phase)) = 1;
    return result;
  }

  static Batch new_batch(const TargetState& state, const StateCovariance& covariance,
                         const Gain& gain, bool allowed) {
    return {state, covariance, gain, Eigen::Matrix4d::Zero(), Eigen::Vector4d::Zero(), allowed};
  }

  core::Result<Gain> injection_gain(double elapsed, MotionKind angular_kind) const {
    using Result = core::Result<Gain>;
    if (!std::isfinite(elapsed) || elapsed < 0)
      return Result::failure(core::ErrorCode::invalid_input, "Invalid ESO correction interval");
    Gain result = Gain::Zero();
    if (elapsed == 0) {
      // 首次没有时间间隔时只能修正可观测位置与相位，不能估计其时间导数。
      result = selector().transpose();
      return Result::success(std::move(result));
    }

    // p = exp(-ωT)，d = 1-p。采用 expm1 和 d/T，避免小时间步相减与 T² 下溢。
    const auto third_order = [elapsed](double bandwidth) {
      const double exponent = -bandwidth * elapsed;
      const double p = std::exp(exponent), d = -std::expm1(exponent), rate = d / elapsed;
      return Eigen::Vector3d(-std::expm1(3 * exponent), 1.5 * d * rate * (1 + p),
                             d * rate * rate);
    };
    const Eigen::Vector3d linear = third_order(options_.translation_bandwidth_radps);
    for (int axis = 0; axis < 3; ++axis) {
      result(component_index(StateComponent::x) + axis, axis) = linear[0];
      result(component_index(StateComponent::vx) + axis, axis) = linear[1];
      result(component_index(StateComponent::ax) + axis, axis) = linear[2];
    }
    constexpr int phase = component_index(StateComponent::phase);
    if (angular_kind == MotionKind::bounded_acceleration) {
      result.block<3, 1>(phase, 3) = third_order(options_.angular_bandwidth_radps);
    } else {
      const double exponent = -options_.angular_bandwidth_radps * elapsed;
      const double d = -std::expm1(exponent);
      result(phase, 3) = -std::expm1(2 * exponent);
      result(component_index(StateComponent::omega), 3) = d * (d / elapsed);
    }
    if (!result.allFinite())
      return Result::failure(core::ErrorCode::invalid_input, "Non-finite ESO injection gain");
    return Result::success(std::move(result));
  }

  core::Result<PreparedMeasurement> prepare(const LinearizedMeasurement& measurement) const {
    using Result = core::Result<PreparedMeasurement>;
    if (!batch_.correction_allowed)
      return Result::failure(core::ErrorCode::invalid_input,
                             "ESO needs a positive prediction interval after model switching");
    if (measurement.residual.size() != 6 || measurement.jacobian.rows() != 6 ||
        measurement.jacobian.cols() != state_dimension || measurement.noise.rows() != 6 ||
        measurement.noise.cols() != 6 || !measurement.residual.allFinite() ||
        !measurement.jacobian.allFinite())
      return Result::failure(core::ErrorCode::invalid_input,
                             "ESO requires a finite six-dimensional full-pose measurement");

    // 当前几何量测不直接观测速度/加速度，拒绝不符合这一结构的输入，不退回 EKF。
    constexpr int phase = component_index(StateComponent::phase);
    for (int column = 0; column < state_dimension; ++column)
      if (column >= 3 && column != phase && !measurement.jacobian.col(column).isZero(0))
        return Result::failure(core::ErrorCode::invalid_input,
                               "Unsupported ESO measurement Jacobian columns");
    const Eigen::Matrix<double, 6, 4> jacobian = measurement.jacobian * selector().transpose();
    // 后续观测在当前后验处线性化，必须先将残差回算到本曝光的固定批先验。
    // 这是局部线性回算；非线性重线性化与门控不承诺严格观测顺序无关。
    const Eigen::Matrix<double, 6, 1> residual = measurement.residual +
        measurement.jacobian * state_difference(state_, batch_.state);
    Eigen::Matrix<double, 6, 5> rhs;
    rhs.leftCols<4>() = jacobian;
    rhs.col(4) = residual;
    const auto weighted = math::solve_positive_definite(measurement.noise, rhs);
    if (!weighted)
      return Result::failure(weighted.error().code, weighted.error().message);

    Eigen::Matrix4d information = jacobian.transpose() * weighted.value().leftCols(4);
    information = ((information + information.transpose()) / 2).eval();
    const Eigen::Vector4d information_rhs = jacobian.transpose() * weighted.value().col(4);
    // 即使只是询问 innovation，也拒绝 R 非正定和四维不可观测的输入。
    const auto observability =
        math::solve_positive_definite(information, Eigen::Matrix4d::Identity());
    if (!observability)
      return Result::failure(observability.error().code, observability.error().message);

    Eigen::MatrixXd covariance = measurement.jacobian * batch_.covariance *
                                 measurement.jacobian.transpose() + measurement.noise;
    covariance = ((covariance + covariance.transpose()) / 2).eval();
    const auto normalized = math::solve_positive_definite(covariance, residual);
    if (!normalized)
      return Result::failure(normalized.error().code, normalized.error().message);
    const double nis = residual.dot(normalized.value().col(0));
    if (!std::isfinite(nis) || nis < 0 || !information_rhs.allFinite())
      return Result::failure(core::ErrorCode::invalid_input, "Invalid ESO batch-prior innovation");

    // 此 NIS 永远使用批先验，与 EKF 的逐观测后验门控不同；update 使用同一过程。
    return Result::success({{std::move(covariance), nis}, information, information_rhs});
  }

  TargetState state_;
  StateCovariance covariance_;
  std::shared_ptr<const MotionModel> motion_;
  EsoOptions options_;
  Batch batch_;
  double elapsed_since_correction_ = 0;
  bool has_corrected_ = false;
};
} // namespace autoaim::estimation
