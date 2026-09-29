#pragma once

#include "autoaim/estimation/measurement_model.hpp"
#include <memory>

namespace autoaim::estimation {
struct Innovation {
  Eigen::MatrixXd covariance;
  double nis;
};
struct UpdateReport {
  bool accepted;
  double prior_nis;
  int observation_dimension;
};

// 单写者数值滤波器，不拥有时钟/任务/开火策略；predict 的 dt 由 Tracker 给出。
class Ekf {
public:
  Ekf(TargetState state, StateCovariance covariance, std::shared_ptr<const MotionModel> motion);
  const TargetState& state() const noexcept { return state_; }
  const StateCovariance& covariance() const noexcept { return covariance_; }
  const std::shared_ptr<const MotionModel>& motion() const noexcept { return motion_; }
  core::Result<bool> predict(core::Seconds dt);
  core::Result<Innovation> innovation(const LinearizedMeasurement& measurement) const;
  // 先计算/门控 NIS，再更新；拒绝时 state/P 完全不变。Joseph 形式保存半正定性。
  core::Result<UpdateReport> update(const LinearizedMeasurement& measurement, double nis_limit);
  core::Result<bool> change_motion(std::shared_ptr<const MotionModel> motion, double initial_alpha_variance);
private:
  TargetState state_;
  StateCovariance covariance_;
  std::shared_ptr<const MotionModel> motion_;
};
}  // namespace autoaim::estimation
