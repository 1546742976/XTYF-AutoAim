#pragma once

#include "autoaim/estimation/ekf.hpp"
#include "autoaim/estimation/eso.hpp"
#include <type_traits>
#include <utility>

namespace autoaim::estimation {
// 统一由 CMake 选择；运行时参数不改变算法类型。
#if AUTOAIM_USE_ESO
using StateEstimator = Eso;
#else
using StateEstimator = Ekf;
#endif

inline constexpr bool eso_enabled = !std::is_same_v<StateEstimator, Ekf>;
inline constexpr const char* estimator_name = eso_enabled ? "eso" : "ekf";

// 编译期选择：EKF 保持原三参数构造，ESO 的实验参数通过其四参数构造传入。
// factory 只构造当前编译选择的类型，没有运行时算法选择。
template <class Estimator = StateEstimator>
Estimator make_state_estimator(TargetState state, StateCovariance covariance,
                               std::shared_ptr<const MotionModel> motion, EsoOptions options = {}) {
  if constexpr (std::is_same_v<Estimator, Ekf>)
    return Estimator(std::move(state), std::move(covariance), std::move(motion));
  else
    return Estimator(std::move(state), std::move(covariance), std::move(motion), options);
}
} // namespace autoaim::estimation
