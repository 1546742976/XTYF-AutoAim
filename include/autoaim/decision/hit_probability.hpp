#pragma once

#include "autoaim/decision/predictor.hpp"

namespace autoaim::decision {
struct ImpactUncertainty {
  double speed_sigma_mps;
  Eigen::Matrix3d aim_rotation_covariance_rad2;
  Eigen::Matrix3d launch_origin_covariance_m2;

  // 发射相对目标时间轴的误差，影响目标在实际发射时的位置，不是重新定义曝光时刻。
  double launch_timing_sigma_s;
  Eigen::Matrix3d scatter_covariance_world_m2;
};

struct MarginOptions {
  double sigma_multiplier;
  core::Metres reserved_edge_margin;
  double minimum_normal_speed_mps;
};

struct ImpactMargin {
  const core::Stamp source;
  const Eigen::Vector2d mean_error_plate_m;
  const Eigen::Matrix2d covariance_plate_m2;
  const Eigen::Vector2d remaining_margin_m;
  const bool inside;

  ImpactMargin(core::Stamp stamp, Eigen::Vector2d error, Eigen::Matrix2d covariance,
               Eigen::Vector2d margin, bool contained)
      : source(stamp), mean_error_plate_m(std::move(error)),
        covariance_plate_m2(std::move(covariance)), remaining_margin_m(std::move(margin)),
        inside(contained) {
  }
};

// 名称保留旧文件路径，但首版只提供一阶协方差与 k-sigma 板边距代理，不返回“命中概率”。
// 相关位姿噪声保留；其它显式噪声块按独立近似处理。切线入射/病态平面交点拒绝求值。
core::Result<ImpactMargin>
evaluate_impact_margin(const InterceptSolution& solution,
                       const math::Point3<math::WorldFrame>& launch_origin,
                       const BallisticModel& ballistic, const ImpactUncertainty& uncertainty,
                       const MarginOptions& options);
} // namespace autoaim::decision
