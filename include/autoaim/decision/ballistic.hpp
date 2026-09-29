#pragma once

#include "autoaim/core/result.hpp"
#include "autoaim/core/units.hpp"
#include "autoaim/math/transform.hpp"

namespace autoaim::decision {
struct BallisticSolution {
  Eigen::Vector3d initial_velocity_world_mps;
  core::Seconds flight_time;
};

class BallisticModel {
public:
  // 显式世界系重力向量(m/s²)；不把云台角或相机轴方向当作世界竖直。
  BallisticModel(Eigen::Vector3d gravity_mps2, core::Seconds maximum_flight_time,
                 core::Metres minimum_range);

  core::Result<BallisticSolution> solve(const math::Point3<math::WorldFrame>& origin,
                                        const math::Point3<math::WorldFrame>& target,
                                        core::MetresPerSecond speed) const;

  math::Point3<math::WorldFrame> position(const math::Point3<math::WorldFrame>& origin,
                                          const Eigen::Vector3d& initial_velocity_world_mps,
                                          core::Seconds elapsed) const;

  const Eigen::Vector3d& gravity() const noexcept {
    return gravity_;
  }

private:
  Eigen::Vector3d gravity_;
  core::Seconds maximum_flight_time_;
  core::Metres minimum_range_;
};
} // namespace autoaim::decision
