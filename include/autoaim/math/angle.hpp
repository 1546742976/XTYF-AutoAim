#pragma once

#include "autoaim/core/units.hpp"

namespace autoaim::math {
inline constexpr double pi = 3.14159265358979323846;

// 返回 [-pi, pi)，用于周期角残差，不能用于直接相减三维欧拉姿态。
core::Radians wrap_to_pi(core::Radians angle);
core::Radians angle_residual(core::Radians measured, core::Radians predicted);
} // namespace autoaim::math
