#include "autoaim/math/angle.hpp"
#include <cmath>

namespace autoaim::math {
core::Radians wrap_to_pi(core::Radians angle) {
  double wrapped = std::remainder(angle.value(), 2 * pi);
  if (wrapped >= pi) wrapped -= 2 * pi;
  return core::Radians(wrapped);
}

core::Radians angle_residual(core::Radians measured, core::Radians predicted) {
  // 分别归一化后相减，避免两个巨大但有限的输入先相减溢出。
  return wrap_to_pi(core::Radians(wrap_to_pi(measured).value() - wrap_to_pi(predicted).value()));
}
}  // namespace autoaim::math
