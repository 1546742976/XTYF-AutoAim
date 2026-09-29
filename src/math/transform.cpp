#include "autoaim/math/transform.hpp"
#include <cmath>

namespace autoaim::math {
SE3 interpolate(const SE3& first, const SE3& second, double fraction) {
  if (!std::isfinite(fraction) || fraction < 0 || fraction > 1)
    throw std::invalid_argument("Interpolation fraction outside [0, 1]");

  return SE3(first.rotation().slerp(fraction, second.rotation()),
             (1 - fraction) * first.translation() + fraction * second.translation());
}
} // namespace autoaim::math
