#include "autoaim/math/se3.hpp"
#include <cmath>
#include <stdexcept>

namespace autoaim::math {
Eigen::Quaterniond checked_rotation(const Eigen::Quaterniond& rotation) {
  const double norm = rotation.norm();

  if (!rotation.coeffs().allFinite() || !std::isfinite(norm) || std::abs(norm - 1) > 0.01)
    throw std::invalid_argument("Invalid unit quaternion");

  return rotation.normalized();
}

Eigen::Quaterniond rotation_exp(const Eigen::Vector3d& tangent) {
  const double angle = tangent.stableNorm();

  if (!tangent.allFinite() || !std::isfinite(angle))
    throw std::invalid_argument("Invalid rotation tangent");

  if (angle < 1e-10)
    return Eigen::Quaterniond(1, tangent.x() / 2, tangent.y() / 2, tangent.z() / 2).normalized();

  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, tangent / angle));
}

Eigen::Vector3d rotation_log(const Eigen::Quaterniond& rotation) {
  auto q = checked_rotation(rotation);

  if (q.w() < 0)
    q.coeffs() *= -1; // q 与 -q 表示同一姿态，选最短弧。
  const double sine = q.vec().norm();

  if (sine < 1e-10)
    return 2 * q.vec();

  return q.vec() * (2 * std::atan2(sine, q.w()) / sine);
}

SE3::SE3(Eigen::Quaterniond rotation, Eigen::Vector3d translation)
    : rotation_(checked_rotation(rotation)), translation_(std::move(translation)) {
  if (!translation_.allFinite())
    throw std::invalid_argument("Non-finite translation");
}

Eigen::Vector3d SE3::apply(const Eigen::Vector3d& point) const {
  if (!point.allFinite())
    throw std::invalid_argument("Non-finite point");

  const Eigen::Vector3d transformed = rotation_ * point + translation_;

  if (!transformed.allFinite())
    throw std::overflow_error("Point transform overflow");

  return transformed;
}

SE3 SE3::inverse() const {
  return SE3(rotation_.conjugate(), -(rotation_.conjugate() * translation_));
}

SE3 SE3::compose(const SE3& other) const {
  return SE3(rotation_ * other.rotation_, apply(other.translation_));
}
} // namespace autoaim::math
