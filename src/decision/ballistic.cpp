#include "autoaim/decision/ballistic.hpp"
#include <cmath>

namespace autoaim::decision {
BallisticModel::BallisticModel(Eigen::Vector3d gravity, core::Seconds maximum_flight_time,
                               core::Metres minimum_range)
    : gravity_(std::move(gravity)), maximum_flight_time_(maximum_flight_time),
      minimum_range_(minimum_range) {
  if (!gravity_.allFinite() || !std::isfinite(gravity_.squaredNorm()) ||
      maximum_flight_time.value() <= 0 || minimum_range.value() <= 0)
    throw std::invalid_argument("Invalid ballistic limits");
}

core::Result<BallisticSolution> BallisticModel::solve(const math::Point3<math::WorldFrame>& origin,
                                                      const math::Point3<math::WorldFrame>& target,
                                                      core::MetresPerSecond speed) const {
  using Result = core::Result<BallisticSolution>;
  const Eigen::Vector3d displacement = target.metres() - origin.metres();

  if (speed.value() <= 0 || !displacement.allFinite() ||
      displacement.norm() < minimum_range_.value())
    return Result::failure(core::ErrorCode::invalid_input, "Invalid launch speed or target range");

  // ||r - g*t²/2||² = v²*t²，对 u=t² 解二次方程。选最短正飞行时间。
  const long double a = 0.25L * gravity_.squaredNorm();
  const long double b = -(static_cast<long double>(displacement.dot(gravity_)) +
                          static_cast<long double>(speed.value()) * speed.value());

  const long double c = displacement.squaredNorm();

  if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c))
    return Result::failure(core::ErrorCode::invalid_input, "Ballistic coefficients overflow");

  long double squared_time;

  if (a == 0) {
    squared_time = -c / b;
  } else {
    const long double discriminant = b * b - 4 * a * c;

    if (!std::isfinite(discriminant) || discriminant < 0 || b >= 0)
      return Result::failure(core::ErrorCode::unavailable, "No finite ballistic solution");

    // 小根用共轭式，避免低抛近直线时两个大数相减造成精度丢失。
    squared_time = 2 * c / (-b + std::sqrt(discriminant));
  }

  const double time = static_cast<double>(std::sqrt(squared_time));

  if (!std::isfinite(time) || time <= 0 || time > maximum_flight_time_.value())
    return Result::failure(core::ErrorCode::unavailable, "Ballistic flight time outside limit");

  const Eigen::Vector3d velocity = displacement / time - 0.5 * gravity_ * time;

  if (!velocity.allFinite() ||
      std::abs(velocity.norm() - speed.value()) > 1e-8 * std::max(1.0, speed.value()))
    return Result::failure(core::ErrorCode::invalid_input, "Ballistic residual invalid");

  return Result::success({velocity, core::Seconds(time)});
}

math::Point3<math::WorldFrame>
BallisticModel::position(const math::Point3<math::WorldFrame>& origin,
                         const Eigen::Vector3d& velocity, core::Seconds elapsed) const {
  if (!velocity.allFinite() || elapsed.value() < 0)
    throw std::invalid_argument("Invalid trajectory query");

  const double t = elapsed.value();

  return math::Point3<math::WorldFrame>(origin.metres() + velocity * t + 0.5 * gravity_ * t * t);
}
} // namespace autoaim::decision
