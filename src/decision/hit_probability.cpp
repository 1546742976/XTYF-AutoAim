#include "autoaim/decision/hit_probability.hpp"
#include "autoaim/math/numeric.hpp"

namespace autoaim::decision {
namespace {
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;

  return result;
}
} // namespace

core::Result<ImpactMargin> evaluate_impact_margin(const InterceptSolution& solution,
                                                  const math::Point3<math::WorldFrame>& origin,
                                                  const BallisticModel& ballistic,
                                                  const ImpactUncertainty& noise,
                                                  const MarginOptions& options) {
  using Result = core::Result<ImpactMargin>;

  if (!std::isfinite(noise.speed_sigma_mps) || noise.speed_sigma_mps < 0 ||
      !std::isfinite(noise.launch_timing_sigma_s) || noise.launch_timing_sigma_s < 0 ||
      !std::isfinite(options.sigma_multiplier) || options.sigma_multiplier <= 0 ||
      options.reserved_edge_margin.value() < 0 ||
      !std::isfinite(options.minimum_normal_speed_mps) || options.minimum_normal_speed_mps <= 0 ||
      !math::covariance_valid(noise.aim_rotation_covariance_rad2) ||
      !math::covariance_valid(noise.launch_origin_covariance_m2) ||
      !math::covariance_valid(noise.scatter_covariance_world_m2) ||
      !math::covariance_valid(solution.plate.covariance) ||
      !solution.plate.velocity_world_mps.allFinite() ||
      !solution.ballistic.initial_velocity_world_mps.allFinite() ||
      solution.ballistic.initial_velocity_world_mps.norm() <= 0 ||
      solution.ballistic.flight_time.value() <= 0)
    return Result::failure(core::ErrorCode::invalid_input, "Invalid impact uncertainty inputs");

  const double t = solution.ballistic.flight_time.value();
  const auto& plate = solution.plate;
  const Eigen::Vector3d velocity = solution.ballistic.initial_velocity_world_mps;
  const Eigen::Vector3d bullet =
      ballistic.position(origin, velocity, solution.ballistic.flight_time).metres();

  const Eigen::Vector3d offset = bullet - plate.pose.value().translation();
  const Eigen::Vector3d relative_velocity =
      velocity + ballistic.gravity() * t - plate.velocity_world_mps;

  const Eigen::Matrix3d rotation = plate.pose.value().rotation().toRotationMatrix();
  const Eigen::Vector3d normal = rotation.col(2);
  const double normal_speed = normal.dot(relative_velocity);

  if (!std::isfinite(normal_speed) || normal_speed >= -options.minimum_normal_speed_mps)
    return Result::failure(core::ErrorCode::unavailable,
                           "Tangential or back-face plane intersection");

  const Eigen::Matrix<double, 2, 3> local = rotation.transpose().topRows<2>();

  // 对 n·(b-p)=0 隐式求交时刻导数；不能仅截取固定飞行时间下的 xyz 协方差。
  const Eigen::Matrix3d intersection =
      Eigen::Matrix3d::Identity() - relative_velocity * normal.transpose() / normal_speed;

  const Eigen::Matrix<double, 2, 3> projectile_jacobian = local * intersection;
  Eigen::Matrix<double, 2, 6> pose_jacobian;
  pose_jacobian.leftCols<3>() = -projectile_jacobian;
  pose_jacobian.rightCols<3>() =
      local * (skew(offset) - relative_velocity * normal.cross(offset).transpose() / normal_speed);

  const Eigen::Vector3d direction = velocity.normalized();
  const Eigen::Matrix3d angular_jacobian = -t * skew(velocity);
  Eigen::Matrix3d projectile_covariance =
      noise.launch_origin_covariance_m2 + noise.scatter_covariance_world_m2 +
      angular_jacobian * noise.aim_rotation_covariance_rad2 * angular_jacobian.transpose() +
      t * t * noise.speed_sigma_mps * noise.speed_sigma_mps * direction * direction.transpose();

  // 发射时间误差首先移动目标；整体平移交点采样时间不会凭空产生落点误差。
  projectile_covariance += noise.launch_timing_sigma_s * noise.launch_timing_sigma_s *
                           plate.velocity_world_mps * plate.velocity_world_mps.transpose();

  Eigen::Matrix2d covariance =
      projectile_jacobian * projectile_covariance * projectile_jacobian.transpose() +
      pose_jacobian * plate.covariance * pose_jacobian.transpose();

  covariance = ((covariance + covariance.transpose()) / 2).eval();

  if (!math::covariance_valid(covariance))
    return Result::failure(core::ErrorCode::invalid_input, "Impact covariance invalid");

  const Eigen::Vector2d mean = local * offset;
  const Eigen::Vector2d half(plate.dimensions.width.value() / 2,
                             plate.dimensions.height.value() / 2);

  const Eigen::Vector2d margin =
      half - mean.cwiseAbs() -
      options.sigma_multiplier * covariance.diagonal().cwiseMax(0).cwiseSqrt() -
      Eigen::Vector2d::Constant(options.reserved_edge_margin.value());

  return Result::success(
      ImpactMargin(solution.source, mean, covariance, margin, margin.minCoeff() > 0));
}
} // namespace autoaim::decision
