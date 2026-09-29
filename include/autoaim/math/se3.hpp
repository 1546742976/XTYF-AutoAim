#pragma once

#include <Eigen/Geometry>

namespace autoaim::math {
Eigen::Quaterniond checked_rotation(const Eigen::Quaterniond& rotation);
Eigen::Quaterniond rotation_exp(const Eigen::Vector3d& tangent);
Eigen::Vector3d rotation_log(const Eigen::Quaterniond& rotation);

// p_parent = rotation * p_local + translation；平移单位为米。
// 姿态通过 SO(3) 切空间处理，不对欧拉角逐分量相减。
class SE3 {
public:
  SE3(Eigen::Quaterniond rotation, Eigen::Vector3d translation);
  const Eigen::Quaterniond& rotation() const noexcept { return rotation_; }
  const Eigen::Vector3d& translation() const noexcept { return translation_; }
  Eigen::Vector3d apply(const Eigen::Vector3d& point) const;
  SE3 inverse() const;
  SE3 compose(const SE3& local_to_intermediate) const;
private:
  Eigen::Quaterniond rotation_;
  Eigen::Vector3d translation_;
};
}  // namespace autoaim::math
