#pragma once

#include "autoaim/math/se3.hpp"

namespace autoaim::math {
struct CameraFrame {};

struct GimbalFrame {};

struct WorldFrame {};

struct AxisFrame {};

struct PlateFrame {};

template <class Frame> class Point3 {
public:
  explicit Point3(Eigen::Vector3d metres) : metres_(std::move(metres)) {
    if (!metres_.allFinite())
      throw std::invalid_argument("Non-finite metric point");
  }

  const Eigen::Vector3d& metres() const noexcept {
    return metres_;
  }

private:
  Eigen::Vector3d metres_;
};

// From/To 进入类型，禁止把相机点无声当作世界点使用。
template <class From, class To> class Transform {
public:
  explicit Transform(SE3 transform) : transform_(std::move(transform)) {
  }

  Point3<To> apply(const Point3<From>& point) const {
    return Point3<To>(transform_.apply(point.metres()));
  }

  Transform<To, From> inverse() const {
    return Transform<To, From>(transform_.inverse());
  }

  const SE3& value() const noexcept {
    return transform_;
  }

private:
  SE3 transform_;
};

template <class A, class B, class C>
Transform<A, C> compose(const Transform<B, C>& outer, const Transform<A, B>& inner) {
  return Transform<A, C>(outer.value().compose(inner.value()));
}

// 非消费式历史插值使用；不允许区间外推。
SE3 interpolate(const SE3& first, const SE3& second, double fraction);
} // namespace autoaim::math
