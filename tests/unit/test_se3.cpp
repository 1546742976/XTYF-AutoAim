#include "autoaim/math/se3.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::math;
  return test::run([] {
    const Eigen::Vector3d tangent(0.7, -1.1, 0.3);
    const auto q = rotation_exp(tangent);
    CHECK(rotation_log(q).isApprox(tangent, 1e-10));
    auto negative = q;
    negative.coeffs() *= -1;
    CHECK(rotation_log(negative).isApprox(tangent, 1e-10));
    CHECK(rotation_log(rotation_exp(Eigen::Vector3d::Zero())).norm() == 0);
    const SE3 pose(q, {1, 2, 3});
    const Eigen::Vector3d point(4, 5, 6);
    CHECK(pose.inverse().apply(pose.apply(point)).isApprox(point, 1e-12));
    CHECK(pose.compose(pose.inverse()).apply(point).isApprox(point, 1e-12));
    CHECK_THROWS(std::invalid_argument, checked_rotation(Eigen::Quaterniond(0, 0, 0, 0)));
    CHECK_THROWS(std::invalid_argument, pose.apply({NAN, 0, 0}));
  });
}
