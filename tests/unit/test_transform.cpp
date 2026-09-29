#include "autoaim/math/transform.hpp"
#include "autoaim/math/angle.hpp"
#include "test_support.hpp"
#include <type_traits>

int main() {
  using namespace autoaim::math;
  static_assert(!std::is_convertible_v<Point3<CameraFrame>, Point3<WorldFrame>>);

  return test::run([] {
    const Transform<CameraFrame, GimbalFrame> first(SE3(rotation_exp({0.2, 0.3, 0}), {0.1, 0, 0}));
    const Transform<GimbalFrame, WorldFrame> second(SE3(rotation_exp({0, 0.5, 1}), {1, 2, 3}));
    const Point3<CameraFrame> point({0.4, 0.6, 5});
    const auto combined = compose(second, first);
    CHECK(combined.apply(point).metres().isApprox(second.apply(first.apply(point)).metres()));
    CHECK(combined.inverse().apply(combined.apply(point)).metres().isApprox(point.metres()));
    const auto middle = interpolate(SE3(rotation_exp({0, 0, pi - 0.1}), {0, 0, 0}),
                                    SE3(rotation_exp({0, 0, -pi + 0.1}), {2, 0, 0}), 0.5);

    CHECK_NEAR(std::abs(rotation_log(middle.rotation()).z()), pi, 1e-10);
    CHECK_NEAR(middle.translation().x(), 1, 1e-12);
    CHECK_THROWS(std::invalid_argument, interpolate(first.value(), first.value(), 1.1));
  });
}
