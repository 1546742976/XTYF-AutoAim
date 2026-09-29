#include "autoaim/vision/corner_refine.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;

  return test::run([] {
    const auto evidence = core::Evidence::from_report(core::MeasurementReport::evaluate(
        {"synthetic-corners", "rotation-fixture", "2026-09-29", "known projected labels"}, {0, 0},
        0.1, core::ClockDomain::replay));

    vision::CornerMapping mapping({0, 3, 2, 1}, "synthetic-corners", "rotation-fixture", evidence);

    // 旋转后的 TL 比 BL 更靠右；不能按图像 x/y 重新编号。
    const vision::Detection raw{
        {cv::Point2f(30, 10), {10, 10}, {10, 50}, {30, 50}}, 1, vision::TeamColor::red, 3, false};

    const auto mapped = mapping.apply(raw, core::ClockDomain::replay);
    CHECK(mapped.corners[0] == raw.corners[0] && mapped.corners[1] == raw.corners[3]);
    CHECK(mapped.corners_reliable);
    CHECK(!mapping.apply(raw, core::ClockDomain::host_monotonic).corners_reliable);
    vision::CornerMapping unverified({0, 3, 2, 1}, "real-model", "labels",
                                     core::Evidence::declared());

    CHECK(!unverified.apply(raw, core::ClockDomain::replay).corners_reliable);
    CHECK_THROWS(std::invalid_argument, vision::CornerMapping({0, 0, 2, 3}, "a", "b", evidence));
  });
}
