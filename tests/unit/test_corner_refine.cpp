#include "autoaim/vision/corner_refine.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim;
  return test::run([] {
    auto bytes = std::make_shared<std::vector<std::uint8_t>>(100 * 80 * 3, 0);
    for (int y = 20; y <= 50; ++y) for (int x : {19, 20, 21, 69, 70, 71})
      (*bytes)[(y * 100 + x) * 3 + 2] = 255;
    const core::Image image(100, 80, 300, bytes);
    const vision::Detection original{{cv::Point2f(21, 21), {71, 21}, {71, 49}, {21, 49}},
      1, vision::TeamColor::red, 3, true};
    const vision::RefinementOptions options{100, 50, 4, 4, 10};
    const auto refined = vision::refine_corners(image, original, options);
    CHECK(refined.refined && refined.detection.corners_reliable);
    CHECK_NEAR(refined.detection.corners[0].x, 20, 0.1);
    CHECK_NEAR(refined.detection.corners[0].y, 20, 0.1);
    // 没有对应灯条时保留输入，不生成假精修或恢复可靠性。
    auto blue = original; blue.color = vision::TeamColor::blue;
    const auto failed = vision::refine_corners(image, blue, options);
    CHECK(!failed.refined && !failed.detection.corners_reliable);
    CHECK(failed.detection.corners == original.corners);
    auto unknown = original; unknown.corners_reliable = false;
    CHECK(!vision::refine_corners(image, unknown, options).detection.corners_reliable);
    auto rolled = original;
    rolled.corners = {original.corners[2], original.corners[3], original.corners[0], original.corners[1]};
    const auto reverse = vision::refine_corners(image, rolled, options);
    CHECK(reverse.refined && reverse.detection.corners[0].y > reverse.detection.corners[3].y);
  });
}
