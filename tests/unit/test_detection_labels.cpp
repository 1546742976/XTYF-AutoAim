#include "autoaim/vision/detection.hpp"
#include "test_support.hpp"

int main() {
  using namespace autoaim::vision;

  return test::run([] {
    CHECK(yolov5_label(1, 0).category == TargetCategory::sentry);
    CHECK(yolov5_label(0, 1).category == TargetCategory::one);
    CHECK(yolov5_label(0, 1).size == ArmorSize::unknown);
    CHECK(yolov5_label(1, 3).category == yolo11_label(10).category);
    CHECK(yolo11_label(10).category == yolo11_label(30).category);
    CHECK(yolo11_label(10).size == ArmorSize::small);
    CHECK(yolo11_label(30).size == ArmorSize::big);
    CHECK(yolo11_label(24).color == TeamColor::purple);
    CHECK(yolo11_label(28).size == ArmorSize::small);

    for (int i = 0; i < 38; ++i) {
      const auto label = yolo11_label(i);
      CHECK(parse_category(category_name(label.category)) == label.category);
      CHECK(parse_armor_size(armor_size_name(label.size)) == label.size);
    }

    CHECK_THROWS(std::invalid_argument, yolov5_label(4, 1));
    CHECK_THROWS(std::invalid_argument, yolo11_label(38));
    CHECK_THROWS(std::invalid_argument, parse_armor_size("large"));
  });
}
