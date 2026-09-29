#include "autoaim/vision/detection.hpp"

namespace autoaim::vision {
namespace {
constexpr const char* categories[] = {"unknown", "one", "two", "three", "four", "five",
                                      "sentry", "outpost", "base", "not_armor"};
constexpr const char* sizes[] = {"unknown", "small", "big"};
} // namespace

const char* category_name(TargetCategory category) {
  const auto index = static_cast<unsigned>(category);

  if (index >= std::size(categories))
    throw std::invalid_argument("Unknown target category");

  return categories[index];
}

const char* armor_size_name(ArmorSize size) {
  const auto index = static_cast<unsigned>(size);

  if (index >= std::size(sizes))
    throw std::invalid_argument("Unknown armor size");

  return sizes[index];
}

TargetCategory parse_category(const std::string& name) {
  for (unsigned i = 0; i < std::size(categories); ++i)
    if (name == categories[i])
      return static_cast<TargetCategory>(i);

  throw std::invalid_argument("Unknown target category: " + name);
}

ArmorSize parse_armor_size(const std::string& name) {
  for (unsigned i = 0; i < std::size(sizes); ++i)
    if (name == sizes[i])
      return static_cast<ArmorSize>(i);

  throw std::invalid_argument("Unknown armor size: " + name);
}

ArmorLabel yolov5_label(int color_id, int number_id) {
  constexpr TeamColor colors[] = {TeamColor::blue, TeamColor::red,
                                   TeamColor::extinguished, TeamColor::purple};
  constexpr TargetCategory numbers[] = {TargetCategory::sentry, TargetCategory::one,
      TargetCategory::two, TargetCategory::three, TargetCategory::four, TargetCategory::five,
      TargetCategory::outpost, TargetCategory::base, TargetCategory::not_armor};

  if (color_id < 0 || color_id >= 4 || number_id < 0 || number_id >= 9)
    throw std::invalid_argument("YOLOv5 label outside reference dictionary");

  // 该输出没有独立大小板通道；不可凭数字统一断言尺寸。
  return {colors[color_id], numbers[number_id], ArmorSize::unknown};
}

ArmorLabel yolo11_label(int raw_class_id) {
  if (raw_class_id < 0 || raw_class_id >= 38)
    throw std::invalid_argument("YOLO11 label outside 38-class reference dictionary");

  constexpr TeamColor colors[] = {TeamColor::blue, TeamColor::red,
                                   TeamColor::extinguished, TeamColor::purple};
  constexpr TargetCategory small[] = {TargetCategory::sentry, TargetCategory::one,
      TargetCategory::two, TargetCategory::three, TargetCategory::four, TargetCategory::five,
      TargetCategory::outpost};

  if (raw_class_id < 21)
    return {colors[raw_class_id % 3], small[raw_class_id / 3], ArmorSize::small};
  if (raw_class_id < 25)
    return {colors[raw_class_id - 21], TargetCategory::base, ArmorSize::big};
  if (raw_class_id < 29)
    return {colors[raw_class_id - 25], TargetCategory::base, ArmorSize::small};

  constexpr TargetCategory large[] = {TargetCategory::three, TargetCategory::four,
                                       TargetCategory::five};

  return {colors[(raw_class_id - 29) % 3], large[(raw_class_id - 29) / 3], ArmorSize::big};
}
} // namespace autoaim::vision
