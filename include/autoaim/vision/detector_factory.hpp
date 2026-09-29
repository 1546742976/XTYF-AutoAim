#pragma once

#include "autoaim/vision/detector.hpp"
#include <variant>

namespace autoaim::vision {
using DetectorOptions = std::variant<TraditionalOptions, Yolov5Options>;

// 构造失败显式抛出配置/后端错误，不悄悄替换检测算法。
std::unique_ptr<Detector> make_detector(const DetectorOptions& options);
} // namespace autoaim::vision
