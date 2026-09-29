#pragma once

#include "autoaim/vision/detection.hpp"
#include "autoaim/math/se3.hpp"
#include <yaml-cpp/yaml.h>
#include <optional>

namespace autoaim::vision {
struct AnnotatedPose {
  math::SE3 plate_to_camera;
  std::string reference_id;
  double position_uncertainty_m;
  double rotation_uncertainty_rad;
};

struct ArmorAnnotation {
  ArmorLabel label;
  std::array<cv::Point2f, 4> corners; // 物理 TL,TR,BR,BL；不可见点仍须提供几何位置。
  std::array<bool, 4> visible;
  std::string track_id; // 可空；非空时在序列内标识同一块物理板，仅用于评测。
  std::optional<AnnotatedPose> pose;
};

// 仅解释标注，不产生运行时能力证据。[] 表示人工确认无目标，缺失节点不是 []。
std::vector<ArmorAnnotation> read_annotations(const YAML::Node& node, cv::Size image_size);
} // namespace autoaim::vision
