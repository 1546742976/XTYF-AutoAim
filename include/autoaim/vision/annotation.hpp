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

// 真值读取和直接评测共用：有限、四点互异且有非零有向面积；不重排角点。
// 图内可见性由读取器单独校验，图外不可见角点仍可合法。
void validate_annotation_corners(const std::array<cv::Point2f, 4>& corners);
} // namespace autoaim::vision
