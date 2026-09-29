#pragma once

#include "autoaim/core/types.hpp"
#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace autoaim::vision {
enum class TeamColor { red, blue, extinguished, purple };
enum class TargetCategory {
  unknown, one, two, three, four, five, sentry, outpost, base, not_armor
};
enum class ArmorSize { unknown, small, big };

struct ArmorLabel {
  TeamColor color;
  TargetCategory category;
  ArmorSize size;
};

// 指定参考模型的标签字典，不是任意同名 YOLO 模型的通用编号。
ArmorLabel yolov5_label(int color_id, int number_id);
ArmorLabel yolo11_label(int raw_class_id);
TargetCategory parse_category(const std::string& name);
ArmorSize parse_armor_size(const std::string& name);
const char* category_name(TargetCategory category);
const char* armor_size_name(ArmorSize size);

struct Detection {
  // 模型候选顺序；经 CornerMapping 后为物理 TL,TR,BR,BL。图像排序不证明物理语义。
  std::array<cv::Point2f, 4> corners;
  // 传统路径为灯条 alignment，YOLOv5 为 sigmoid(objectness)，YOLO11 为最大类分数。
  // 各路径的启发式分数并非同一种已校准概率，不能直接解释为命中概率。
  float confidence;
  TeamColor color;
  int raw_class_id; // 仅用于对应模型诊断/旧配置兼容，不能参与跨模型关联。
  bool corners_reliable;
  TargetCategory category = TargetCategory::unknown;
  ArmorSize armor_size = ArmorSize::unknown;
};

struct DetectionTiming {
  bool measured = false;
  double preprocessing_ms = 0;
  double inference_wall_ms = 0; // 包含 OpenVINO 融合预处理；异步含完成轮询等待，非内核用时。
  double postprocessing_ms = 0;
};

struct DetectionBatch {
  const core::Stamp source;
  const std::vector<Detection> detections;
  const DetectionTiming timing;

  DetectionBatch(core::Stamp stamp, std::vector<Detection> candidates, DetectionTiming cost = {})
      : source(stamp), detections(std::move(candidates)), timing(cost) {
  }
};
} // namespace autoaim::vision
