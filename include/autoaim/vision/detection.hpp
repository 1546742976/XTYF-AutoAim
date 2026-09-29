#pragma once

#include "autoaim/core/types.hpp"
#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace autoaim::vision {
enum class TeamColor { red, blue };
struct Detection {
  std::array<cv::Point2f, 4> corners;  // TL,TR,BR,BL；可靠性说明物理语义是否已证明。
  float confidence;
  TeamColor color;
  int class_id;                      // -1 表示未分类，不能冒充物理板身份。
  bool corners_reliable;
};
struct DetectionBatch {
  const core::Stamp source;
  const std::vector<Detection> detections;
  DetectionBatch(core::Stamp stamp, std::vector<Detection> candidates)
      : source(stamp), detections(std::move(candidates)) {}
};
}  // namespace autoaim::vision
