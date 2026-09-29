#pragma once

#include "autoaim/vision/detection.hpp"
#include "autoaim/core/evidence.hpp"
#include "autoaim/core/image.hpp"

namespace autoaim::vision {
class CornerMapping {
public:
  // indices[k] 是检测器输出数组中物理 TL/TR/BR/BL 对应的索引。
  // model_id + mapping_id 绑定模型与标签约定，不从像素上下关系猜语义。
  CornerMapping(std::array<std::size_t, 4> indices, std::string model_id,
                std::string mapping_id, core::Evidence evidence);
  Detection apply(const Detection& detection, core::ClockDomain domain) const;
private:
  std::array<std::size_t, 4> indices_;
  std::string model_id_;
  std::string mapping_id_;
  core::Evidence evidence_;
};

struct RefinementOptions {
  int brightness_threshold;
  int color_difference;
  double search_radius_px;
  double maximum_shift_px;
  std::size_t minimum_pixels;
};
struct RefinementResult {
  Detection detection;
  bool refined;
};
// 只读原图；以已给定的左/右灯条端点为局部约束，不重排物理角点。
// 任一灯条失败都返回全部原角点并将可靠性降为 false。
RefinementResult refine_corners(const core::Image& image, const Detection& detection,
                                const RefinementOptions& options);
}  // namespace autoaim::vision
