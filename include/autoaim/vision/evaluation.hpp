#pragma once

#include "autoaim/vision/annotation.hpp"
#include <map>

namespace autoaim::vision {
struct EvaluatedDetection {
  Detection detection; // 已按配置转换角点顺序，来自实际处理链而非评测重跑的检测结果。
  std::optional<math::SE3> plate_to_camera;
};

struct PoseTruthLimits {
  std::string accepted_reference_id; // 使用者明确认可的独立参考；不是标注内 passed 布尔值。
  double maximum_position_uncertainty_m;
  double maximum_rotation_uncertainty_rad;
};

struct EvaluationOptions {
  double minimum_iou;
  std::optional<PoseTruthLimits> pose_truth{};
};

struct FrameEvaluation {
  std::size_t truths = 0;
  std::size_t predictions = 0;
  std::size_t matched = 0;
  std::size_t semantic_matched = 0;
  std::size_t category_errors = 0;
  std::size_t color_errors = 0;
  std::size_t size_errors = 0;
  std::size_t visible_corners = 0;
  double corner_error_sum_px = 0;
  double corner_error_squared_px = 0;
  double maximum_corner_error_px = 0;
  std::size_t pose_pairs = 0;
  double position_error_sum_m = 0;
  double rotation_error_sum_rad = 0;
  std::vector<bool> truth_matched;
};

// 轴对齐四角包围框 IoU；按 IoU 降序、真值索引、预测索引稳定贪心一对一匹配。
// 几何 TP 与语义 TP 分开。类别/板型 unknown 标注不计该项错误；可见角点才计误差。
FrameEvaluation evaluate_frame(const std::vector<ArmorAnnotation>& truth,
                                const std::vector<EvaluatedDetection>& predictions,
                                const EvaluationOptions& options);

enum class InterventionGroup : std::size_t { unknown, manual, none, count };

struct EvaluationFrameState {
  bool detector_ran;
  bool accepted_by_pipeline;
  bool expired;
};

struct SequenceTotals {
  std::size_t frames = 0;
  std::size_t unlabeled_frames = 0;
  std::size_t negative_frames = 0;
  std::size_t processed_frames = 0;
  std::size_t rejected_or_dropped_frames = 0;
  std::size_t expired_frames = 0;
  std::size_t frames_with_misses = 0;
  std::size_t usable_matches = 0;
  FrameEvaluation detections; // 不保留逐帧匹配向量。
};

// 帧号严格递增，跳帧允许；未知标注打断连续漏检统计，不能冒充无目标或漏检。
// detector 指标保留过期推理的结果，usable_matches 仅计实际管线接纳的结果。
class SequenceEvaluation {
public:
  explicit SequenceEvaluation(EvaluationOptions options);
  void add(core::FrameId id, const std::optional<std::vector<ArmorAnnotation>>& truth,
           const std::vector<EvaluatedDetection>& predictions, EvaluationFrameState state,
           InterventionGroup intervention);
  const SequenceTotals& totals() const noexcept;
  const std::array<SequenceTotals, 3>& by_intervention() const noexcept;
  std::size_t maximum_empty_detection_streak() const noexcept;
  std::size_t maximum_track_loss_streak() const noexcept;

private:
  EvaluationOptions options_;
  core::FrameId last_id_ = 0;
  SequenceTotals total_;
  std::array<SequenceTotals, 3> groups_{};
  std::size_t empty_streak_ = 0;
  std::size_t maximum_empty_streak_ = 0;
  std::size_t maximum_track_streak_ = 0;
  std::map<std::string, std::size_t> tracks_;
};
} // namespace autoaim::vision
