#include "autoaim/vision/evaluation.hpp"
#include <algorithm>
#include <tuple>

namespace autoaim::vision {
namespace {
cv::Rect2d bounds(const std::array<cv::Point2f, 4>& corners) {
  double x0 = corners[0].x, x1 = x0, y0 = corners[0].y, y1 = y0;

  for (const auto& p : corners) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      throw std::invalid_argument("Nonfinite evaluation corner");
    x0 = std::min(x0, double(p.x));
    x1 = std::max(x1, double(p.x));
    y0 = std::min(y0, double(p.y));
    y1 = std::max(y1, double(p.y));
  }

  return {x0, y0, x1 - x0, y1 - y0};
}
} // namespace

FrameEvaluation evaluate_frame(const std::vector<ArmorAnnotation>& truth,
                                const std::vector<EvaluatedDetection>& predictions,
                                const EvaluationOptions& options) {
  if (!std::isfinite(options.minimum_iou) || options.minimum_iou <= 0 || options.minimum_iou > 1)
    throw std::invalid_argument("Evaluation IoU must be in (0,1]");

  if (options.pose_truth) {
    const auto& p = *options.pose_truth;
    if (p.accepted_reference_id.empty() || !std::isfinite(p.maximum_position_uncertainty_m) ||
        !std::isfinite(p.maximum_rotation_uncertainty_rad) ||
        p.maximum_position_uncertainty_m < 0 || p.maximum_rotation_uncertainty_rad < 0)
      throw std::invalid_argument("Invalid explicit pose truth acceptance limits");
  }

  struct Pair {
    double iou;
    std::size_t truth;
    std::size_t prediction;
  };
  std::vector<Pair> pairs;
  for (std::size_t t = 0; t < truth.size(); ++t) {
    const auto a = bounds(truth[t].corners);
    if (a.area() <= 0)
      throw std::invalid_argument("Degenerate ground truth box");

    for (std::size_t p = 0; p < predictions.size(); ++p) {
      const auto b = bounds(predictions[p].detection.corners);
      const double intersection = (a & b).area();
      const double united = a.area() + b.area() - intersection;
      const double iou = united > 0 ? intersection / united : 0;
      if (iou >= options.minimum_iou)
        pairs.push_back({iou, t, p});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
    if (a.iou != b.iou)
      return a.iou > b.iou;

    return std::tie(a.truth, a.prediction) < std::tie(b.truth, b.prediction);
  });

  FrameEvaluation result;
  result.truths = truth.size();
  result.predictions = predictions.size();
  result.truth_matched.resize(truth.size(), false);
  std::vector<bool> used(predictions.size(), false);

  for (const auto& pair : pairs) {
    if (result.truth_matched[pair.truth] || used[pair.prediction])
      continue;
    result.truth_matched[pair.truth] = true;
    used[pair.prediction] = true;
    ++result.matched;
    const auto& t = truth[pair.truth];
    const auto& p = predictions[pair.prediction];
    const bool category = t.label.category != TargetCategory::unknown &&
                          t.label.category != p.detection.category;
    const bool color = t.label.color != p.detection.color;
    const bool size = t.label.size != ArmorSize::unknown && t.label.size != p.detection.armor_size;
    result.category_errors += category;
    result.color_errors += color;
    result.size_errors += size;
    result.semantic_matched += !category && !color && !size;

    for (int k = 0; k < 4; ++k) {
      if (!t.visible[k])
        continue;
      const double error = cv::norm(t.corners[k] - p.detection.corners[k]);
      ++result.visible_corners;
      result.corner_error_sum_px += error;
      result.corner_error_squared_px += error * error;
      result.maximum_corner_error_px = std::max(result.maximum_corner_error_px, error);
    }

    if (options.pose_truth && t.pose && p.plate_to_camera &&
        t.pose->reference_id == options.pose_truth->accepted_reference_id &&
        t.pose->position_uncertainty_m <= options.pose_truth->maximum_position_uncertainty_m &&
        t.pose->rotation_uncertainty_rad <= options.pose_truth->maximum_rotation_uncertainty_rad) {
      ++result.pose_pairs;
      result.position_error_sum_m +=
          (t.pose->plate_to_camera.translation() - p.plate_to_camera->translation()).norm();
      result.rotation_error_sum_rad += math::rotation_log(
          t.pose->plate_to_camera.rotation().conjugate() * p.plate_to_camera->rotation()).norm();
    }
  }

  return result;
}
} // namespace autoaim::vision
