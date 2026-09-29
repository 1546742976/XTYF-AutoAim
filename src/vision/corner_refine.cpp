#include "autoaim/vision/corner_refine.hpp"
#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace autoaim::vision {
CornerMapping::CornerMapping(std::array<std::size_t, 4> indices, std::string model_id,
    std::string mapping_id, core::Evidence evidence)
    : indices_(indices), model_id_(std::move(model_id)), mapping_id_(std::move(mapping_id)),
      evidence_(std::move(evidence)) {
  auto sorted = indices_;
  std::sort(sorted.begin(), sorted.end());
  if (sorted != std::array<std::size_t, 4>{0, 1, 2, 3} || model_id_.empty() || mapping_id_.empty())
    throw std::invalid_argument("Corner mapping must be a named permutation of four indices");
}
Detection CornerMapping::apply(const Detection& detection, core::ClockDomain domain) const {
  auto result = detection;
  bool finite = true;
  for (std::size_t i = 0; i < 4; ++i) {
    result.corners[i] = detection.corners[indices_[i]];
    finite = finite && std::isfinite(result.corners[i].x) && std::isfinite(result.corners[i].y);
  }
  result.corners_reliable = finite && evidence_.qualifies(domain, model_id_, mapping_id_);
  return result;
}

RefinementResult refine_corners(const core::Image& image, const Detection& original,
                                const RefinementOptions& options) {
  if (options.brightness_threshold < 0 || options.brightness_threshold > 255 ||
      options.color_difference < 0 || options.color_difference > 255 ||
      !std::isfinite(options.search_radius_px) || options.search_radius_px <= 0 ||
      !std::isfinite(options.maximum_shift_px) || options.maximum_shift_px <= 0 ||
      options.minimum_pixels < 3)
    throw std::invalid_argument("Invalid corner refinement limits");
  auto result = original;
  const auto failed = [&] { auto unchanged = original; unchanged.corners_reliable = false;
                           return RefinementResult{unchanged, false}; };
  for (const auto& p : original.corners)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 ||
        p.x >= image.width || p.y >= image.height) return failed();
  if (original.color != TeamColor::red && original.color != TeamColor::blue) return failed();
  const int channel = original.color == TeamColor::red ? 2 : 0;
  for (const auto& pair : {std::pair<int, int>{0, 3}, {1, 2}}) {
    const auto a = original.corners[pair.first];
    const auto b = original.corners[pair.second];
    const double length = cv::norm(b - a);
    if (length < 1) return failed();
    const cv::Point2f direction = (b - a) / length;
    const double radius = options.search_radius_px;
    const int left = static_cast<int>(std::max(0.0, std::floor(std::min(a.x, b.x) - radius)));
    const int right = static_cast<int>(std::min(double(image.width - 1), std::ceil(std::max(a.x, b.x) + radius)));
    const int top = static_cast<int>(std::max(0.0, std::floor(std::min(a.y, b.y) - radius)));
    const int bottom = static_cast<int>(std::min(double(image.height - 1), std::ceil(std::max(a.y, b.y) + radius)));
    std::vector<cv::Point2f> points;
    for (int y = top; y <= bottom; ++y) for (int x = left; x <= right; ++x) {
      const auto* pixel = image.pixels->data() + y * image.stride + x * 3;
      const cv::Point2f delta = cv::Point2f(float(x), float(y)) - a;
      const double along = delta.dot(direction);
      const double across = std::abs(delta.x * direction.y - delta.y * direction.x);
      if (along < -radius || along > length + radius || across > radius) continue;
      if (pixel[channel] >= options.brightness_threshold &&
          int(pixel[channel]) - int(pixel[2 - channel]) >= options.color_difference)
        points.emplace_back(float(x), float(y));
    }
    if (points.size() < options.minimum_pixels) return failed();
    cv::Vec4f fit;
    cv::fitLine(points, fit, cv::DIST_HUBER, 0, 0.01, 0.01);
    cv::Point2f axis(fit[0], fit[1]), center(fit[2], fit[3]);
    if (axis.dot(direction) < 0) axis = -axis;
    if (axis.dot(direction) < 0.9) return failed();
    double first = std::numeric_limits<double>::infinity(), last = -first;
    for (const auto& p : points) {
      const double projection = (p - center).dot(axis);
      first = std::min(first, projection); last = std::max(last, projection);
    }
    const auto refined_a = center + axis * first;
    const auto refined_b = center + axis * last;
    // 有限局部修正，不能将邻近另一块板的灯条吸入当前观测。
    if (cv::norm(refined_a - a) > options.maximum_shift_px ||
        cv::norm(refined_b - b) > options.maximum_shift_px || last - first < 1) return failed();
    result.corners[pair.first] = refined_a;
    result.corners[pair.second] = refined_b;
  }
  return {result, true};
}
}  // namespace autoaim::vision
