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

#if !AUTOAIM_I1_CONTRAST_IRLS
RefinementResult refine_corners(const core::Image& image, const Detection& original,
                                const RefinementOptions& options) {
  if (options.brightness_threshold < 0 || options.brightness_threshold > 255 ||
      options.color_difference < 0 || options.color_difference > 255 ||
      !std::isfinite(options.search_radius_px) || options.search_radius_px <= 0 ||
      !std::isfinite(options.maximum_shift_px) || options.maximum_shift_px <= 0 ||
      options.minimum_pixels < 3)
    throw std::invalid_argument("Invalid corner refinement limits");

  auto result = original;
  const auto failed = [&] {
    auto unchanged = original;
    unchanged.corners_reliable = false;

    return RefinementResult{unchanged, false};
  };

  for (const auto& p : original.corners)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 || p.x >= image.width ||
        p.y >= image.height)
      return failed();

  if (original.color != TeamColor::red && original.color != TeamColor::blue)
    return failed();

  const int channel = original.color == TeamColor::red ? 2 : 0;

  for (const auto& pair : {std::pair<int, int>{0, 3}, {1, 2}}) {
    const auto a = original.corners[pair.first];
    const auto b = original.corners[pair.second];
    const double length = cv::norm(b - a);

    if (length < 1)
      return failed();

    const cv::Point2f direction = (b - a) / length;
    const double radius = options.search_radius_px;
    const int left = static_cast<int>(std::max(0.0, std::floor(std::min(a.x, b.x) - radius)));
    const int right =
        static_cast<int>(std::min(double(image.width - 1), std::ceil(std::max(a.x, b.x) + radius)));

    const int top = static_cast<int>(std::max(0.0, std::floor(std::min(a.y, b.y) - radius)));
    const int bottom = static_cast<int>(
        std::min(double(image.height - 1), std::ceil(std::max(a.y, b.y) + radius)));

    std::vector<cv::Point2f> points;

    for (int y = top; y <= bottom; ++y)
      for (int x = left; x <= right; ++x) {
        const auto* pixel = image.pixels->data() + y * image.stride + x * 3;
        const cv::Point2f delta = cv::Point2f(float(x), float(y)) - a;
        const double along = delta.dot(direction);
        const double across = std::abs(delta.x * direction.y - delta.y * direction.x);

        if (along < -radius || along > length + radius || across > radius)
          continue;

        if (pixel[channel] >= options.brightness_threshold &&
            int(pixel[channel]) - int(pixel[2 - channel]) >= options.color_difference)
          points.emplace_back(float(x), float(y));
      }

    if (points.size() < options.minimum_pixels)
      return failed();

    cv::Vec4f fit;
    cv::fitLine(points, fit, cv::DIST_HUBER, 0, 0.01, 0.01);
    cv::Point2f axis(fit[0], fit[1]), center(fit[2], fit[3]);

    if (axis.dot(direction) < 0)
      axis = -axis;

    if (axis.dot(direction) < 0.9)
      return failed();

    double first = std::numeric_limits<double>::infinity(), last = -first;

    for (const auto& p : points) {
      const double projection = (p - center).dot(axis);
      first = std::min(first, projection);
      last = std::max(last, projection);
    }

    const auto refined_a = center + axis * first;
    const auto refined_b = center + axis * last;

    // 有限局部修正，不能将邻近另一块板的灯条吸入当前观测。
    if (cv::norm(refined_a - a) > options.maximum_shift_px ||
        cv::norm(refined_b - b) > options.maximum_shift_px || last - first < 1)
      return failed();

    result.corners[pair.first] = refined_a;
    result.corners[pair.second] = refined_b;
  }

  return {result, true};
}
#else

// I1-CONTRAST-IRLS is selected by its CMake algorithm option.
// Contrast weights refine the existing centerline fit, not the physical endpoint convention.
// No new evidence, calibrated pixel covariance, equal-length or parallel-bar constraint.
RefinementResult refine_corners(const core::Image& image, const Detection& original,
                                const RefinementOptions& options) {
  if (options.brightness_threshold < 0 || options.brightness_threshold > 255 ||
      options.color_difference < 0 || options.color_difference > 255 ||
      !std::isfinite(options.search_radius_px) || options.search_radius_px <= 0 ||
      !std::isfinite(options.maximum_shift_px) || options.maximum_shift_px <= 0 ||
      options.minimum_pixels < 3)
    throw std::invalid_argument("Invalid corner refinement limits");

  auto result = original;
  const auto failed = [&] {
    auto unchanged = original;
    unchanged.corners_reliable = false;
    return RefinementResult{unchanged, false};
  };

  for (const auto& p : original.corners)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0 || p.x >= image.width ||
        p.y >= image.height)
      return failed();

  if (original.color != TeamColor::red && original.color != TeamColor::blue)
    return failed();

  const int channel = original.color == TeamColor::red ? 2 : 0;
  for (const auto& pair : {std::pair<int, int>{0, 3}, {1, 2}}) {
    const auto a = original.corners[pair.first];
    const auto b = original.corners[pair.second];
    const double length = cv::norm(b - a);
    if (length < 1)
      return failed();

    // Keep the original float pixel-selection expressions, including ROI boundary rounding.
    const cv::Point2f direction = (b - a) / length;
    const double radius = options.search_radius_px;
    const int left = static_cast<int>(std::max(0.0, std::floor(std::min(a.x, b.x) - radius)));
    const int right =
        static_cast<int>(std::min(double(image.width - 1), std::ceil(std::max(a.x, b.x) + radius)));
    const int top = static_cast<int>(std::max(0.0, std::floor(std::min(a.y, b.y) - radius)));
    const int bottom = static_cast<int>(
        std::min(double(image.height - 1), std::ceil(std::max(a.y, b.y) + radius)));

    std::vector<cv::Point2f> points;
    std::vector<double> contrast_weights;
    for (int y = top; y <= bottom; ++y)
      for (int x = left; x <= right; ++x) {
        const auto* pixel = image.pixels->data() + y * image.stride + x * 3;
        const cv::Point2f delta = cv::Point2f(float(x), float(y)) - a;
        const double along = delta.dot(direction);
        const double across = std::abs(delta.x * direction.y - delta.y * direction.x);
        if (along < -radius || along > length + radius || across > radius)
          continue;
        const int contrast = int(pixel[channel]) - int(pixel[2 - channel]);
        if (pixel[channel] >= options.brightness_threshold && contrast >= options.color_difference) {
          points.emplace_back(float(x), float(y));
          contrast_weights.push_back(std::max(1, contrast) / 255.0);
        }
      }
    if (points.size() < options.minimum_pixels)
      return failed();

    const bool equal_contrast = std::all_of(contrast_weights.begin(), contrast_weights.end(),
        [&](double weight) { return weight == contrast_weights.front(); });
    cv::Vec4f fit;
    cv::fitLine(points, fit, cv::DIST_HUBER, 0, 0.01, 0.01);
    cv::Point2d axis(fit[0], fit[1]), center(fit[2], fit[3]);
    const auto finite = [](const cv::Point2d& point) {
      return std::isfinite(point.x) && std::isfinite(point.y);
    };
    const double initial_norm = cv::norm(axis);
    if (!finite(axis) || !finite(center) || !std::isfinite(initial_norm) || initial_norm <= 0)
      return failed();
    axis /= initial_norm;
    if (axis.dot(direction) < 0)
      axis = -axis;

    std::vector<double> weights(points.size());
    // The fixed numerical limits are local experimental constants, not calibrated noise.
    constexpr double huber_delta_px = 1.345;
    for (int iteration = 0; iteration < 10; ++iteration) {
      double total = 0;
      cv::Point2d weighted_sum(0, 0);
      for (std::size_t i = 0; i < points.size(); ++i) {
        const cv::Point2d point = points[i];
        const auto offset = point - center;
        const double distance = std::abs(offset.x * axis.y - offset.y * axis.x);
        const double huber = distance <= huber_delta_px ? 1.0 : huber_delta_px / distance;
        weights[i] = contrast_weights[i] * huber;
        total += weights[i];
        weighted_sum += point * weights[i];
      }
      if (!std::isfinite(total) || total <= 0 || !finite(weighted_sum))
        return failed();
      const auto next_center = weighted_sum / total;
      if (!finite(next_center))
        return failed();

      double xx = 0, xy = 0, yy = 0;
      for (std::size_t i = 0; i < points.size(); ++i) {
        const cv::Point2d point = points[i];
        const auto offset = point - next_center;
        xx += weights[i] * offset.x * offset.x;
        xy += weights[i] * offset.x * offset.y;
        yy += weights[i] * offset.y * offset.y;
      }
      xx /= total;
      xy /= total;
      yy /= total;
      const double trace = xx + yy;
      const double eigen_gap = std::hypot(xx - yy, 2 * xy);
      const double largest_eigenvalue = (trace + eigen_gap) / 2;
      if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(yy) ||
          !std::isfinite(largest_eigenvalue) || largest_eigenvalue <= 1e-12 ||
          eigen_gap <= 1e-12 * std::max(1.0, trace))
        return failed();

      // Constant contrast leaves the original Huber objective unchanged. After checking
      // finite, nonzero, directional scatter, keep its existing numerical solution.
      if (equal_contrast)
        break;

      const double theta = 0.5 * std::atan2(2 * xy, xx - yy);
      cv::Point2d next_axis(std::cos(theta), std::sin(theta));
      if (next_axis.dot(direction) < 0)
        next_axis = -next_axis;
      const double angle_change = std::atan2(
          std::abs(axis.x * next_axis.y - axis.y * next_axis.x), axis.dot(next_axis));
      const bool converged = angle_change <= 1e-6 && cv::norm(next_center - center) <= 1e-4;
      axis = next_axis;
      center = next_center;
      if (converged)
        break;
    }

    if (equal_contrast) {
      // Preserve the original float projection and endpoint arithmetic exactly when
      // contrast adds no information; recomputing TLS would only perturb the baseline.
      cv::Point2f original_axis(fit[0], fit[1]), original_center(fit[2], fit[3]);
      if (original_axis.dot(direction) < 0)
        original_axis = -original_axis;
      if (original_axis.dot(direction) < 0.9)
        return failed();
      double first = std::numeric_limits<double>::infinity(), last = -first;
      for (const auto& p : points) {
        const double projection = (p - original_center).dot(original_axis);
        first = std::min(first, projection);
        last = std::max(last, projection);
      }
      const auto refined_a = original_center + original_axis * first;
      const auto refined_b = original_center + original_axis * last;
      if (cv::norm(refined_a - a) > options.maximum_shift_px ||
          cv::norm(refined_b - b) > options.maximum_shift_px || last - first < 1)
        return failed();
      result.corners[pair.first] = refined_a;
      result.corners[pair.second] = refined_b;
      continue;
    }

    if (axis.dot(direction) < 0.9)
      return failed();
    double first = std::numeric_limits<double>::infinity(), last = -first;
    // All originally selected pixels still define the endpoint extent, including low weights.
    for (const auto& p : points) {
      const cv::Point2d point = p;
      const double projection = (point - center).dot(axis);
      first = std::min(first, projection);
      last = std::max(last, projection);
    }
    const auto refined_a = center + axis * first;
    const auto refined_b = center + axis * last;
    if (!finite(refined_a) || !finite(refined_b) ||
        cv::norm(refined_a - cv::Point2d(a)) > options.maximum_shift_px ||
        cv::norm(refined_b - cv::Point2d(b)) > options.maximum_shift_px || last - first < 1)
      return failed();
    result.corners[pair.first] = refined_a;
    result.corners[pair.second] = refined_b;
  }
  return {result, true};
}
#endif
} // namespace autoaim::vision
