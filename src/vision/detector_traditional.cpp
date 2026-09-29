#include "autoaim/vision/detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace autoaim::vision {
TraditionalDetector::TraditionalDetector(TraditionalOptions options) : options_(options) {
  if ((options.enemy != TeamColor::red && options.enemy != TeamColor::blue) ||
      options.brightness_threshold < 0 || options.brightness_threshold > 255 ||
      options.color_difference < 0 || options.color_difference > 255 ||
      !std::isfinite(options.minimum_light_length) || options.minimum_light_length <= 0 ||
      !std::isfinite(options.minimum_light_ratio) || options.minimum_light_ratio <= 1 ||
      !std::isfinite(options.minimum_pair_ratio) || options.minimum_pair_ratio <= 0 ||
      !std::isfinite(options.maximum_pair_ratio) || options.maximum_pair_ratio < options.minimum_pair_ratio ||
      options.maximum_lightbars < 2) throw std::invalid_argument("Invalid traditional detector options");
}

DetectionBatch TraditionalDetector::detect(const FramePacket& packet) {
  const auto& image = packet.frame.image;
  // OpenCV 的借用视图仅作输入，租约在整个同步调用中持有，不修改原图。
  const cv::Mat bgr(image.height, image.width, CV_8UC3,
    const_cast<std::uint8_t*>(image.pixels->data()), image.stride);
  std::vector<cv::Mat> channels;
  cv::split(bgr, channels);
  const int enemy = options_.enemy == TeamColor::red ? 2 : 0;
  const int other = enemy == 2 ? 0 : 2;
  cv::Mat contrast, bright, mask;
  cv::subtract(channels[enemy], channels[other], contrast);
  cv::threshold(contrast, contrast, options_.color_difference, 255, cv::THRESH_BINARY);
  cv::threshold(channels[enemy], bright, options_.brightness_threshold, 255, cv::THRESH_BINARY);
  cv::bitwise_and(contrast, bright, mask);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  struct Light { cv::Point2f center; cv::Point2f direction; float length; };
  std::vector<Light> lights;
  for (const auto& contour : contours) {
    if (contour.size() < 4) continue;
    const auto rectangle = cv::minAreaRect(contour);
    const float length = std::max(rectangle.size.width, rectangle.size.height);
    const float width = std::min(rectangle.size.width, rectangle.size.height);
    if (width <= 0 || length < options_.minimum_light_length || length / width < options_.minimum_light_ratio) continue;
    cv::Point2f corners[4]; rectangle.points(corners);
    auto direction = corners[1] - corners[0];
    if (cv::norm(direction) < length - 1e-3) direction = corners[2] - corners[1];
    direction *= 1.0f / length;
    // 兼容路径只提供图像排序，不能证明物理 TL/TR；可靠性始终为 false。
    if (direction.y < 0 || (direction.y == 0 && direction.x < 0)) direction *= -1;
    lights.push_back({rectangle.center, direction, length});
  }
  std::stable_sort(lights.begin(), lights.end(), [](const auto& a, const auto& b) { return a.length > b.length; });
  if (lights.size() > options_.maximum_lightbars) lights.resize(options_.maximum_lightbars);
  std::sort(lights.begin(), lights.end(), [](const auto& a, const auto& b) { return a.center.x < b.center.x; });
  std::vector<Detection> detections;
  for (std::size_t left = 0; left < lights.size(); ++left) {
    for (std::size_t right = left + 1; right < lights.size(); ++right) {
      const auto& a = lights[left]; const auto& b = lights[right];
      const float mean_length = (a.length + b.length) / 2;
      const auto separation = b.center - a.center;
      const double distance = cv::norm(separation);
      if (distance <= 0) continue;
      const double alignment = std::abs(a.direction.dot(b.direction));
      const double ratio = distance / mean_length;
      if (alignment < 0.9 || std::min(a.length, b.length) / std::max(a.length, b.length) < 0.5 ||
          ratio < options_.minimum_pair_ratio || ratio > options_.maximum_pair_ratio ||
          std::abs(separation.dot(a.direction)) / distance > 0.5) continue;
      detections.push_back({{a.center - a.direction * (a.length / 2), b.center - b.direction * (b.length / 2),
        b.center + b.direction * (b.length / 2), a.center + a.direction * (a.length / 2)},
        static_cast<float>(alignment), options_.enemy, -1, false});
    }
  }
  return DetectionBatch(packet.frame.stamp, std::move(detections));
}
}  // namespace autoaim::vision
