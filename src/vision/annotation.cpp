#include "autoaim/vision/annotation.hpp"
#include <set>

namespace autoaim::vision {
void validate_annotation_corners(const std::array<cv::Point2f, 4>& corners) {
  double twice_area = 0;
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const auto& point = corners[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
      throw std::invalid_argument("Nonfinite annotation corner");
    for (std::size_t j = 0; j < i; ++j)
      if (point == corners[j])
        throw std::invalid_argument("Repeated annotation corner");
    const auto& next = corners[(i + 1) % corners.size()];
    twice_area += double(point.x) * next.y - double(point.y) * next.x;
  }

  if (!std::isfinite(twice_area) || twice_area == 0)
    throw std::invalid_argument("Degenerate annotation polygon");
}

std::vector<ArmorAnnotation> read_annotations(const YAML::Node& node, cv::Size image_size) {
  if (!node.IsSequence() || image_size.width <= 0 || image_size.height <= 0)
    throw std::invalid_argument("Annotations require an explicit sequence and image dimensions");

  std::vector<ArmorAnnotation> result;
  std::set<std::string> identities;
  for (const auto& item : node) {
    const auto color = item["color"].as<std::string>();
    if (color != "red" && color != "blue" && color != "extinguished" && color != "purple")
      throw std::invalid_argument("Unknown annotation color");

    ArmorAnnotation annotation{
        {color == "red" ? TeamColor::red : color == "blue" ? TeamColor::blue :
         color == "purple" ? TeamColor::purple : TeamColor::extinguished,
         parse_category(item["category"].as<std::string>()),
         parse_armor_size(item["plate_type"].as<std::string>())}, {}, {}, {}, std::nullopt};
    const auto points = item["corners"].as<std::vector<std::vector<float>>>();
    const auto visible = item["visible"].as<std::vector<bool>>();

    if (points.size() != 4 || visible.size() != 4)
      throw std::invalid_argument("Annotation requires four ordered corners and visibility flags");

    for (int i = 0; i < 4; ++i) {
      if (points[i].size() != 2 || !std::isfinite(points[i][0]) || !std::isfinite(points[i][1]) ||
          (visible[i] && (points[i][0] < 0 || points[i][1] < 0 ||
                         points[i][0] >= image_size.width || points[i][1] >= image_size.height)))
        throw std::invalid_argument("Invalid/visible-outside-image annotation corner");
      annotation.corners[i] = {points[i][0], points[i][1]};
      annotation.visible[i] = visible[i];
    }

    validate_annotation_corners(annotation.corners);

    if (item["track_id"]) {
      annotation.track_id = item["track_id"].as<std::string>();
      if (annotation.track_id.empty() || !identities.insert(annotation.track_id).second)
        throw std::invalid_argument("Empty/repeated frame annotation identity");
    }

    if (item["pose_truth"]) {
      const auto pose = item["pose_truth"];
      const auto p = pose["position_camera_m"].as<std::vector<double>>();
      const auto q = pose["rotation_camera_wxyz"].as<std::vector<double>>();
      const auto id = pose["reference_id"].as<std::string>();
      const double position = pose["position_uncertainty_m"].as<double>();
      const double rotation = pose["rotation_uncertainty_rad"].as<double>();

      if (p.size() != 3 || q.size() != 4 || id.empty() ||
          !std::isfinite(position) || position < 0 || !std::isfinite(rotation) || rotation < 0)
        throw std::invalid_argument("Incomplete pose truth provenance/uncertainty");
      annotation.pose.emplace(AnnotatedPose{
          math::SE3(Eigen::Quaterniond(q[0], q[1], q[2], q[3]), {p[0], p[1], p[2]}),
          id, position, rotation});
    }
    result.push_back(std::move(annotation));
  }

  return result;
}
} // namespace autoaim::vision
