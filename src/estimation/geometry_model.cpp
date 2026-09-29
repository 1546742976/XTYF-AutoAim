#include "autoaim/estimation/geometry_model.hpp"
#include <algorithm>

namespace autoaim::estimation {
namespace {
std::vector<PlateGeometry> validate_plates(std::vector<PlateGeometry> plates, HeightLayout layout,
                                           std::size_t count) {
  if (count < 2 || count > 4 || plates.size() != count)
    throw std::invalid_argument("Geometry needs exactly 2/3/4 plates");

  std::vector<double> heights;

  for (auto& plate : plates) {
    if (!plate.offset_axis_m.allFinite() || plate.offset_axis_m.head<2>().norm() <= 0)
      throw std::invalid_argument("Invalid calibrated plate offset");

    plate.plate_to_axis = math::checked_rotation(plate.plate_to_axis);
    heights.push_back(plate.offset_axis_m.z());
  }

  std::sort(heights.begin(), heights.end());
  constexpr double tolerance_m = 1e-6;
  const auto equal = [&](double a, double b) {
    return std::abs(a - b) <= tolerance_m;
  };

  if (layout == HeightLayout::same) {
    if (!equal(heights.front(), heights.back()))
      throw std::invalid_argument("Same-height profile has different heights");
  } else if (layout == HeightLayout::paired) {
    if (count != 4 || !equal(heights[0], heights[1]) || !equal(heights[2], heights[3]) ||
        equal(heights[1], heights[2]))
      throw std::invalid_argument("Paired heights require two distinct pairs of four plates");
  } else if (layout == HeightLayout::distinct) {
    for (std::size_t i = 1; i < heights.size(); ++i)
      if (equal(heights[i - 1], heights[i]))
        throw std::invalid_argument("Distinct-height profile contains equal heights");
  } else
    throw std::invalid_argument("Unknown height layout");

  return plates;
}
} // namespace

GeometryProfile::GeometryProfile(std::string profile_id, HeightLayout heights,
                                 std::size_t expected_count, Eigen::Quaterniond basis,
                                 std::vector<PlateGeometry> offsets,
                                 core::Evidence calibration_evidence)
    : id(std::move(profile_id)), layout(heights), axis_to_world(math::checked_rotation(basis)),
      plates(validate_plates(std::move(offsets), heights, expected_count)),
      evidence(std::move(calibration_evidence)) {
  if (id.empty())
    throw std::invalid_argument("Geometry profile id is required");
}

math::Transform<math::PlateFrame, math::WorldFrame>
GeometryProfile::plate_pose(const math::Point3<math::WorldFrame>& center, core::Radians phase,
                            std::size_t index) const {
  const auto& plate = plates.at(index);
  const auto rotation =
      axis_to_world *
      Eigen::Quaterniond(Eigen::AngleAxisd(phase.value(), Eigen::Vector3d::UnitZ()));

  return math::Transform<math::PlateFrame, math::WorldFrame>(
      math::SE3(rotation * plate.plate_to_axis, center.metres() + rotation * plate.offset_axis_m));
}

Eigen::Vector3d GeometryProfile::normal_world(core::Radians phase, std::size_t index) const {
  return plate_pose(math::Point3<math::WorldFrame>(Eigen::Vector3d::Zero()), phase, index)
             .value()
             .rotation() *
         Eigen::Vector3d::UnitZ();
}

core::Result<GeometryProfile> load_geometry(const core::Config& config) {
  using Result = core::Result<GeometryProfile>;

  try {
    const auto name = config.require<std::string>("geometry.layout");
    const auto layout = name == "same"     ? HeightLayout::same
                        : name == "paired" ? HeightLayout::paired
                        : name == "distinct"
                            ? HeightLayout::distinct
                            : throw std::invalid_argument("Unknown geometry.layout");

    const auto basis = config.require<std::vector<double>>("geometry.axis_to_world_wxyz");

    if (basis.size() != 4)
      throw std::invalid_argument("Geometry basis needs four quaternion components");

    const auto nodes = config.require<std::vector<YAML::Node>>("geometry.plates");
    std::vector<PlateGeometry> plates;

    for (const auto& node : nodes) {
      const auto offset = node["offset_m"].as<std::vector<double>>();
      const auto rotation = node["plate_to_axis_wxyz"].as<std::vector<double>>();

      if (offset.size() != 3 || rotation.size() != 4)
        throw std::invalid_argument("Invalid geometry plate dimensions");

      plates.push_back({{offset[0], offset[1], offset[2]},
                        {rotation[0], rotation[1], rotation[2], rotation[3]},
                        vision::PlateDimensions(core::Metres(node["width_m"].as<double>()),
                                                core::Metres(node["height_m"].as<double>()))});
    }

    return Result::success(GeometryProfile(config.require<std::string>("geometry.id"), layout,
                                           config.require<std::size_t>("geometry.plate_count"),
                                           {basis[0], basis[1], basis[2], basis[3]},
                                           std::move(plates),
                                           config.declared_evidence("geometry.calibrated")));
  } catch (const std::exception& error) {
    return Result::failure(core::ErrorCode::invalid_input, error.what());
  }
}
} // namespace autoaim::estimation
