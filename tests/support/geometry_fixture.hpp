#pragma once

#include "autoaim/estimation/geometry_model.hpp"
#include "autoaim/math/angle.hpp"

namespace test {
inline autoaim::estimation::GeometryProfile geometry(std::size_t count,
    autoaim::estimation::HeightLayout layout, bool tilted) {
  using namespace autoaim;
  std::vector<estimation::PlateGeometry> plates;
  for (std::size_t i = 0; i < count; ++i) {
    const double angle = 2 * math::pi * i / count;
    const double height = layout == estimation::HeightLayout::same ? 0 :
      layout == estimation::HeightLayout::paired ? 0.04 * (i % 2) : 0.04 * i;
    const auto yaw = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d mounting;
    mounting.col(0) = Eigen::Vector3d(0, 1, 0);
    mounting.col(1) = Eigen::Vector3d(0, 0, 1);
    mounting.col(2) = Eigen::Vector3d(1, 0, 0);
    plates.push_back({{0.25 * std::cos(angle), 0.25 * std::sin(angle), height},
      Eigen::Quaterniond(yaw) * Eigen::Quaterniond(mounting),
      vision::PlateDimensions(core::Metres(0.135), core::Metres(0.055))});
  }
  const auto evidence = core::Evidence::from_report(core::MeasurementReport::evaluate(
    {"synthetic-geometry", "v1", "2026-09-29", "known profile"}, {0, 0}, 0.001, core::ClockDomain::replay));
  return estimation::GeometryProfile("synthetic-geometry", layout, count,
    Eigen::Quaterniond(Eigen::AngleAxisd(tilted ? 0.4 : 0, Eigen::Vector3d::UnitY())), plates, evidence);
}
}  // namespace test
