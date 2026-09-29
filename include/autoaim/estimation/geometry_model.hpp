#pragma once

#include "autoaim/vision/pnp.hpp"
#include "autoaim/core/config.hpp"

namespace autoaim::estimation {
enum class HeightLayout { same, paired, distinct };

struct PlateGeometry {
  Eigen::Vector3d offset_axis_m;

  // 物理板系到相位零点轴系的完整安装旋转；法向由此给出，不由偏移猜测。
  Eigen::Quaterniond plate_to_axis;
  vision::PlateDimensions dimensions;
};

struct GeometryProfile {
  const std::string id;
  const HeightLayout layout;

  // 显式给出三根基轴，固定横向参考、相位零点和右手正旋，不仅是转轴方向。
  const Eigen::Quaterniond axis_to_world;
  const std::vector<PlateGeometry> plates;
  const core::Evidence evidence;

  GeometryProfile(std::string profile_id, HeightLayout heights, std::size_t expected_count,
                  Eigen::Quaterniond basis, std::vector<PlateGeometry> offsets,
                  core::Evidence calibration_evidence);

  math::Transform<math::PlateFrame, math::WorldFrame>
  plate_pose(const math::Point3<math::WorldFrame>& center, core::Radians phase,
             std::size_t index) const;

  Eigen::Vector3d normal_world(core::Radians phase, std::size_t index) const;
};

// paired 精确定义为四板、两组各两板；三板不能被静默凑成两对。
core::Result<GeometryProfile> load_geometry(const core::Config& config);
} // namespace autoaim::estimation
