#pragma once

#include "autoaim/estimation/observation.hpp"
#include "autoaim/estimation/geometry_model.hpp"
#include "autoaim/estimation/motion_model.hpp"

namespace autoaim::estimation {
enum class MeasurementKind { position, full_pose };

struct LinearizedMeasurement {
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd noise;
};

class MeasurementModel {
public:
  explicit MeasurementModel(MeasurementKind kind);
  int dimension() const noexcept;
  math::Transform<math::PlateFrame, math::WorldFrame> predict(const TargetState& state,
                                                              const GeometryProfile& geometry,
                                                              std::size_t physical_plate) const;

  // 残差顺序为位置(m)、可选 SO(3) 左乘旋转向量(rad)，不用欧拉角相减。
  // 关联必须先选定明确的候选/身份假设；原 Observation 始终保留全部候选。
  core::Result<LinearizedMeasurement> linearize(const TargetState& state,
                                                const GeometryProfile& geometry,
                                                std::size_t physical_plate,
                                                const ObservedPose& observation) const;

private:
  MeasurementKind kind_;
};
} // namespace autoaim::estimation
