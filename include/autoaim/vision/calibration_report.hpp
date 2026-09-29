#pragma once

#include "autoaim/vision/calibration_solver.hpp"

namespace autoaim::vision {
enum class CalibrationCapability { intrinsics, extrinsics };

// 报告绑定拟合参数、样本划分、方法与独立验证残差，不写入可直接授予能力的布尔值。
YAML::Node intrinsic_report(const CalibrationDataset& data, const IntrinsicSolution& solution,
                            core::MeasurementProvenance provenance, core::ClockDomain domain,
                            double maximum_rms_px);
YAML::Node hand_eye_report(const CalibrationDataset& data, const IntrinsicSolution& intrinsic,
                           const HandEyeSolution& solution, core::MeasurementProvenance provenance,
                           core::ClockDomain domain, double maximum_rotation_rad,
                           double maximum_translation_m);

// 参数/能力不匹配是输入错误；未验证或残差超限返回 missing。忽略用户填写的 passed。
// 文件不证明样本真实性，仍须人工核对；simulation 永远不满足 host_monotonic 能力。
core::Evidence calibration_report_evidence(const core::Config& report, const Calibration& binding,
                                           CalibrationCapability capability);
} // namespace autoaim::vision
