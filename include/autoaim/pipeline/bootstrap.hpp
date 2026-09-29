#pragma once

#include "autoaim/pipeline/queue.hpp"
#include "autoaim/pipeline/uart_adapter.hpp"
#include "autoaim/vision/detector_factory.hpp"
#include "autoaim/vision/corner_refine.hpp"
#include "autoaim/estimation/tracker.hpp"
#include "autoaim/decision/armor_selector.hpp"
#include "autoaim/decision/hit_probability.hpp"
#include "autoaim/mission/target_selector.hpp"
#include "autoaim/mission/infantry/infantry_mission.hpp"
#include "autoaim/control/command_guard.hpp"
#include "autoaim/control/smoother.hpp"
#include <map>

namespace autoaim::pipeline {
// 全部参数由 bootstrap 显式装配；测试参数只在 config/offline 中提供。
struct PipelineConfig {
  std::filesystem::path input_manifest;
  core::Role role;
  bool program_fire_requested;
  vision::DetectorOptions detector;
  vision::Calibration calibration;
  std::vector<std::shared_ptr<const estimation::GeometryProfile>> profiles;
  std::shared_ptr<const estimation::MotionModel> cv_motion;
  std::shared_ptr<const estimation::MotionModel> ca_motion;
  estimation::TrackerOptions tracker;
  estimation::TrackerSetOptions targets;
  QueueOptions queue;
  std::size_t history_capacity;
  core::Seconds history_maximum_gap;
  vision::PnpQualityOptions pnp;
  std::map<int, vision::PlateDimensions> plate_sizes;
  vision::CornerMapping corner_mapping;
  vision::RefinementOptions refinement;
  bool refine;
  vision::PoseCovariance additional_pose_covariance;
  decision::PredictionOptions prediction;
  decision::BallisticModel ballistic;
  decision::InterceptLimits intercept;
  decision::ArmorSelectionOptions armor_selection;
  decision::ImpactUncertainty impact;
  decision::MarginOptions margin;
  decision::AimSafetyLimits adequacy;
  mission::TargetSelectionOptions target_selection;
  mission::InfantryModeOptions infantry;
  control::GuardOptions guard;
  control::SmootherOptions smoother;
  core::Seconds publish_period;
  core::Seconds intent_lifetime;
  core::Seconds after_send_delay;
  math::Point3<math::WorldFrame> launch_origin;
  Eigen::Quaterniond aim_reference_to_world;
  core::Evidence control_channel;
  std::map<vision::ArmorSize, vision::PlateDimensions> typed_plate_sizes{};
  std::optional<UartFeedbackOptions> uart_feedback{};
  bool independent_button_input = false; // 缺少新配置时保持旧 enable_event 语义。
  core::Evidence button_evidence = core::Evidence::declared();
};

// 缺少显式尺寸返回空；legacy 配置只按其旧模型编号查表，不推断板型。
const vision::PlateDimensions* plate_dimensions(const PipelineConfig& config,
                                                const vision::Detection& detection);

// 只读文件，不创建线程、检测器推理请求或设备会话；首版入口只接受 replay。
core::Result<PipelineConfig> load_pipeline_config(const std::filesystem::path& path);
} // namespace autoaim::pipeline
