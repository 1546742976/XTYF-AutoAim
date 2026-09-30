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
struct ConfigurationFileSnapshot {
  std::filesystem::path path;
  std::string contents;
};

struct PipelineConfigurationSnapshot {
  ConfigurationFileSnapshot entry;
  ConfigurationFileSnapshot base;
  std::optional<ConfigurationFileSnapshot> fast_choose;
  YAML::Node effective;
};

// bootstrap 合成 config/fast_choose.yaml 的日常参数与基础场景契约，再显式装配。
// 基础配置保留标定、几何、类别和角点声明；旧完整配置仍可直接装载。
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
  std::shared_ptr<const PipelineConfigurationSnapshot> configuration_snapshot{};
};

// 缺少显式尺寸返回空；legacy 配置只按其旧模型编号查表，不推断板型。
const vision::PlateDimensions* plate_dimensions(const PipelineConfig& config,
                                                const vision::Detection& detection);

// 只读文件，不创建线程、检测器推理请求或设备会话；首版入口只接受 replay。
core::Result<PipelineConfig> load_pipeline_config(const std::filesystem::path& path);
// 一次预加载共用同一快调文件的字节快照；缓存仅属于本次调用。
core::Result<std::vector<PipelineConfig>>
load_pipeline_configs(const std::vector<std::filesystem::path>& paths);
} // namespace autoaim::pipeline
