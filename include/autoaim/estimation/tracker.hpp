#pragma once

#include "autoaim/estimation/association.hpp"
#include "autoaim/estimation/geometry_selector.hpp"
#include "autoaim/estimation/state_machine.hpp"

namespace autoaim::estimation {
struct TrackerOptions {
  std::size_t maximum_hypotheses;
  StateCovariance initial_covariance;
  IdentityLimits identity;
  IdentityLimits geometry;
  TrackingLimits tracking;
  HealthLimits health;
  MotionSelectionLimits motion_selection;
  NisGate nis;
  core::Seconds geometry_minimum_dwell;
  double geometry_complexity_penalty;
  double association_margin;
  double initial_alpha_variance;
  std::string calibration_id;
  EsoOptions eso{};
};

using ObservationBatch = std::vector<std::shared_ptr<const Observation>>;

// 一个实例表示一个候选目标。多目标分配在调用者的目标集合协调器中完成。
// 所有方法单写者调用；批次必须来自同一曝光且已通过 pipeline 的统一结果接纳。
class Tracker {
public:
  Tracker(std::uint64_t target_id, std::vector<std::shared_ptr<const GeometryProfile>> profiles,
          std::shared_ptr<const MotionModel> constant_velocity,
          std::shared_ptr<const MotionModel> bounded_acceleration, TrackerOptions options);

  ~Tracker();
  core::Result<bool> update(const ObservationBatch& observations);

  // 只在副本上预测到查询观测时刻，不修改跟踪器。无可用关联时返回 +infinity。
  double association_cost(const Observation& observation) const;
  std::shared_ptr<const TargetSnapshot> snapshot(core::TimePoint now);
  std::size_t hypothesis_count() const noexcept;
  std::uint64_t id() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

struct TrackerSetOptions {
  std::size_t maximum_targets;
  double association_margin;
  core::Metres maximum_same_target_separation;
};

// 有界多目标集合；同一观测最多交给一个 Tracker，歧义观测不另造重复目标。
class TrackerSet {
public:
  TrackerSet(core::Generation generation,
             std::vector<std::shared_ptr<const GeometryProfile>> profiles,
             std::shared_ptr<const MotionModel> cv, std::shared_ptr<const MotionModel> ca,
             TrackerOptions tracker_options, TrackerSetOptions options);

  ~TrackerSet();
  core::Result<bool> update(const core::Stamp& source, const ObservationBatch& observations);
  std::vector<std::shared_ptr<const TargetSnapshot>> snapshots(core::TimePoint now);
  void reset(core::Generation next_generation);
  std::size_t dropped_observations() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace autoaim::estimation
