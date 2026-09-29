#pragma once

#include "autoaim/decision/predictor.hpp"

namespace autoaim::decision {
struct ArmorSelectionOptions {
  double minimum_incidence_cosine;
  double direction_weight;
  double flight_time_weight;
  double position_variance_weight;
  double switch_margin;
  core::Seconds minimum_dwell;
};

class ArmorSelector {
public:
  explicit ArmorSelector(ArmorSelectionOptions options);

  // 返回本次候选数组下标，不跨帧保存指向候选的引用，也不产生开火许可。
  // 当前板不可解/不满足入射条件时立即重选，不受正常切换驻留时间阻碍。
  // 候选 estimated_send 是本次决策时刻，必须一致且严格前进；允许复用同一源观测。
  // 重用不会改写 source，观测接纳仍由上游按帧号/世代独立校验。
  std::optional<std::size_t> select(const core::Stamp& source, std::uint64_t target_id,
                                    const std::string& profile_id,
                                    const std::vector<InterceptSolution>& candidates,
                                    const Eigen::Vector3d& measured_aim_direction_world,
                                    const Eigen::Vector3d& gravity_mps2);

  void reset();

private:
  struct Key {
    std::uint64_t target;
    std::string profile;
    std::optional<std::size_t> plate;
  };

  ArmorSelectionOptions options_;
  std::optional<Key> active_;
  std::optional<core::TimePoint> switched_at_;
  std::optional<core::Stamp> last_source_;
  std::optional<core::TimePoint> last_decision_;
};
} // namespace autoaim::decision
