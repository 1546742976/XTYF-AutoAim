#pragma once

#include "autoaim/estimation/target_snapshot.hpp"

namespace autoaim::mission {
struct TargetSelectionOptions {
  core::Seconds maximum_source_age;
  core::Seconds lost_timeout;
  double maximum_position_variance_m2;
};
// 单线程；持有目标 id 和最后有效曝光时间，不以重复消费延长锁定。
class TargetSelector {
public:
  explicit TargetSelector(TargetSelectionOptions options);
  std::shared_ptr<const estimation::TargetSnapshot> select(
      const std::vector<std::shared_ptr<const estimation::TargetSnapshot>>& targets,
      core::Generation generation, core::TimePoint now,
      const Eigen::Vector3d& origin_world_m, const Eigen::Vector3d& actual_direction_world);
  void reset();
  std::optional<std::uint64_t> locked_target() const { return locked_; }
private:
  TargetSelectionOptions options_;
  std::optional<std::uint64_t> locked_;
  std::optional<core::TimePoint> last_valid_;
  std::optional<core::Generation> generation_;
};
}  // namespace autoaim::mission
