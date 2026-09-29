#include "autoaim/mission/target_selector.hpp"
#include <algorithm>
#include <limits>

namespace autoaim::mission {
TargetSelector::TargetSelector(TargetSelectionOptions options) : options_(options) {
  if (options.maximum_source_age.value() <= 0 ||
      options.lost_timeout.value() < options.maximum_source_age.value() ||
      !std::isfinite(options.maximum_position_variance_m2) ||
      options.maximum_position_variance_m2 <= 0)
    throw std::invalid_argument("Invalid target selection limits");
}

void TargetSelector::reset() {
  locked_.reset();
  last_valid_.reset();
  generation_.reset();
}

std::shared_ptr<const estimation::TargetSnapshot> TargetSelector::select(
    const std::vector<std::shared_ptr<const estimation::TargetSnapshot>>& targets,
    core::Generation generation, core::TimePoint now, const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction) {
  if (generation_ && generation < *generation_)
    return {};

  if (!generation_ || generation != *generation_) {
    reset();
    generation_ = generation;
  }

  if (!origin.allFinite() || !direction.allFinite() || direction.norm() < 1e-12)
    return {};

  std::shared_ptr<const estimation::TargetSnapshot> best;
  double best_angle = std::numeric_limits<double>::infinity();

  for (const auto& target : targets) {
    // 未知物理身份仍可跟随真实可见板；绝不因此猜测另一块板或获得开火权。
    if (!target || target->source.generation != generation ||
        !core::fresh(target->source.exposure, now, options_.maximum_source_age) ||
        !target->historical_pose || !target->historical_pose->reliable ||
        !target->visible_plate.covariance ||
        !math::covariance_valid(*target->visible_plate.covariance) ||
        target->visible_plate.covariance->topLeftCorner<3, 3>().diagonal().maxCoeff() >
            options_.maximum_position_variance_m2)
      continue;

    const Eigen::Vector3d offset =
        target->visible_plate.plate_to_world.value().translation() - origin;

    if (offset.norm() < 1e-9 || offset.dot(direction) <= 0)
      continue;

    if (locked_ && target->target_id == *locked_) {
      if (!last_valid_ || target->source.exposure.nanoseconds() > last_valid_->nanoseconds())
        last_valid_ = target->source.exposure;

      return target;
    }

    const double angle =
        std::acos(std::clamp(offset.normalized().dot(direction.normalized()), -1.0, 1.0));

    if (angle < best_angle ||
        (angle == best_angle && best && target->target_id < best->target_id)) {
      best = target;
      best_angle = angle;
    }
  }

  if (locked_ && last_valid_ && core::fresh(*last_valid_, now, options_.lost_timeout))
    return {};

  locked_.reset();
  last_valid_.reset();

  if (best) {
    locked_ = best->target_id;
    last_valid_ = best->source.exposure;
  }

  return best;
}
} // namespace autoaim::mission
