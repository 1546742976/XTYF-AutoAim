#include "autoaim/decision/armor_selector.hpp"
#include "autoaim/math/numeric.hpp"
#include <algorithm>
#include <limits>

namespace autoaim::decision {
ArmorSelector::ArmorSelector(ArmorSelectionOptions options) : options_(options) {
  for (const double value :
       {options.direction_weight, options.flight_time_weight, options.position_variance_weight})
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("Invalid armor selection weight");

  if (!std::isfinite(options.minimum_incidence_cosine) || options.minimum_incidence_cosine <= 0 ||
      options.minimum_incidence_cosine > 1 || !std::isfinite(options.switch_margin) ||
      options.switch_margin <= 0 || options.minimum_dwell.value() < 0)
    throw std::invalid_argument("Invalid armor selection hysteresis");
}

void ArmorSelector::reset() {
  active_.reset();
  switched_at_.reset();
  last_source_.reset();
}

std::optional<std::size_t> ArmorSelector::select(const core::Stamp& source, std::uint64_t target_id,
                                                 const std::string& profile_id,
                                                 const std::vector<InterceptSolution>& candidates,
                                                 const Eigen::Vector3d& measured_aim,
                                                 const Eigen::Vector3d& gravity) {
  if (last_source_ && source.generation > last_source_->generation)
    reset();

  if (last_source_ &&
      (source.generation != last_source_->generation || source.frame_id <= last_source_->frame_id ||
       source.exposure.domain() != last_source_->exposure.domain() ||
       core::elapsed(source.exposure, last_source_->exposure).value() <= 0))
    return std::nullopt;

  last_source_.emplace(source);

  if (!target_id || profile_id.empty() || !measured_aim.allFinite() || measured_aim.norm() <= 0 ||
      !gravity.allFinite()) {
    active_.reset();

    return std::nullopt;
  }

  const auto aim = measured_aim.normalized();
  std::vector<double> costs(candidates.size(), std::numeric_limits<double>::infinity());
  std::optional<std::size_t> best, previous;

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto& candidate = candidates[i];

    if (candidate.source.frame_id != source.frame_id ||
        candidate.source.generation != source.generation ||
        !core::same_time(candidate.source.exposure, source.exposure) ||
        !candidate.ballistic.initial_velocity_world_mps.allFinite() ||
        candidate.ballistic.flight_time.value() <= 0 ||
        !math::covariance_valid(candidate.plate.covariance))
      continue;

    const Eigen::Vector3d impact_velocity = candidate.ballistic.initial_velocity_world_mps +
                                            gravity * candidate.ballistic.flight_time.value();

    if (!impact_velocity.allFinite() || impact_velocity.norm() <= 0 ||
        candidate.ballistic.initial_velocity_world_mps.norm() <= 0)
      continue;

    const auto normal = candidate.plate.pose.value().rotation() * Eigen::Vector3d::UnitZ();

    if (normal.dot(-impact_velocity.normalized()) < options_.minimum_incidence_cosine)
      continue;

    const auto direction = candidate.ballistic.initial_velocity_world_mps.normalized();
    const double angle = std::atan2(aim.cross(direction).norm(), aim.dot(direction));
    costs[i] = options_.direction_weight * angle +
               options_.flight_time_weight * candidate.ballistic.flight_time.value() +
               options_.position_variance_weight *
                   candidate.plate.covariance.topLeftCorner<3, 3>().trace();

    if (!best || costs[i] < costs[*best])
      best = i;

    if (active_ && active_->target == target_id && active_->profile == profile_id &&
        active_->plate == candidate.plate.physical_plate)
      previous = i;
  }

  if (!best) {
    active_.reset();

    return std::nullopt;
  }

  if (previous && *previous != *best &&
      (costs[*previous] - costs[*best] < options_.switch_margin ||
       core::elapsed(source.exposure, *switched_at_).value() < options_.minimum_dwell.value()))
    best = previous;

  if (!previous || *previous != *best) {
    active_ = Key{target_id, profile_id, candidates[*best].plate.physical_plate};
    switched_at_ = source.exposure;
  }

  return best;
}
} // namespace autoaim::decision
