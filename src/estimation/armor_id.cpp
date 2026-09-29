#include "autoaim/estimation/armor_id.hpp"
#include "autoaim/math/angle.hpp"
#include <algorithm>
#include <limits>

namespace autoaim::estimation {
std::vector<PhysicalHypothesis> initial_identity_hypotheses(const ObservedPose& observation,
                                                          const GeometryProfile& profile) {
  std::vector<PhysicalHypothesis> result;
  const auto& measured = observation.plate_to_world.value();
  for (std::size_t i = 0; i < profile.plates.size(); ++i) {
    const auto& plate = profile.plates[i];
    const auto relative = profile.axis_to_world.conjugate() * measured.rotation() * plate.plate_to_axis.conjugate();
    const auto matrix = relative.toRotationMatrix();
    const core::Radians phase(std::atan2(matrix(1, 0), matrix(0, 0)));
    const auto rotating_basis = profile.axis_to_world * Eigen::Quaterniond(Eigen::AngleAxisd(phase.value(), Eigen::Vector3d::UnitZ()));
    const auto predicted_rotation = rotating_basis * plate.plate_to_axis;
    const double error = math::rotation_log(measured.rotation() * predicted_rotation.conjugate()).norm();
    result.push_back({i, {math::Point3<math::WorldFrame>(measured.translation() - rotating_basis * plate.offset_axis_m),
      Eigen::Vector3d::Zero(), phase, 0, 0}, error});
  }
  return result;
}
IdentityResolver::IdentityResolver(std::size_t count, IdentityLimits limits)
    : limits_(limits), scores_(count, 0) {
  if (count == 0 || limits.minimum_observations < 2 || !std::isfinite(limits.minimum_score_margin) ||
      limits.minimum_score_margin <= 0 || !std::isfinite(limits.maximum_cost_per_observation) ||
      limits.maximum_cost_per_observation <= 0 || !std::isfinite(limits.previous_score_weight) ||
      limits.previous_score_weight < 0 || limits.previous_score_weight >= 1)
    throw std::invalid_argument("Invalid identity hypothesis limits");
}
bool IdentityResolver::observe(const core::Stamp& source, const std::vector<double>& costs) {
  if (costs.size() != scores_.size()) throw std::invalid_argument("Identity score count mismatch");
  for (const double cost : costs) if (std::isnan(cost) || cost < 0) throw std::invalid_argument("Invalid identity score");
  if (last_source_ && (source.generation != last_source_->generation || source.frame_id <= last_source_->frame_id ||
      source.exposure.domain() != last_source_->exposure.domain() ||
      core::elapsed(source.exposure, last_source_->exposure).value() <= 0)) return false;
  for (std::size_t i = 0; i < scores_.size(); ++i)
    scores_[i] = limits_.previous_score_weight * scores_[i] +
      (1 - limits_.previous_score_weight) * std::min(costs[i], limits_.maximum_cost_per_observation);
  const auto best = static_cast<std::size_t>(std::min_element(scores_.begin(), scores_.end()) - scores_.begin());
  double second = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scores_.size(); ++i) if (i != best) second = std::min(second, scores_[i]);
  if (second - scores_[best] < limits_.minimum_score_margin || costs[best] >= limits_.maximum_cost_per_observation) {
    winner_.reset(); support_ = 0;
  } else {
    if (!winner_ || *winner_ != best) { winner_ = best; support_ = 0; }
    if (support_ < limits_.minimum_observations) ++support_;
  }
  last_source_.emplace(source);
  return true;
}
std::optional<std::size_t> IdentityResolver::selected() const noexcept {
  if (support_ < limits_.minimum_observations) return std::nullopt;
  return winner_;
}
}  // namespace autoaim::estimation
