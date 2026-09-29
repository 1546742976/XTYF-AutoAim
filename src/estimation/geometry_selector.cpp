#include "autoaim/estimation/geometry_selector.hpp"
#include <set>

namespace autoaim::estimation {
GeometrySelector::GeometrySelector(std::vector<std::shared_ptr<const GeometryProfile>> profiles,
                                   IdentityLimits support, core::Seconds minimum_dwell,
                                   double complexity_penalty, std::string calibration_id)
    : profiles_(std::move(profiles)), evidence_(profiles_.size(), support),
      minimum_dwell_(minimum_dwell), complexity_penalty_(complexity_penalty),
      calibration_id_(std::move(calibration_id)) {
  if (minimum_dwell.value() < 0 || !std::isfinite(complexity_penalty) || complexity_penalty < 0 ||
      calibration_id_.empty())
    throw std::invalid_argument("Invalid geometry selection limits");

  std::set<std::string> ids;

  for (const auto& profile : profiles_)
    if (!profile || !ids.insert(profile->id).second)
      throw std::invalid_argument("Null/duplicate geometry profile");
}

bool GeometrySelector::observe(const core::Stamp& source, const std::vector<double>& prior_costs) {
  if (prior_costs.size() != profiles_.size())
    throw std::invalid_argument("Geometry cost count mismatch");

  auto costs = prior_costs;

  for (std::size_t i = 0; i < costs.size(); ++i) {
    if (std::isnan(costs[i]) || costs[i] < 0)
      throw std::invalid_argument("Invalid geometry prior cost");

    const auto& profile = *profiles_[i];
    const double height_parameters = profile.layout == HeightLayout::same ? 1
                                     : profile.layout == HeightLayout::paired
                                         ? 2
                                         : profile.plates.size();

    const bool tilted =
        (profile.axis_to_world * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm() > 1e-6;

    costs[i] += complexity_penalty_ * (height_parameters - 1 + (tilted ? 2 : 0));
  }

  if (!evidence_.observe(source, costs))
    return false;

  const auto proposed = evidence_.selected();

  if (proposed && (!active_ || (*proposed != *active_ &&
                                core::elapsed(source.exposure, *switched_at_).value() >=
                                    minimum_dwell_.value()))) {
    active_ = proposed;
    switched_at_ = source.exposure;
  }

  supported_ = proposed && active_ && *proposed == *active_;

  return true;
}

std::optional<GeometrySelection> GeometrySelector::selection(core::ClockDomain domain) const {
  if (!active_)
    return std::nullopt;

  const auto& profile = profiles_[*active_];

  return GeometrySelection{*active_, profile, supported_,
                           profile->evidence.qualifies(domain, profile->id, calibration_id_)};
}
} // namespace autoaim::estimation
