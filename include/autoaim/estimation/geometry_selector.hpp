#pragma once

#include "autoaim/estimation/armor_id.hpp"
#include <memory>

namespace autoaim::estimation {
struct GeometrySelection {
  std::size_t index;
  std::shared_ptr<const GeometryProfile> profile;
  bool supported;
  bool calibrated;
};
class GeometrySelector {
public:
  GeometrySelector(std::vector<std::shared_ptr<const GeometryProfile>> profiles,
      IdentityLimits support, core::Seconds minimum_dwell, double complexity_penalty,
      std::string calibration_id);
  bool observe(const core::Stamp& source, const std::vector<double>& prior_costs);
  // 已选 profile 可保留用于诊断；supported=false 时不能据此推断其余板并开火。
  std::optional<GeometrySelection> selection(core::ClockDomain domain) const;
private:
  std::vector<std::shared_ptr<const GeometryProfile>> profiles_;
  IdentityResolver evidence_;
  core::Seconds minimum_dwell_;
  double complexity_penalty_;
  std::string calibration_id_;
  std::optional<std::size_t> active_;
  std::optional<core::TimePoint> switched_at_;
  bool supported_ = false;
};
}  // namespace autoaim::estimation
