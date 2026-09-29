#pragma once

#include "support/tracker_fixture.hpp"
#include "support/geometry_fixture.hpp"

namespace test {
inline std::shared_ptr<const autoaim::estimation::TargetSnapshot> snapshot(bool known = true) {
  using namespace autoaim;
  auto profile = std::make_shared<const estimation::GeometryProfile>(geometry(4, estimation::HeightLayout::distinct, true));
  const core::Stamp source(10, 1, core::TimePoint(100000000, core::ClockDomain::replay));
  const estimation::TargetState state{math::Point3<math::WorldFrame>({3, 0, 1}), {0.1, 0, 0}, core::Radians(0.2), 1, 0};
  const auto observed = observation(*profile, source, state.center, state.phase, 2);
  return std::make_shared<const estimation::TargetSnapshot>(source, source.exposure, 1, state,
    estimation::StateCovariance::Identity() * 0.001, known ? std::optional<std::size_t>(2) : std::nullopt,
    known ? estimation::TrackingQuality::converged : estimation::TrackingQuality::degraded, known,
    observed->historical_pose, std::make_shared<const estimation::MotionModel>(0.01, 0.01, core::Seconds(1)),
    profile, observed->world_candidates[0], observed->pnp.dimensions, observed->time_origin,
    observed->timing_uncertainty, observed->timing_evidence);
}
}  // namespace test
