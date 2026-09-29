#pragma once

#include "autoaim/core/types.hpp"
#include "autoaim/estimation/motion_model.hpp"
#include "autoaim/estimation/geometry_model.hpp"
#include "autoaim/estimation/observation.hpp"
#include "autoaim/math/numeric.hpp"
#include "autoaim/vision/frame_packet.hpp"
#include <memory>
#include <optional>

namespace autoaim::estimation {
enum class TrackingQuality { unconverged, converged, degraded };

struct TargetSnapshot {
  const core::Stamp source;
  const core::TimePoint state_time;
  const std::uint64_t target_id;
  const TargetState state;
  const StateCovariance covariance;
  const std::optional<std::size_t> physical_plate;
  const TrackingQuality quality;
  const bool pose_reliable;
  const std::optional<vision::AlignedPose> historical_pose;
  const std::shared_ptr<const MotionModel> motion;
  const std::shared_ptr<const GeometryProfile> geometry;
  const ObservedPose visible_plate;
  const vision::PlateDimensions visible_dimensions;
  const core::TimeOrigin time_origin;
  const core::Seconds timing_uncertainty;
  const core::Evidence timing_evidence;

  TargetSnapshot(core::Stamp stamp, core::TimePoint time, std::uint64_t id,
      TargetState estimate, StateCovariance uncertainty, std::optional<std::size_t> plate,
      TrackingQuality health, bool pose_quality, std::optional<vision::AlignedPose> aligned_pose,
      std::shared_ptr<const MotionModel> model,
      std::shared_ptr<const GeometryProfile> profile, ObservedPose visible, vision::PlateDimensions dimensions,
      core::TimeOrigin origin, core::Seconds time_sigma, core::Evidence time_evidence)
      : source(stamp), state_time(time), target_id(id), state(std::move(estimate)),
        covariance(std::move(uncertainty)), physical_plate(plate), quality(health),
        pose_reliable(pose_quality), historical_pose(std::move(aligned_pose)),
        motion(std::move(model)), geometry(std::move(profile)), visible_plate(std::move(visible)), visible_dimensions(dimensions),
        time_origin(origin), timing_uncertainty(time_sigma), timing_evidence(std::move(time_evidence)) {
    if (!motion || !geometry || id == 0 || core::elapsed(time, stamp.exposure).value() < 0 ||
        !state.velocity_mps.allFinite() || !std::isfinite(state.omega_radps) ||
        !std::isfinite(state.alpha_radps2) || !math::covariance_valid(covariance))
      throw std::invalid_argument("Invalid target snapshot");
    if ((historical_pose && !core::same_time(historical_pose->sampled_at, source.exposure)) ||
        (pose_reliable && (!historical_pose || !historical_pose->reliable)))
      throw std::invalid_argument("Snapshot historical pose does not match source");
    if (time_sigma.value() < 0 || (physical_plate && *physical_plate >= geometry->plates.size()) ||
        (quality == TrackingQuality::converged && !physical_plate))
      throw std::invalid_argument("Snapshot identity/timing contract invalid");
  }
};
}  // namespace autoaim::estimation
