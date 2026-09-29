#include "autoaim/estimation/observation.hpp"
#include "autoaim/math/numeric.hpp"

namespace autoaim::estimation {
Observation::Observation(core::Stamp stamp, vision::Detection pixels, vision::PnpEstimate estimate,
                         std::vector<ObservedPose> candidates, vision::AlignedPose aligned,
                         core::TimeOrigin origin, core::Seconds time_sigma, core::Evidence evidence)
    : source(stamp), detection(std::move(pixels)), pnp(std::move(estimate)),
      world_candidates(std::move(candidates)), historical_pose(std::move(aligned)),
      time_origin(origin), timing_uncertainty(time_sigma), timing_evidence(std::move(evidence)),
      reliable(pnp.pose_reliable && historical_pose.reliable) {
  if (source.frame_id != pnp.source.frame_id || source.generation != pnp.source.generation ||
      !core::same_time(source.exposure, pnp.source.exposure) ||
      !core::same_time(source.exposure, historical_pose.sampled_at) || time_sigma.value() < 0 ||
      world_candidates.size() != pnp.candidates.size() ||
      (pnp.selected && *pnp.selected >= world_candidates.size()) ||
      (pnp.pose_valid && !pnp.selected) ||
      (pnp.pose_reliable && (!pnp.pose_valid || pnp.ambiguous)))
    throw std::invalid_argument("Inconsistent full observation contract");

  for (const auto& candidate : world_candidates)
    if (candidate.covariance && !math::covariance_valid(*candidate.covariance))
      throw std::invalid_argument("Invalid observation covariance");
}

core::Result<Observation> make_observation(const vision::FramePacket& frame,
                                           const vision::Detection& detection,
                                           const vision::PnpEstimate& estimate,
                                           const vision::Calibration& calibration,
                                           const vision::PoseCovariance& additional_covariance) {
  using Result = core::Result<Observation>;

  if (!frame.pose)
    return Result::failure(core::ErrorCode::unavailable, "No exposure-aligned pose");

  if (!math::covariance_valid(additional_covariance) ||
      frame.frame.image.width != calibration.width() ||
      frame.frame.image.height != calibration.height())
    return Result::failure(core::ErrorCode::invalid_input,
                           "Calibration size or added covariance invalid");

  try {
    const auto camera_to_world =
        math::compose(frame.pose->gimbal_to_world, calibration.camera_to_gimbal());

    const auto rotation = camera_to_world.value().rotation().toRotationMatrix();
    vision::PoseCovariance transform = vision::PoseCovariance::Zero();
    transform.topLeftCorner<3, 3>() = rotation;
    transform.bottomRightCorner<3, 3>() = rotation;
    std::vector<ObservedPose> world;

    for (const auto& candidate : estimate.candidates) {
      std::optional<vision::PoseCovariance> covariance;

      if (candidate.covariance)
        covariance =
            transform * *candidate.covariance * transform.transpose() + additional_covariance;

      world.push_back(
          {math::compose(camera_to_world, candidate.plate_to_camera), std::move(covariance)});
    }

    return Result::success(Observation(
        frame.frame.stamp, detection, estimate, std::move(world), *frame.pose,
        frame.frame.time_origin, frame.frame.timing_uncertainty, frame.frame.timing_evidence));
  } catch (const std::exception& error) {
    return Result::failure(core::ErrorCode::invalid_input, error.what());
  }
}
} // namespace autoaim::estimation
