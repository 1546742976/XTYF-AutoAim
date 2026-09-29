#include "autoaim/decision/predictor.hpp"
#include "autoaim/math/numeric.hpp"

namespace autoaim::decision {
core::Result<FutureTarget> predict_future(const estimation::TargetSnapshot& snapshot,
    core::TimePoint when, const PredictionOptions& options) {
  using Result = core::Result<FutureTarget>;
  for (const double value : {options.geometry_position_variance_m2, options.geometry_rotation_variance_rad2,
                             options.unknown_velocity_sigma_mps})
    if (!std::isfinite(value) || value < 0) return Result::failure(core::ErrorCode::invalid_input, "Invalid prediction uncertainty");
  if (when.domain() != snapshot.state_time.domain() || core::elapsed(when, snapshot.state_time).value() < 0)
    return Result::failure(core::ErrorCode::invalid_input, "Prediction time precedes state or uses wrong clock");
  const auto dt = core::elapsed(when, snapshot.state_time);
  auto propagated = snapshot.motion->propagate(snapshot.state, snapshot.covariance, dt);
  if (!propagated) return Result::failure(propagated.error().code, propagated.error().message);
  std::vector<PredictedPlate> plates;
  const bool known = snapshot.physical_plate && snapshot.pose_reliable && snapshot.quality == estimation::TrackingQuality::converged;
  if (!known) {
    if (!snapshot.visible_plate.covariance)
      return Result::failure(core::ErrorCode::unavailable, "Visible plate has no usable covariance");
    auto covariance = *snapshot.visible_plate.covariance;
    covariance.topLeftCorner<3, 3>().diagonal().array() +=
      std::pow(options.unknown_velocity_sigma_mps * core::elapsed(when, snapshot.source.exposure).value(), 2) +
      options.geometry_position_variance_m2;
    covariance.bottomRightCorner<3, 3>().diagonal().array() += options.geometry_rotation_variance_rad2;
    plates.push_back({std::nullopt, snapshot.visible_plate.plate_to_world, covariance, snapshot.visible_dimensions,
      Eigen::Vector3d::Zero(), false});
  } else {
    const auto& predicted = propagated.value();
    const Eigen::Vector3d axis = snapshot.geometry->axis_to_world * Eigen::Vector3d::UnitZ();
    for (std::size_t i = 0; i < snapshot.geometry->plates.size(); ++i) {
      const auto pose = snapshot.geometry->plate_pose(predicted.state.center, predicted.state.phase, i);
      Eigen::Matrix<double, 6, estimation::state_dimension> jacobian =
        Eigen::Matrix<double, 6, estimation::state_dimension>::Zero();
      jacobian.block<3, 3>(0, estimation::component_index(estimation::StateComponent::x)).setIdentity();
      constexpr int phase = estimation::component_index(estimation::StateComponent::phase);
      jacobian.col(phase).head<3>() = axis.cross(pose.value().translation() - predicted.state.center.metres());
      jacobian.col(phase).tail<3>() = axis;
      vision::PoseCovariance covariance = jacobian * predicted.covariance * jacobian.transpose();
      covariance.topLeftCorner<3, 3>().diagonal().array() += options.geometry_position_variance_m2;
      covariance.bottomRightCorner<3, 3>().diagonal().array() += options.geometry_rotation_variance_rad2;
      const Eigen::Vector3d velocity = predicted.state.velocity_mps + predicted.state.omega_radps *
        axis.cross(pose.value().translation() - predicted.state.center.metres());
      plates.push_back({i, pose, covariance, snapshot.geometry->plates[i].dimensions, velocity, true});
    }
  }
  for (const auto& plate : plates) if (!math::covariance_valid(plate.covariance))
    return Result::failure(core::ErrorCode::invalid_input, "Predicted plate covariance invalid");
  return Result::success(FutureTarget(snapshot.source, when, std::move(propagated).value(), std::move(plates)));
}

namespace {
const PredictedPlate* find_plate(const FutureTarget& target, const std::optional<std::size_t>& index) {
  for (const auto& plate : target.plates) if (plate.physical_plate == index) return &plate;
  return nullptr;
}
}  // namespace
core::Result<InterceptSolution> solve_intercept(const estimation::TargetSnapshot& snapshot,
    const InterceptRequest& request, const PredictionOptions& prediction_options,
    const BallisticModel& ballistic, const InterceptLimits& limits) {
  using Result = core::Result<InterceptSolution>;
  if (request.speed.value() <= 0 || request.after_send_delay.value() < 0 || limits.maximum_iterations == 0 ||
      limits.maximum_iterations > 64 || limits.time_tolerance.value() <= 0 || limits.position_tolerance.value() <= 0)
    return Result::failure(core::ErrorCode::invalid_input, "Invalid intercept limits/speed/delay");
  try {
    const auto fire = core::advance(request.estimated_send, request.after_send_delay);
    auto initial = predict_future(snapshot, fire, prediction_options);
    if (!initial) return Result::failure(initial.error().code, initial.error().message);
    const auto* first = find_plate(initial.value(), request.physical_plate);
    if (!first) return Result::failure(core::ErrorCode::unavailable, "Requested physical plate is not identifiable");
    auto seed = ballistic.solve(request.launch_origin, math::Point3<math::WorldFrame>(first->pose.value().translation()), request.speed);
    if (!seed) return Result::failure(seed.error().code, seed.error().message);
    auto flight_time = seed.value().flight_time;
    for (std::size_t iteration = 1; iteration <= limits.maximum_iterations; ++iteration) {
      const auto hit = core::advance(fire, flight_time);
      const auto future = predict_future(snapshot, hit, prediction_options);
      if (!future) return Result::failure(future.error().code, future.error().message);
      const auto* target = find_plate(future.value(), request.physical_plate);
      if (!target) return Result::failure(core::ErrorCode::unavailable, "Plate unavailable during intercept");
      auto solution = ballistic.solve(request.launch_origin,
        math::Point3<math::WorldFrame>(target->pose.value().translation()), request.speed);
      if (!solution) return Result::failure(solution.error().code, solution.error().message);
      const double change = std::abs(solution.value().flight_time.value() - flight_time.value());
      flight_time = solution.value().flight_time;
      if (change > limits.time_tolerance.value()) continue;
      const auto final_hit = core::advance(fire, flight_time);
      const auto final_future = predict_future(snapshot, final_hit, prediction_options);
      if (!final_future) return Result::failure(final_future.error().code, final_future.error().message);
      const auto* final_plate = find_plate(final_future.value(), request.physical_plate);
      if (!final_plate) return Result::failure(core::ErrorCode::unavailable, "Final plate unavailable");
      const auto point = ballistic.position(request.launch_origin, solution.value().initial_velocity_world_mps, flight_time);
      const auto local = final_plate->pose.value().inverse().apply(point.metres());
      if ((point.metres() - final_plate->pose.value().translation()).norm() > limits.position_tolerance.value()) continue;
      if (std::abs(local.x()) >= final_plate->dimensions.width.value() / 2 ||
          std::abs(local.y()) >= final_plate->dimensions.height.value() / 2)
        return Result::failure(core::ErrorCode::unavailable, "Nominal intercept outside plate polygon");
      return Result::success(InterceptSolution(snapshot.source, request.estimated_send, fire, final_hit,
        solution.value(), *final_plate, iteration));
    }
    return Result::failure(core::ErrorCode::unavailable, "Intercept iteration did not converge");
  } catch (const std::exception& error) { return Result::failure(core::ErrorCode::invalid_input, error.what()); }
}
}  // namespace autoaim::decision
