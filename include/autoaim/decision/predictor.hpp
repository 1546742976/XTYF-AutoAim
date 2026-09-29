#pragma once

#include "autoaim/estimation/target_snapshot.hpp"
#include "autoaim/decision/ballistic.hpp"

namespace autoaim::decision {
struct PredictedPlate {
  std::optional<std::size_t> physical_plate;
  math::Transform<math::PlateFrame, math::WorldFrame> pose;
  vision::PoseCovariance covariance;
  vision::PlateDimensions dimensions;
  Eigen::Vector3d velocity_world_mps;
  bool reliable;
};

struct FutureTarget {
  const core::Stamp source;
  const core::TimePoint predicted_at;
  const estimation::TargetState state;
  const estimation::StateCovariance covariance;
  const std::vector<PredictedPlate> plates;

  FutureTarget(core::Stamp stamp, core::TimePoint time, estimation::StatePrediction prediction,
               std::vector<PredictedPlate> predicted_plates)
      : source(stamp), predicted_at(time), state(std::move(prediction.state)),
        covariance(std::move(prediction.covariance)), plates(std::move(predicted_plates)) {
  }
};

struct PredictionOptions {
  double geometry_position_variance_m2;
  double geometry_rotation_variance_rad2;
  double unknown_velocity_sigma_mps;
};

// 只读快照，在副本上按固定模型外推；不会改 Tracker，也不会刷新源曝光时间。
// 身份未知时仅返回当前可见板的保守位置及扩张协方差，不能推断其它板或标为可靠。
core::Result<FutureTarget> predict_future(const estimation::TargetSnapshot& snapshot,
                                          core::TimePoint when, const PredictionOptions& options);

struct InterceptRequest {
  core::TimePoint estimated_send;
  core::Seconds after_send_delay;
  math::Point3<math::WorldFrame> launch_origin;
  core::MetresPerSecond speed;
  std::optional<std::size_t> physical_plate;
};

struct InterceptLimits {
  std::size_t maximum_iterations;
  core::Seconds time_tolerance;
  core::Metres position_tolerance;
};

struct InterceptSolution {
  const core::Stamp source;
  const core::TimePoint estimated_send;
  const core::TimePoint fire_at;
  const core::TimePoint hit_at;
  const BallisticSolution ballistic;
  const PredictedPlate plate;
  const std::size_t iterations;

  InterceptSolution(core::Stamp stamp, core::TimePoint send, core::TimePoint fire,
                    core::TimePoint hit, BallisticSolution trajectory, PredictedPlate target,
                    std::size_t count)
      : source(stamp), estimated_send(send), fire_at(fire), hit_at(hit),
        ballistic(std::move(trajectory)), plate(std::move(target)), iterations(count) {
  }
};

// 有界定点迭代同时解目标未来位置与飞行时间；未收敛/无解显式失败。
// estimated_send 是预测输入，不被记录成真正的主机写入完成时刻。
core::Result<InterceptSolution> solve_intercept(const estimation::TargetSnapshot& snapshot,
                                                const InterceptRequest& request,
                                                const PredictionOptions& prediction,
                                                const BallisticModel& ballistic,
                                                const InterceptLimits& limits);
} // namespace autoaim::decision
